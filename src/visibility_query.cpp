#include "detail/runtime.h"
#include <new>

namespace physx_pack
{
namespace
{
namespace vis = visibility;
bool NormalizeRay(const Ray &Input, vis::Ray &Output) noexcept
{
  if (!detail::Finite(Input.Origin) || !detail::Finite(Input.Direction) ||
      !std::isfinite(Input.MaxDistance) || !(Input.MaxDistance > 0))
    return false;
  const double Length =
      std::hypot(double(Input.Direction.X), double(Input.Direction.Y), double(Input.Direction.Z));
  if (!(Length > 0) || !std::isfinite(Length))
    return false;
  Output.Origin = detail::Vector(Input.Origin);
  Output.Direction = {static_cast<float>(Input.Direction.X / Length),
                      static_cast<float>(Input.Direction.Y / Length),
                      static_cast<float>(Input.Direction.Z / Length)};
  Output.MaxDistance = Input.MaxDistance;
  return true;
}
void TransformGeometry(const InstanceDesc &D, vis::GeometryView &G) noexcept
{
  G.WorldPose = detail::GeometryPose(D.WorldPose);
  if (G.Type == vis::GeometryType::TriangleMesh)
  {
    G.TriangleMesh.Scale.Scale = detail::Vector(D.Scale);
    G.TriangleMesh.DoubleSided = D.DoubleSided;
  }
  else if (G.Type == vis::GeometryType::ConvexMesh)
    G.Convex.Scale.Scale = detail::Vector(D.Scale);
  else
  {
    auto &H = G.HeightField;
    H.RowScale = D.Scale.X;
    H.HeightScale = D.Scale.Y;
    H.ColumnScale = D.Scale.Z;
    H.DoubleSided = D.DoubleSided;
    if (H.LocalBoundsDefined)
    {
      H.LocalBounds = detail::ScaledBounds(H.LocalBounds, D.Scale);
      H.LocalBoundsDefined = detail::ValidBounds(H.LocalBounds);
    }
  }
}
} // namespace

struct QueryContext::Impl
{
  detail::GeometryReader Lease;
  visibility::QueryScratch Scratch;
  std::vector<std::uint32_t> Stack;
  QueryStats Stats;

  struct TraceResult
  {
    visibility::RayHit Hit;
    Error Failure = Error::None;
  };

  TraceResult Trace(detail::PackageData &Package, const detail::PreparedInstance &Instance,
                    const visibility::Ray &Ray)
  {
    TraceResult Result;
    if (Instance.ResourceError != Error::None && !detail::Transient(Instance.ResourceError))
    {
      Result.Failure = Instance.ResourceError;
      return Result;
    }
    visibility::GeometryView View{};
    Result.Failure = Package.Geometry.Bind(Instance.Slot, Instance.Type, Lease, &View);
    if (Result.Failure != Error::None)
      return Result;
    TransformGeometry(Instance.Desc, View);
    visibility::QueryControl Control{};
    Control.Scratch = &Scratch;
    ++Stats.GeometryTests;
    Result.Hit = visibility::RaycastGeometry(Ray, View, Instance.Desc.DoubleSided, &Control);
    if (Result.Hit.Status == visibility::QueryStatus::Undefined)
    {
      Result.Failure = Lease.LastError();
      if (Result.Failure == Error::None)
        Result.Failure = Control.Failure;
      if (Result.Failure == Error::None)
        Result.Failure = Error::CorruptData;
    }
    Lease.Reset();
    return Result;
  }

  bool Intersects(const visibility::Ray &Ray, const visibility::Aabb &Bounds, float Limit) noexcept
  {
    ++Stats.BoundsTests;
    // Even a tiny nonzero direction can cross a slab on a long ray. The
    // mesh traversal's near-parallel shortcut is not safe for scene culling.
    const double Origin[3]{Ray.Origin.X, Ray.Origin.Y, Ray.Origin.Z};
    const double Direction[3]{Ray.Direction.X, Ray.Direction.Y, Ray.Direction.Z};
    const double Minimum[3]{Bounds.Minimum.X, Bounds.Minimum.Y, Bounds.Minimum.Z};
    const double Maximum[3]{Bounds.Maximum.X, Bounds.Maximum.Y, Bounds.Maximum.Z};
    const double Magnitude = std::max({1.0, std::abs(Origin[0]), std::abs(Origin[1]),
                                       std::abs(Origin[2]), std::abs(double(Limit))});
    const double Roundoff = 16 * std::numeric_limits<float>::epsilon() * Magnitude;
    double Near = 0, Far = double(Limit) + Roundoff;
    for (unsigned Axis = 0; Axis < 3; ++Axis)
    {
      if (Direction[Axis] == 0)
      {
        if (Origin[Axis] < Minimum[Axis] - Roundoff || Origin[Axis] > Maximum[Axis] + Roundoff)
          return false;
        continue;
      }
      double A = (Minimum[Axis] - Roundoff - Origin[Axis]) / Direction[Axis];
      double B = (Maximum[Axis] + Roundoff - Origin[Axis]) / Direction[Axis];
      if (!std::isfinite(A) || !std::isfinite(B))
        return true;
      if (A > B)
        std::swap(A, B);
      Near = std::max(Near, A);
      Far = std::min(Far, B);
      if (Near > Far)
        return false;
    }
    return true;
  }

  VisibilityResult Check(const detail::SceneData *Scene, const QueryRequest &Request)
  {
    ++Stats.Queries;
    VisibilityResult Result;
    if (!Scene)
      return Result;
    visibility::Ray Ray{};
    if (!Request.Target || !NormalizeRay(Request.QueryRay, Ray))
    {
      Result.Reason = Error::InvalidArgument;
      return Result;
    }
    const auto Target =
        std::lower_bound(Scene->Instances.begin(), Scene->Instances.end(), Request.Target,
                         [](const auto &I, InstanceId Id) { return I.Desc.Id < Id; });
    if (Target == Scene->Instances.end() || Target->Desc.Id != Request.Target)
    {
      Result.Reason = Error::TargetNotFound;
      Result.ProblemInstance = Request.Target;
      return Result;
    }
    const auto TargetHit = Trace(*Scene->Package, *Target, Ray);
    if (TargetHit.Failure != Error::None)
    {
      Result.Reason = TargetHit.Failure;
      Result.ProblemInstance = Request.Target;
      return Result;
    }
    if (TargetHit.Hit.Status != visibility::QueryStatus::Hit)
    {
      Result.Reason = Error::TargetNotHit;
      Result.ProblemInstance = Request.Target;
      return Result;
    }
    Result.TargetDistance = TargetHit.Hit.Distance;
    Error Unknown = Error::None;
    InstanceId Problem = 0;
    float Limit = Result.TargetDistance;
    auto Visit = [&](std::uint32_t Index)
    {
      const auto &Instance = Scene->Instances[Index];
      if (Instance.Desc.Id == Request.Target)
        return;
      if (Instance.WorldBoundsDefined && !Intersects(Ray, Instance.WorldBounds, Limit))
        return;
      const auto Hit = Trace(*Scene->Package, Instance, Ray);
      if (Hit.Failure != Error::None)
      {
        if (!Problem || Instance.Desc.Id < Problem)
        {
          Unknown = Hit.Failure;
          Problem = Instance.Desc.Id;
        }
      }
      else if (Hit.Hit.Status == visibility::QueryStatus::Hit && Hit.Hit.Distance <= Limit)
      {
        if (!Result.BlockingInstance || Hit.Hit.Distance < Result.BlockingDistance ||
            Instance.Desc.Id < Result.BlockingInstance)
        {
          Result.BlockingInstance = Instance.Desc.Id;
          Result.BlockingDistance = Hit.Hit.Distance;
          Limit = Hit.Hit.Distance;
        }
      }
    };
    Stack.clear();
    if (!Scene->Nodes.empty())
      Stack.push_back(0);
    while (!Stack.empty())
    {
      const auto Index = Stack.back();
      Stack.pop_back();
      const auto &Node = Scene->Nodes[Index];
      if (!Intersects(Ray, Node.Bounds, Limit))
        continue;
      if (Node.Count)
      {
        for (std::uint32_t i = 0; i < Node.Count; ++i)
          Visit(Scene->Order[Node.Begin + i]);
      }
      else
      {
        Stack.push_back(Node.Right);
        Stack.push_back(Node.Left);
      }
    }
    for (const auto Index : Scene->Unbounded)
      Visit(Index);
    if (Result.BlockingInstance)
    {
      Result.State = VisibilityState::Blocked;
      Result.Reason = Error::None;
    }
    else if (Unknown != Error::None)
    {
      Result.Reason = Unknown;
      Result.ProblemInstance = Problem;
    }
    else
    {
      Result.State = VisibilityState::Visible;
      Result.Reason = Error::None;
    }
    return Result;
  }
};

QueryContext::QueryContext() : Impl_(std::make_unique<Impl>()) {}
QueryContext::~QueryContext() noexcept = default;
VisibilityResult QueryContext::Check(const SceneSnapshot &Snapshot,
                                     const QueryRequest &Request) noexcept
{
  try
  {
    return Impl_->Check(Snapshot.Data_.get(), Request);
  }
  catch (const std::bad_alloc &)
  {
    Impl_->Lease.Reset();
    VisibilityResult R;
    R.Reason = Error::OutOfMemory;
    return R;
  }
  catch (...)
  {
    Impl_->Lease.Reset();
    VisibilityResult R;
    R.Reason = Error::CorruptData;
    return R;
  }
}
Error QueryContext::CheckBatch(const SceneSnapshot &Snapshot, const QueryRequest *Requests,
                               std::size_t Count, VisibilityResult *Results) noexcept
{
  if (Count && (!Requests || !Results))
    return Error::InvalidArgument;
  for (std::size_t i = 0; i < Count; ++i)
    Results[i] = Check(Snapshot, Requests[i]);
  return Error::None;
}

namespace
{
template <typename ImplType>
RaycastResult TraceSceneRay(ImplType &Impl, const detail::SceneData *Scene,
                            const RaycastRequest &Request)
{
  RaycastResult Result;
  if (!Scene)
    return Result;
  visibility::Ray RayValue{};
  if (!NormalizeRay(Request.QueryRay, RayValue))
  {
    Result.Reason = Error::InvalidArgument;
    return Result;
  }
  Error Unknown = Error::None;
  InstanceId UnknownInstance = 0;
  float Best = Request.QueryRay.MaxDistance;
  auto Visit = [&](std::uint32_t Index)
  {
    const auto &Instance = Scene->Instances[Index];
    if (Request.IgnoreInstance && Instance.Desc.Id == Request.IgnoreInstance)
      return;
    if (Instance.WorldBoundsDefined && !Impl.Intersects(RayValue, Instance.WorldBounds, Best))
      return;
    const auto Hit = Impl.Trace(*Scene->Package, Instance, RayValue);
    if (Hit.Failure != Error::None)
    {
      if (Unknown == Error::None || Instance.Desc.Id < UnknownInstance)
      {
        Unknown = Hit.Failure;
        UnknownInstance = Instance.Desc.Id;
      }
      return;
    }
    if (Hit.Hit.Status == visibility::QueryStatus::Hit && Hit.Hit.Distance <= Best)
    {
      if (Result.State != RaycastState::Hit || Hit.Hit.Distance < Result.Distance ||
          (Hit.Hit.Distance == Result.Distance && Instance.Desc.Id < Result.HitInstance))
      {
        Result.State = RaycastState::Hit;
        Result.Reason = Error::None;
        Result.Distance = Hit.Hit.Distance;
        Result.HitInstance = Instance.Desc.Id;
        Best = Hit.Hit.Distance;
      }
    }
  };
  Impl.Stack.clear();
  if (!Scene->Nodes.empty())
    Impl.Stack.push_back(0);
  while (!Impl.Stack.empty())
  {
    const auto Index = Impl.Stack.back();
    Impl.Stack.pop_back();
    const auto &Node = Scene->Nodes[Index];
    if (!Impl.Intersects(RayValue, Node.Bounds, Best))
      continue;
    if (Node.Count)
      for (std::uint32_t I = 0; I < Node.Count; ++I)
        Visit(Scene->Order[Node.Begin + I]);
    else
    {
      Impl.Stack.push_back(Node.Right);
      Impl.Stack.push_back(Node.Left);
    }
  }
  for (const auto Index : Scene->Unbounded)
    Visit(Index);
  if (Result.State == RaycastState::Hit)
    return Result;
  if (Unknown != Error::None)
  {
    Result.State = RaycastState::Unknown;
    Result.Reason = Unknown;
    Result.HitInstance = UnknownInstance;
    return Result;
  }
  Result.State = RaycastState::Miss;
  Result.Reason = Error::None;
  return Result;
}
} // namespace

Error QueryContext::TraceBatch(const SceneSnapshot &Snapshot, const RaycastRequest *Requests,
                               std::size_t Count, RaycastResult *Results) noexcept
{
  if (Count && (!Requests || !Results))
    return Error::InvalidArgument;
  try
  {
    for (std::size_t I = 0; I < Count; ++I)
    {
      ++Impl_->Stats.Queries;
      Results[I] = TraceSceneRay(*Impl_, Snapshot.Data_.get(), Requests[I]);
    }
    return Error::None;
  }
  catch (const std::bad_alloc &)
  {
    Impl_->Lease.Reset();
    return Error::OutOfMemory;
  }
  catch (...)
  {
    Impl_->Lease.Reset();
    return Error::CorruptData;
  }
}
QueryStats QueryContext::GetStats() const noexcept
{
  return Impl_->Stats;
}
} // namespace physx_pack
