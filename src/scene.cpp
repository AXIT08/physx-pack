#include "detail/runtime.h"
#include <atomic>
#include <mutex>
#include <new>
#include <numeric>
#include <unordered_set>

namespace physx_pack
{
namespace
{
namespace rt = detail;
namespace vis = visibility;

bool NormalizeInstance(InstanceDesc &D) noexcept
{
  const auto &P = D.WorldPose;
  if (!D.Id || D.Resource.Origin.empty() || D.Resource.BundleId.empty() ||
      D.Resource.SerializedFile.empty() || !rt::Finite(P.Position) || !rt::Finite(D.Scale) ||
      D.Scale.X == 0 || D.Scale.Y == 0 || D.Scale.Z == 0 || !std::isfinite(P.RotationX) ||
      !std::isfinite(P.RotationY) || !std::isfinite(P.RotationZ) || !std::isfinite(P.RotationW) ||
      (D.MeshType != MeshVariant::Automatic && D.MeshType != MeshVariant::Triangle &&
       D.MeshType != MeshVariant::Convex))
    return false;
  const double Length =
      std::sqrt(double(P.RotationX) * P.RotationX + double(P.RotationY) * P.RotationY +
                double(P.RotationZ) * P.RotationZ + double(P.RotationW) * P.RotationW);
  if (!(Length > 0) || !std::isfinite(Length))
    return false;
  D.WorldPose.RotationX = static_cast<float>(P.RotationX / Length);
  D.WorldPose.RotationY = static_cast<float>(P.RotationY / Length);
  D.WorldPose.RotationZ = static_cast<float>(P.RotationZ / Length);
  D.WorldPose.RotationW = static_cast<float>(P.RotationW / Length);
  return true;
}

void WorldBounds(rt::PreparedInstance &I) noexcept
{
  I.WorldBoundsDefined = false;
  if (!I.LocalBoundsDefined)
    return;
  const auto &P = I.Desc.WorldPose;
  const double x = P.RotationX, y = P.RotationY, z = P.RotationZ, w = P.RotationW;
  const double Matrix[3][3]{{1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)},
                            {2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
                            {2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)}};
  vis::Aabb Result{};
  double RoundoffMagnitude =
      std::max({1.0, std::abs(double(P.Position.X)), std::abs(double(P.Position.Y)),
                std::abs(double(P.Position.Z))});
  for (unsigned Corner = 0; Corner < 8; ++Corner)
  {
    const double V[3]{
        (Corner & 1 ? I.LocalBounds.Maximum.X : I.LocalBounds.Minimum.X) * double(I.Desc.Scale.X),
        (Corner & 2 ? I.LocalBounds.Maximum.Y : I.LocalBounds.Minimum.Y) * double(I.Desc.Scale.Y),
        (Corner & 4 ? I.LocalBounds.Maximum.Z : I.LocalBounds.Minimum.Z) * double(I.Desc.Scale.Z)};
    for (unsigned Axis = 0; Axis < 3; ++Axis)
      RoundoffMagnitude = std::max(RoundoffMagnitude, std::abs(Matrix[Axis][0] * V[0]) +
                                                          std::abs(Matrix[Axis][1] * V[1]) +
                                                          std::abs(Matrix[Axis][2] * V[2]));
    vis::Vec3 Point{static_cast<float>(P.Position.X + Matrix[0][0] * V[0] + Matrix[0][1] * V[1] +
                                       Matrix[0][2] * V[2]),
                    static_cast<float>(P.Position.Y + Matrix[1][0] * V[0] + Matrix[1][1] * V[1] +
                                       Matrix[1][2] * V[2]),
                    static_cast<float>(P.Position.Z + Matrix[2][0] * V[0] + Matrix[2][1] * V[1] +
                                       Matrix[2][2] * V[2])};
    if (!vis::IsFinite(Point))
      return;
    if (!Corner)
      Result = {Point, Point};
    else
      Result = rt::MergeBounds(Result, {Point, Point});
  }
  // Bound float roundoff in the forward/inverse pose transforms as well as
  // the conversion from double corner coordinates to query-space floats.
  const double Magnitude = std::max(
      {RoundoffMagnitude, std::abs(double(Result.Minimum.X)), std::abs(double(Result.Minimum.Y)),
       std::abs(double(Result.Minimum.Z)), std::abs(double(Result.Maximum.X)),
       std::abs(double(Result.Maximum.Y)), std::abs(double(Result.Maximum.Z))});
  const double Padding = 16 * std::numeric_limits<float>::epsilon() * Magnitude;
  Result.Minimum = {static_cast<float>(Result.Minimum.X - Padding),
                    static_cast<float>(Result.Minimum.Y - Padding),
                    static_cast<float>(Result.Minimum.Z - Padding)};
  Result.Maximum = {static_cast<float>(Result.Maximum.X + Padding),
                    static_cast<float>(Result.Maximum.Y + Padding),
                    static_cast<float>(Result.Maximum.Z + Padding)};
  const float Inf = std::numeric_limits<float>::infinity();
  Result.Minimum = {std::nextafter(Result.Minimum.X, -Inf), std::nextafter(Result.Minimum.Y, -Inf),
                    std::nextafter(Result.Minimum.Z, -Inf)};
  Result.Maximum = {std::nextafter(Result.Maximum.X, Inf), std::nextafter(Result.Maximum.Y, Inf),
                    std::nextafter(Result.Maximum.Z, Inf)};
  if (rt::ValidBounds(Result))
  {
    I.WorldBounds = Result;
    I.WorldBoundsDefined = true;
  }
}

void Resolve(rt::PackageData &P, rt::PreparedInstance &I)
{
  I.LocalBoundsDefined = I.WorldBoundsDefined = false;
  const auto &Key = I.Desc.Resource;
  I.ResourceError =
      P.Reader.ResolveSource({Key.Origin, Key.BundleId, Key.SerializedFile, Key.PathId}, &I.Slot);
  if (I.ResourceError != Error::None)
    return;
  rt::GeometryKind Kind{};
  if (!P.Reader.GetGeometryKind(I.Slot, &Kind))
  {
    I.ResourceError = Error::CorruptData;
    return;
  }
  if (Kind == rt::GeometryKind::HeightField)
  {
    if (I.Desc.MeshType != MeshVariant::Automatic)
    {
      I.ResourceError = Error::UnsupportedGeometry;
      return;
    }
    I.Type = vis::GeometryType::HeightField;
  }
  else if (Kind == rt::GeometryKind::TriangleMesh || Kind == rt::GeometryKind::ConvexMesh)
  {
    I.Type = (I.Desc.MeshType == MeshVariant::Convex ||
              (I.Desc.MeshType == MeshVariant::Automatic && Kind == rt::GeometryKind::ConvexMesh))
                 ? vis::GeometryType::ConvexMesh
                 : vis::GeometryType::TriangleMesh;
  }
  else
  {
    I.ResourceError = Error::UnsupportedGeometry;
    return;
  }
  rt::GeometryDescription Description{};
  I.ResourceError = P.Geometry.Describe(I.Slot, I.Type, &Description);
  if (I.ResourceError == Error::None)
  {
    I.LocalBounds = Description.LocalBounds;
    I.LocalBoundsDefined = Description.BoundsDefined;
    WorldBounds(I);
  }
}

std::uint32_t BuildNode(rt::SceneData &D, std::uint32_t Begin, std::uint32_t End)
{
  const auto Index = static_cast<std::uint32_t>(D.Nodes.size());
  D.Nodes.push_back({});
  vis::Aabb Bounds = D.Instances[D.Order[Begin]].WorldBounds;
  for (auto i = Begin + 1; i < End; ++i)
    Bounds = rt::MergeBounds(Bounds, D.Instances[D.Order[i]].WorldBounds);
  D.Nodes[Index].Bounds = Bounds;
  if (End - Begin <= 4)
  {
    D.Nodes[Index].Begin = Begin;
    D.Nodes[Index].Count = End - Begin;
    return Index;
  }
  const double Extent[3]{double(Bounds.Maximum.X) - Bounds.Minimum.X,
                         double(Bounds.Maximum.Y) - Bounds.Minimum.Y,
                         double(Bounds.Maximum.Z) - Bounds.Minimum.Z};
  const auto Axis = static_cast<unsigned>(std::max_element(Extent, Extent + 3) - Extent);
  const auto Middle = Begin + (End - Begin) / 2;
  auto Center = [Axis](const rt::PreparedInstance &I)
  {
    const auto &B = I.WorldBounds;
    return Axis == 0   ? double(B.Minimum.X) + B.Maximum.X
           : Axis == 1 ? double(B.Minimum.Y) + B.Maximum.Y
                       : double(B.Minimum.Z) + B.Maximum.Z;
  };
  std::nth_element(D.Order.begin() + Begin, D.Order.begin() + Middle, D.Order.begin() + End,
                   [&](auto A, auto B)
                   {
                     const auto CA = Center(D.Instances[A]), CB = Center(D.Instances[B]);
                     return CA != CB ? CA < CB : D.Instances[A].Desc.Id < D.Instances[B].Desc.Id;
                   });
  D.Nodes[Index].Left = BuildNode(D, Begin, Middle);
  D.Nodes[Index].Right = BuildNode(D, Middle, End);
  return Index;
}
void Build(rt::SceneData &D)
{
  D.Order.clear();
  D.Unbounded.clear();
  D.Nodes.clear();
  for (std::uint32_t i = 0; i < D.Instances.size(); ++i)
    (D.Instances[i].WorldBoundsDefined ? D.Order : D.Unbounded).push_back(i);
  D.Nodes.reserve(D.Order.size() * 2);
  if (!D.Order.empty())
    BuildNode(D, 0, static_cast<std::uint32_t>(D.Order.size()));
  ++D.Stats.BvhBuilds;
}
void Refit(rt::SceneData &D) noexcept
{
  for (auto i = D.Nodes.size(); i > 0; --i)
  {
    auto &N = D.Nodes[i - 1];
    if (N.Count)
    {
      N.Bounds = D.Instances[D.Order[N.Begin]].WorldBounds;
      for (std::uint32_t j = 1; j < N.Count; ++j)
        N.Bounds = rt::MergeBounds(N.Bounds, D.Instances[D.Order[N.Begin + j]].WorldBounds);
    }
    else
      N.Bounds = rt::MergeBounds(D.Nodes[N.Left].Bounds, D.Nodes[N.Right].Bounds);
  }
  ++D.Stats.BvhRefits;
}
} // namespace

struct Scene::Impl
{
  std::shared_ptr<const detail::SceneData> Published;
  std::mutex Writer;
};
Scene::Scene(std::shared_ptr<Package> Package) : Impl_(std::make_unique<Impl>())
{
  if (Package)
  {
    auto Data = std::make_shared<detail::SceneData>();
    Data->Package = Package->Data_;
    Impl_->Published = std::move(Data);
  }
}
Scene::~Scene() noexcept = default;
SceneSnapshot Scene::GetSnapshot() const noexcept
{
  SceneSnapshot Result;
  Result.Data_ = std::atomic_load(&Impl_->Published);
  return Result;
}
SceneStats Scene::GetStats() const noexcept
{
  const auto D = std::atomic_load(&Impl_->Published);
  return D ? D->Stats : SceneStats{};
}
std::uint64_t SceneSnapshot::Generation() const noexcept
{
  return Data_ ? Data_->Stats.Generation : 0;
}
std::size_t SceneSnapshot::InstanceCount() const noexcept
{
  return Data_ ? Data_->Instances.size() : 0;
}

Error Scene::ApplyUpdates(const InstanceDesc *Upserts, std::size_t UpsertCount,
                          const InstanceId *Removals, std::size_t RemovalCount) noexcept
{
  try
  {
    if ((UpsertCount && !Upserts) || (RemovalCount && !Removals) || UpsertCount > UINT32_MAX ||
        RemovalCount > UINT32_MAX)
      return Error::InvalidArgument;
    std::lock_guard<std::mutex> Lock(Impl_->Writer);
    auto Previous = std::atomic_load(&Impl_->Published);
    if (!Previous)
      return Error::NotOpen;
    if (!UpsertCount && !RemovalCount)
      return Error::None;
    std::unordered_set<InstanceId> Seen;
    std::vector<InstanceDesc> Updates;
    Updates.reserve(UpsertCount);
    for (std::size_t i = 0; i < UpsertCount; ++i)
    {
      Updates.push_back(Upserts[i]);
      if (!NormalizeInstance(Updates.back()) || !Seen.insert(Upserts[i].Id).second)
        return Error::InvalidArgument;
    }
    for (std::size_t i = 0; i < RemovalCount; ++i)
      if (!Removals[i] || !Seen.insert(Removals[i]).second)
        return Error::InvalidArgument;
    auto Next = std::make_shared<detail::SceneData>(*Previous);
    bool Rebuild = false;
    for (std::size_t i = 0; i < RemovalCount; ++i)
    {
      auto It = std::lower_bound(Next->Instances.begin(), Next->Instances.end(), Removals[i],
                                 [](const auto &I, auto Id) { return I.Desc.Id < Id; });
      if (It == Next->Instances.end() || It->Desc.Id != Removals[i])
        return Error::TargetNotFound;
      Next->Instances.erase(It);
      Rebuild = true;
    }
    for (auto &Update : Updates)
    {
      auto It = std::lower_bound(Next->Instances.begin(), Next->Instances.end(), Update.Id,
                                 [](const auto &I, auto Id) { return I.Desc.Id < Id; });
      const bool Existing = It != Next->Instances.end() && It->Desc.Id == Update.Id;
      detail::PreparedInstance Prepared;
      if (Existing && It->Desc.Resource == Update.Resource &&
          It->Desc.MeshType == Update.MeshType && !detail::Transient(It->ResourceError))
      {
        Prepared = *It;
        Prepared.Desc = std::move(Update);
        WorldBounds(Prepared);
      }
      else
      {
        Prepared.Desc = std::move(Update);
        Resolve(*Next->Package, Prepared);
        ++Next->Stats.ResourceResolutions;
      }
      if (Existing)
      {
        Rebuild |= It->WorldBoundsDefined != Prepared.WorldBoundsDefined;
        *It = std::move(Prepared);
      }
      else
      {
        Next->Instances.insert(It, std::move(Prepared));
        Rebuild = true;
      }
    }
    if (Next->Instances.size() > UINT32_MAX)
      return Error::InvalidArgument;
    if (Rebuild)
      Build(*Next);
    else
      Refit(*Next);
    ++Next->Stats.Generation;
    Next->Stats.InstanceCount = Next->Instances.size();
    Next->Stats.UnboundedInstances = Next->Unbounded.size();
    std::shared_ptr<const detail::SceneData> Published = std::move(Next);
    std::atomic_store(&Impl_->Published, std::move(Published));
    return Error::None;
  }
  catch (const std::bad_alloc &)
  {
    return Error::OutOfMemory;
  }
  catch (...)
  {
    return Error::InvalidArgument;
  }
}
} // namespace physx_pack
