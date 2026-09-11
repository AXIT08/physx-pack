#include "detail/ray_query.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <new>
#include <stdexcept>
#include <utility>
#include <vector>

namespace physx_pack
{
namespace visibility
{
namespace
{

constexpr float KDirectionLengthTolerance = 1.0e-3f;
constexpr float KTriangleEpsilon = 1.0e-7f;
constexpr float KRaySurfaceOffset = 10.0f;

Vec3 Add(const Vec3 &Left, const Vec3 &Right) noexcept
{
  return {Left.X + Right.X, Left.Y + Right.Y, Left.Z + Right.Z};
}

Vec3 Subtract(const Vec3 &Left, const Vec3 &Right) noexcept
{
  return {Left.X - Right.X, Left.Y - Right.Y, Left.Z - Right.Z};
}

Vec3 Multiply(const Vec3 &Value, float Scale) noexcept
{
  return {Value.X * Scale, Value.Y * Scale, Value.Z * Scale};
}

float Dot(const Vec3 &Left, const Vec3 &Right) noexcept
{
  return Left.X * Right.X + Left.Y * Right.Y + Left.Z * Right.Z;
}

Vec3 Cross(const Vec3 &Left, const Vec3 &Right) noexcept
{
  return {
      Left.Y * Right.Z - Left.Z * Right.Y,
      Left.Z * Right.X - Left.X * Right.Z,
      Left.X * Right.Y - Left.Y * Right.X,
  };
}

float LengthSquared(const Vec3 &Value) noexcept
{
  return Dot(Value, Value);
}

bool Normalize(const Vec3 &Value, Vec3 *Output) noexcept
{
  const float MagnitudeSquared = LengthSquared(Value);
  if (!std::isfinite(MagnitudeSquared) || !(MagnitudeSquared > 0.0f))
  {
    return false;
  }
  const float InverseMagnitude = 1.0f / std::sqrt(MagnitudeSquared);
  *Output = Multiply(Value, InverseMagnitude);
  return IsFinite(*Output);
}

bool Normalize(const Quat &Value, Quat *Output) noexcept
{
  if (!std::isfinite(Value.X) || !std::isfinite(Value.Y) ||
      !std::isfinite(Value.Z) || !std::isfinite(Value.W))
  {
    return false;
  }
  const double MagnitudeSquared =
      static_cast<double>(Value.X) * Value.X +
      static_cast<double>(Value.Y) * Value.Y +
      static_cast<double>(Value.Z) * Value.Z +
      static_cast<double>(Value.W) * Value.W;
  if (!std::isfinite(MagnitudeSquared) || !(MagnitudeSquared > 0.0))
  {
    return false;
  }
  const double InverseMagnitude = 1.0 / std::sqrt(MagnitudeSquared);
  *Output = {
      static_cast<float>(Value.X * InverseMagnitude),
      static_cast<float>(Value.Y * InverseMagnitude),
      static_cast<float>(Value.Z * InverseMagnitude),
      static_cast<float>(Value.W * InverseMagnitude),
  };
  return true;
}

Vec3 Rotate(const Quat &Rotation, const Vec3 &Value) noexcept
{
  const Vec3 Axis{Rotation.X, Rotation.Y, Rotation.Z};
  const Vec3 First = Cross(Axis, Value);
  const Vec3 Second = Cross(Axis, First);
  return Add(Value,
             Add(Multiply(First, 2.0f * Rotation.W),
                 Multiply(Second, 2.0f)));
}

Vec3 RotateInverse(const Quat &Rotation, const Vec3 &Value) noexcept
{
  return Rotate({-Rotation.X, -Rotation.Y, -Rotation.Z, Rotation.W},
                Value);
}

bool ResolvePoseRotation(const Pose &Value, Quat *Rotation) noexcept
{
  return IsFinite(Value.Position) && Normalize(Value.Rotation, Rotation);
}

bool IsValidAabb(const Aabb &Bounds) noexcept
{
  return IsFinite(Bounds.Minimum) && IsFinite(Bounds.Maximum) &&
         Bounds.Minimum.X <= Bounds.Maximum.X &&
         Bounds.Minimum.Y <= Bounds.Maximum.Y &&
         Bounds.Minimum.Z <= Bounds.Maximum.Z;
}

RayHit UndefinedHit() noexcept
{
  RayHit Result{};
  Result.Status = QueryStatus::Undefined;
  return Result;
}

bool ContinueQuery(QueryControl *Control)
{
  if (Control == nullptr)
  {
    return true;
  }
  if (Control->Interrupted)
  {
    return false;
  }
  if (Control->Continue != nullptr &&
      !Control->Continue(Control->Context))
  {
    Control->Interrupted = true;
    return false;
  }
  return true;
}

class MeshWorkspaceLease
{
public:
  explicit MeshWorkspaceLease(QueryControl *Control)
      : Scratch_(Control != nullptr ? Control->Scratch : nullptr)
  {
    if (Scratch_ != nullptr)
    {
      if (Scratch_->Depth == Scratch_->Workspaces.size())
      {
        Scratch_->Workspaces.push_back(std::make_unique<MeshQueryWorkspace>());
      }
      Workspace_ = Scratch_->Workspaces[Scratch_->Depth++].get();
    }
    Clear();
  }

  ~MeshWorkspaceLease()
  {
    Clear();
    if (Scratch_ != nullptr)
    {
      --Scratch_->Depth;
    }
  }

  MeshQueryWorkspace &Get() { return *Workspace_; }

private:
  void Clear()
  {
    Workspace_->RTreeStack.clear();
    Workspace_->Bv4Stack.clear();
    Workspace_->RangeIndices.clear();
  }

  QueryScratch *Scratch_ = nullptr;
  MeshQueryWorkspace Local_;
  MeshQueryWorkspace *Workspace_ = &Local_;
};

RayHit MakeHit(const Ray &QueryRay, float Distance, const Vec3 &Normal,
               std::uint32_t FaceIndex = UINT32_MAX) noexcept
{
  RayHit Result{};
  Result.Status = QueryStatus::Hit;
  Result.Distance = Distance;
  Result.Position = Add(QueryRay.Origin,
                        Multiply(QueryRay.Direction, Distance));
  Result.Normal = Normal;
  Result.FaceIndex = FaceIndex;
  return Result;
}

bool InDistanceRange(float Distance, float Maximum) noexcept
{
  return std::isfinite(Distance) && Distance >= 0.0f &&
         Distance <= Maximum;
}

bool IsIdentityScale(const MeshScale &Scale) noexcept
{
  return Scale.Scale.X == 1.0f && Scale.Scale.Y == 1.0f &&
         Scale.Scale.Z == 1.0f && Scale.Rotation.X == 0.0f &&
         Scale.Rotation.Y == 0.0f && Scale.Rotation.Z == 0.0f &&
         Scale.Rotation.W == 1.0f;
}

bool ApplyInverseScale(const MeshScale &Scale, const Vec3 &Value,
                       Vec3 *Output) noexcept
{
  Quat Rotation{};
  if (!IsFinite(Scale.Scale) || Scale.Scale.X == 0.0f ||
      Scale.Scale.Y == 0.0f || Scale.Scale.Z == 0.0f ||
      !Normalize(Scale.Rotation, &Rotation))
  {
    return false;
  }
  const Vec3 Rotated = RotateInverse(Rotation, Value);
  const Vec3 Scaled{
      Rotated.X / Scale.Scale.X,
      Rotated.Y / Scale.Scale.Y,
      Rotated.Z / Scale.Scale.Z,
  };
  *Output = Rotate(Rotation, Scaled);
  return IsFinite(*Output);
}

Vec3 TransformMeshNormal(const Pose &WorldPose, const MeshScale &Scale,
                         const Vec3 &Normal) noexcept
{
  Quat PoseRotation{};
  Quat ScaleRotation{};
  if (!Normalize(WorldPose.Rotation, &PoseRotation) ||
      !Normalize(Scale.Rotation, &ScaleRotation))
  {
    return {};
  }
  const Vec3 Rotated = RotateInverse(ScaleRotation, Normal);
  const Vec3 Scaled{
      Rotated.X / Scale.Scale.X,
      Rotated.Y / Scale.Scale.Y,
      Rotated.Z / Scale.Scale.Z,
  };
  Vec3 Result{};
  (void)Normalize(Rotate(PoseRotation,
                         Rotate(ScaleRotation, Scaled)),
                  &Result);
  return Result;
}

RayHit RaycastConvex(const Ray &QueryRay,
                     const GeometryView &Geometry,
                     QueryControl *Control)
{
  Quat PoseRotation{};
  if (!ResolvePoseRotation(Geometry.WorldPose, &PoseRotation) ||
      Geometry.Convex.Planes.Count == 0 ||
      Geometry.Convex.Planes.Data == nullptr)
  {
    return UndefinedHit();
  }

  Vec3 PoseOrigin = RotateInverse(
      PoseRotation,
      Subtract(QueryRay.Origin, Geometry.WorldPose.Position));
  Vec3 PoseDirection =
      RotateInverse(PoseRotation, QueryRay.Direction);
  Vec3 LocalOrigin{};
  Vec3 LocalDirection{};
  if (!ApplyInverseScale(Geometry.Convex.Scale, PoseOrigin, &LocalOrigin) ||
      !ApplyInverseScale(Geometry.Convex.Scale, PoseDirection,
                         &LocalDirection))
  {
    return UndefinedHit();
  }

  bool OriginInsideAllPlanes = true;
  float LatestEntry = -std::numeric_limits<float>::infinity();
  float EarliestExit = std::numeric_limits<float>::infinity();
  std::uint32_t FaceIndex = UINT32_MAX;
  for (std::size_t Index = 0; Index < Geometry.Convex.Planes.Count; ++Index)
  {
    if (!ContinueQuery(Control))
    {
      return UndefinedHit();
    }
    const ConvexPlane &Plane = Geometry.Convex.Planes.Data[Index];
    if (!IsFinite(Plane.Normal) || !std::isfinite(Plane.Distance))
    {
      return UndefinedHit();
    }
    const float DistanceToPlane =
        Dot(Plane.Normal, LocalOrigin) + Plane.Distance;
    const float DirectionProjection = Dot(Plane.Normal, LocalDirection);
    if (DistanceToPlane > 0.0f)
    {
      OriginInsideAllPlanes = false;
    }
    if (DirectionProjection > KVisibilityPlaneEpsilon)
    {
      EarliestExit = std::min(
          EarliestExit, -DistanceToPlane / DirectionProjection);
    }
    else if (DirectionProjection < -KVisibilityPlaneEpsilon)
    {
      const float Entry = -DistanceToPlane / DirectionProjection;
      if (Entry > LatestEntry)
      {
        LatestEntry = Entry;
        FaceIndex = static_cast<std::uint32_t>(Index);
      }
    }
    else if (DistanceToPlane > 0.0f)
    {
      return {};
    }
  }

  if (OriginInsideAllPlanes)
  {
    return MakeHit(QueryRay, 0.0f,
                   Multiply(QueryRay.Direction, -1.0f));
  }
  if (!(LatestEntry < EarliestExit && LatestEntry > 0.0f &&
        LatestEntry <
            QueryRay.MaxDistance - KVisibilityConvexEndEpsilon))
  {
    return {};
  }

  const Vec3 Normal = TransformMeshNormal(
      Geometry.WorldPose, Geometry.Convex.Scale,
      Geometry.Convex.Planes.Data[FaceIndex].Normal);
  if (!IsFinite(Normal) || !(LengthSquared(Normal) > 0.0f))
  {
    return UndefinedHit();
  }
  return MakeHit(QueryRay, LatestEntry, Normal, FaceIndex);
}

struct MeshRay
{
  Ray Local{};
  float DirectionLength = 1.0f;
  bool NegativeDeterminant = false;
};

bool BuildMeshRay(const Ray &QueryRay, const Pose &WorldPose,
                  const MeshScale &Scale, MeshRay *Output) noexcept
{
  Quat PoseRotation{};
  if (!ResolvePoseRotation(WorldPose, &PoseRotation))
  {
    return false;
  }
  const Vec3 PoseOrigin = RotateInverse(
      PoseRotation, Subtract(QueryRay.Origin, WorldPose.Position));
  const Vec3 PoseDirection =
      RotateInverse(PoseRotation, QueryRay.Direction);
  Vec3 VertexOrigin{};
  Vec3 VertexDirection{};
  if (!ApplyInverseScale(Scale, PoseOrigin, &VertexOrigin) ||
      !ApplyInverseScale(Scale, PoseDirection, &VertexDirection))
  {
    return false;
  }
  const float DirectionLength = std::sqrt(LengthSquared(VertexDirection));
  if (!std::isfinite(DirectionLength) || !(DirectionLength > 0.0f))
  {
    return false;
  }
  Output->Local.Origin = VertexOrigin;
  Output->Local.Direction = Multiply(VertexDirection, 1.0f / DirectionLength);
  Output->Local.MaxDistance =
      DirectionLength * QueryRay.MaxDistance +
      (IsIdentityScale(Scale) ? 0.0f : 0.001f);
  Output->DirectionLength = DirectionLength;
  Output->NegativeDeterminant =
      Scale.Scale.X * Scale.Scale.Y * Scale.Scale.Z < 0.0f;
  return IsDefinedRay(Output->Local);
}

} // namespace

bool IsFinite(const Vec3 &Value) noexcept
{
  return std::isfinite(Value.X) && std::isfinite(Value.Y) &&
         std::isfinite(Value.Z);
}

bool IsDefinedRay(const Ray &Value) noexcept
{
  if (!IsFinite(Value.Origin) || !IsFinite(Value.Direction) ||
      !std::isfinite(Value.MaxDistance) || Value.MaxDistance < 0.0f)
  {
    return false;
  }
  const float DirectionSquared = LengthSquared(Value.Direction);
  return std::isfinite(DirectionSquared) &&
         std::fabs(DirectionSquared - 1.0f) <=
             KDirectionLengthTolerance;
}

PreparedAabbRay PrepareAabbRay(const Ray &QueryRay) noexcept
{
  PreparedAabbRay Result{};
  Result.Source = QueryRay;
  Result.Defined = IsDefinedRay(QueryRay);
  Result.Origin = {QueryRay.Origin.X, QueryRay.Origin.Y, QueryRay.Origin.Z};
  const std::array<float, 3> Direction = {
      QueryRay.Direction.X, QueryRay.Direction.Y, QueryRay.Direction.Z};
  for (std::size_t Axis = 0; Axis < 3; ++Axis)
  {
    Result.Parallel[Axis] = std::fabs(Direction[Axis]) < 1.0e-9f;
    if (!Result.Parallel[Axis])
    {
      Result.InverseDirection[Axis] = 1.0f / Direction[Axis];
    }
  }
  return Result;
}

RayHit RaycastAabb(const Ray &QueryRay, const Aabb &Bounds) noexcept
{
  return RaycastAabb(PrepareAabbRay(QueryRay), QueryRay.MaxDistance, Bounds);
}

RayHit RaycastAabb(const PreparedAabbRay &Prepared, float MaxDistance,
                   const Aabb &Bounds) noexcept
{
  if (!Prepared.Defined || !std::isfinite(MaxDistance) || MaxDistance < 0.0f ||
      !IsValidAabb(Bounds))
  {
    return UndefinedHit();
  }

  float Near = 0.0f;
  Ray QueryRay = Prepared.Source;
  QueryRay.MaxDistance = MaxDistance;
  float Far = MaxDistance;
  int NearAxis = -1;
  float NearSign = 0.0f;
  const auto &Origin = Prepared.Origin;
  const std::array<float, 3> Minimum = {
      Bounds.Minimum.X, Bounds.Minimum.Y, Bounds.Minimum.Z};
  const std::array<float, 3> Maximum = {
      Bounds.Maximum.X, Bounds.Maximum.Y, Bounds.Maximum.Z};
  for (std::size_t Axis = 0; Axis < 3; ++Axis)
  {
    if (Prepared.Parallel[Axis])
    {
      if (Origin[Axis] < Minimum[Axis] || Origin[Axis] > Maximum[Axis])
      {
        return {};
      }
      continue;
    }
    const float InverseDirection = Prepared.InverseDirection[Axis];
    float First = (Minimum[Axis] - Origin[Axis]) * InverseDirection;
    float Second = (Maximum[Axis] - Origin[Axis]) * InverseDirection;
    float Sign = -1.0f;
    if (First > Second)
    {
      std::swap(First, Second);
      Sign = 1.0f;
    }
    if (First > Near)
    {
      Near = First;
      NearAxis = static_cast<int>(Axis);
      NearSign = Sign;
    }
    Far = std::min(Far, Second);
    if (Near > Far)
    {
      return {};
    }
  }

  if (!InDistanceRange(Near, QueryRay.MaxDistance))
  {
    return {};
  }
  Vec3 Normal = Multiply(QueryRay.Direction, -1.0f);
  if (NearAxis == 0)
  {
    Normal = {NearSign, 0.0f, 0.0f};
  }
  else if (NearAxis == 1)
  {
    Normal = {0.0f, NearSign, 0.0f};
  }
  else if (NearAxis == 2)
  {
    Normal = {0.0f, 0.0f, NearSign};
  }
  return MakeHit(QueryRay, Near, Normal);
}

namespace
{

struct TriangleHit
{
  QueryStatus Status = QueryStatus::Miss;
  float LocalDistance = 0.0f;
  Vec3 LocalNormal{};
  std::uint32_t FaceIndex = UINT32_MAX;
};

bool ReadTriangleIndices(const TriangleData &Triangles,
                         std::uint32_t TriangleIndex,
                         std::array<std::uint32_t, 3> *Output) noexcept
{
  if (Output == nullptr || TriangleIndex >= Triangles.TriangleCount)
  {
    return false;
  }
  if (Triangles.LoadIndices != nullptr)
  {
    return Triangles.LoadIndices(
        Triangles.ProviderContext, TriangleIndex, Output);
  }
  if (Triangles.Indices == nullptr)
  {
    return false;
  }
  const std::size_t Base = static_cast<std::size_t>(TriangleIndex) * 3U;
  if (Triangles.IndexWidth == TriangleIndexWidth::Index32)
  {
    const auto *Indices =
        static_cast<const std::uint32_t *>(Triangles.Indices);
    *Output = {Indices[Base], Indices[Base + 1], Indices[Base + 2]};
  }
  else
  {
    const auto *Indices =
        static_cast<const std::uint16_t *>(Triangles.Indices);
    *Output = {Indices[Base], Indices[Base + 1], Indices[Base + 2]};
  }
  return Triangles.LoadIndices != nullptr ||
         ((*Output)[0] < Triangles.Vertices.Count &&
          (*Output)[1] < Triangles.Vertices.Count &&
          (*Output)[2] < Triangles.Vertices.Count);
}

TriangleHit RaycastTriangle(const Ray &LocalRay,
                            const TriangleData &Triangles,
                            std::uint32_t TriangleIndex,
                            bool CullBackfaces,
                            bool NegativeDeterminant,
                            const std::array<std::uint32_t, 3> *IndicesOverride = nullptr) noexcept
{
  if (Triangles.Vertices.Data == nullptr &&
      Triangles.LoadVertex == nullptr)
  {
    TriangleHit Result{};
    Result.Status = QueryStatus::Undefined;
    return Result;
  }
  std::array<std::uint32_t, 3> Indices{};
  if (IndicesOverride != nullptr)
  {
    Indices = *IndicesOverride;
  }
  else if (!ReadTriangleIndices(Triangles, TriangleIndex, &Indices))
  {
    TriangleHit Result{};
    Result.Status = QueryStatus::Undefined;
    return Result;
  }
  if (NegativeDeterminant)
  {
    std::swap(Indices[1], Indices[2]);
  }

  std::array<Vec3, 3> Vertices{};
  for (std::size_t Slot = 0; Slot < Vertices.size(); ++Slot)
  {
    const std::uint32_t VertexIndex = Indices[Slot];
    if (Triangles.LoadVertex != nullptr)
    {
      if (!Triangles.LoadVertex(
              Triangles.ProviderContext, VertexIndex, &Vertices[Slot]))
      {
        TriangleHit Result{};
        Result.Status = QueryStatus::Undefined;
        return Result;
      }
    }
    else
    {
      Vertices[Slot] = Triangles.Vertices.Data[VertexIndex];
    }
    if (!IsFinite(Vertices[Slot]))
    {
      TriangleHit Result{};
      Result.Status = QueryStatus::Undefined;
      return Result;
    }
  }
  const Vec3 &First = Vertices[0];
  const Vec3 &Second = Vertices[1];
  const Vec3 &Third = Vertices[2];
  if (!IsFinite(First) || !IsFinite(Second) || !IsFinite(Third))
  {
    TriangleHit Result{};
    Result.Status = QueryStatus::Undefined;
    return Result;
  }

  const Vec3 EdgeOne = Subtract(Second, First);
  const Vec3 EdgeTwo = Subtract(Third, First);
  const Vec3 DirectionCross = Cross(LocalRay.Direction, EdgeTwo);
  const float Determinant = Dot(EdgeOne, DirectionCross);
  const float Epsilon = Triangles.Epsilon;
  if (!std::isfinite(Epsilon) || Epsilon < 0.0f)
  {
    TriangleHit Result{};
    Result.Status = QueryStatus::Undefined;
    return Result;
  }
  if ((CullBackfaces && Determinant <= Epsilon) ||
      (!CullBackfaces && std::fabs(Determinant) <= Epsilon))
  {
    return {};
  }
  const float InverseDeterminant = 1.0f / Determinant;
  const Vec3 OriginDelta = Subtract(LocalRay.Origin, First);
  const float U = Dot(OriginDelta, DirectionCross) * InverseDeterminant;
  if (U < -Epsilon || U > 1.0f + Epsilon)
  {
    return {};
  }
  const Vec3 OriginCross = Cross(OriginDelta, EdgeOne);
  const float V = Dot(LocalRay.Direction, OriginCross) * InverseDeterminant;
  if (V < -Epsilon || U + V > 1.0f + Epsilon)
  {
    return {};
  }
  const float Distance = Dot(EdgeTwo, OriginCross) * InverseDeterminant;
  if (!InDistanceRange(Distance, LocalRay.MaxDistance))
  {
    return {};
  }

  Vec3 Normal{};
  if (!Normalize(Cross(EdgeOne, EdgeTwo), &Normal))
  {
    TriangleHit Result{};
    Result.Status = QueryStatus::Undefined;
    return Result;
  }
  if (!CullBackfaces && Dot(Normal, LocalRay.Direction) > 0.0f)
  {
    Normal = Multiply(Normal, -1.0f);
  }

  TriangleHit Result{};
  Result.Status = QueryStatus::Hit;
  Result.LocalDistance = Distance;
  Result.LocalNormal = Normal;
  Result.FaceIndex = TriangleIndex;
  return Result;
}

void MergeMeshHit(const TriangleHit &Candidate,
                  TriangleHit *Closest) noexcept
{
  if (Candidate.Status == QueryStatus::Undefined)
  {
    Closest->Status = QueryStatus::Undefined;
    return;
  }
  if (Candidate.Status != QueryStatus::Hit)
  {
    return;
  }
  if (Closest->Status == QueryStatus::Miss ||
      (Closest->Status == QueryStatus::Hit &&
       Candidate.LocalDistance < Closest->LocalDistance))
  {
    *Closest = Candidate;
  }
}

TriangleHit RaycastTriangleRange(const Ray &LocalRay,
                                 const TriangleData &Triangles,
                                 std::uint32_t First,
                                 std::uint32_t Count,
                                 bool CullBackfaces,
                                 bool NegativeDeterminant,
                                 const TriangleHit &Initial,
                                 QueryControl *Control,
                                 MeshQueryWorkspace &Workspace)
{
  TriangleHit Closest = Initial;
  if (First > Triangles.TriangleCount ||
      Count > Triangles.TriangleCount - First)
  {
    Closest.Status = QueryStatus::Undefined;
    return Closest;
  }
  auto &RangeIndices = Workspace.RangeIndices;
  RangeIndices.clear();
  if (Triangles.LoadIndexRange != nullptr && Count != 0U)
  {
    std::size_t ValueCount = 0;
    ValueCount = static_cast<std::size_t>(Count) * 3U;
    try
    {
      RangeIndices.resize(ValueCount);
    }
    catch (const std::length_error &)
    {
      Closest.Status = QueryStatus::Undefined;
      return Closest;
    }
    catch (const std::bad_alloc &)
    {
      Closest.Status = QueryStatus::Undefined;
      return Closest;
    }
    if (!Triangles.LoadIndexRange(
            Triangles.ProviderContext, First, Count,
            RangeIndices.empty() ? nullptr : RangeIndices.data()))
    {
      Closest.Status = QueryStatus::Undefined;
      return Closest;
    }
  }
  for (std::uint32_t Offset = 0; Offset < Count; ++Offset)
  {
    if (!ContinueQuery(Control))
    {
      Closest.Status = QueryStatus::Undefined;
      return Closest;
    }
    Ray ClippedRay = LocalRay;
    if (Closest.Status == QueryStatus::Hit)
    {
      ClippedRay.MaxDistance = Closest.LocalDistance;
    }
    std::array<std::uint32_t, 3> Indices{};
    const std::array<std::uint32_t, 3> *IndicesOverride = nullptr;
    if (Triangles.LoadIndexRange != nullptr && Count != 0U)
    {
      const std::size_t Base = static_cast<std::size_t>(Offset) * 3U;
      Indices = {RangeIndices[Base], RangeIndices[Base + 1U],
                 RangeIndices[Base + 2U]};
      IndicesOverride = &Indices;
    }
    const TriangleHit Candidate = RaycastTriangle(
        ClippedRay, Triangles, First + Offset, CullBackfaces,
        NegativeDeterminant, IndicesOverride);
    MergeMeshHit(Candidate, &Closest);
    if (Closest.Status == QueryStatus::Undefined)
    {
      return Closest;
    }
  }
  return Closest;
}

using RTreeStackEntry = QueryRTreeEntry;

TriangleHit RaycastRTree(const Ray &LocalRay,
                         const TriangleMeshGeometry &Mesh,
                         bool CullBackfaces,
                         bool NegativeDeterminant,
                         QueryControl *Control,
                         MeshQueryWorkspace &Workspace)
{
  const bool LazyPages = Mesh.RTree.LoadPage != nullptr;
  if (Mesh.RTree.RootPageCount > Mesh.RTree.Pages.Count ||
      (Mesh.RTree.Pages.Count != 0 && Mesh.RTree.Pages.Data == nullptr &&
       !LazyPages))
  {
    TriangleHit Result{};
    Result.Status = QueryStatus::Undefined;
    return Result;
  }

  auto &Stack = Workspace.RTreeStack;
  Stack.reserve(Mesh.RTree.RootPageCount);
  for (std::uint32_t Root = Mesh.RTree.RootPageCount; Root != 0; --Root)
  {
    if (!ContinueQuery(Control))
    {
      TriangleHit Result{};
      Result.Status = QueryStatus::Undefined;
      return Result;
    }
    Stack.push_back({true, Root - 1U, {}, false});
  }

  TriangleHit Closest{};
  std::size_t Visits = 0;
  const std::size_t VisitBudget =
      Mesh.RTree.Pages.Count >
              (std::numeric_limits<std::size_t>::max() -
               Mesh.RTree.RootPageCount) /
                  4U
          ? std::numeric_limits<std::size_t>::max()
          : Mesh.RTree.Pages.Count * 4U + Mesh.RTree.RootPageCount;
  while (!Stack.empty())
  {
    if (!ContinueQuery(Control))
    {
      Closest.Status = QueryStatus::Undefined;
      return Closest;
    }
    RTreeStackEntry Entry = Stack.back();
    Stack.pop_back();
    ++Visits;
    if (Visits > VisitBudget)
    {
      Closest.Status = QueryStatus::Undefined;
      return Closest;
    }

    Ray ClippedRay = LocalRay;
    if (Closest.Status == QueryStatus::Hit)
    {
      ClippedRay.MaxDistance = Closest.LocalDistance;
    }
    if (Entry.HasBounds &&
        RaycastAabb(ClippedRay, Entry.Bounds).Status != QueryStatus::Hit)
    {
      continue;
    }

    if (!Entry.IsPage)
    {
      const std::uint32_t Data = Entry.Value;
      if ((Data & 1U) != 0U)
      {
        Closest = RaycastTriangleRange(
            LocalRay, Mesh.Triangles, DecodeRTreeLeafFirst(Data),
            DecodeRTreeLeafCount(Data), CullBackfaces,
            NegativeDeterminant, Closest, Control, Workspace);
        if (Closest.Status == QueryStatus::Undefined)
        {
          return Closest;
        }
        continue;
      }
      if (Data % KVisibilityRTreePageStride != 0U)
      {
        Closest.Status = QueryStatus::Undefined;
        return Closest;
      }
      Entry.Value = Data / KVisibilityRTreePageStride;
      Entry.IsPage = true;
    }

    if (Entry.Value >= Mesh.RTree.Pages.Count)
    {
      Closest.Status = QueryStatus::Undefined;
      return Closest;
    }
    RTreePage LazyPage{};
    const RTreePage *Page = nullptr;
    if (LazyPages)
    {
      if (!Mesh.RTree.LoadPage(
              Mesh.RTree.ProviderContext, Entry.Value, &LazyPage))
      {
        Closest.Status = QueryStatus::Undefined;
        return Closest;
      }
      Page = &LazyPage;
    }
    else
    {
      Page = &Mesh.RTree.Pages.Data[Entry.Value];
    }
    for (std::size_t Slot = 0; Slot < 4; ++Slot)
    {
      if ((Page->ValidMask & (1U << Slot)) == 0U)
      {
        continue;
      }
      const RayHit BoundsHit = RaycastAabb(ClippedRay, Page->Bounds[Slot]);
      if (BoundsHit.Status == QueryStatus::Undefined)
      {
        Closest.Status = QueryStatus::Undefined;
        return Closest;
      }
      if (BoundsHit.Status == QueryStatus::Hit)
      {
        Stack.push_back(
            {false, Page->Data[Slot], Page->Bounds[Slot], true});
      }
    }
  }
  return Closest;
}

Aabb DecodeBv4Bounds(const Bv4View &Tree,
                     const Bv4QuantizedBlock &Block,
                     std::size_t Slot) noexcept
{
  const Bv4QuantizedLane &XLane = Block.X.Axis[Slot];
  const Bv4QuantizedLane &YLane = Block.Y.Axis[Slot];
  const Bv4QuantizedLane &ZLane = Block.Z.Axis[Slot];
  // PhysX 的 Q 解码直接把两个 signed lane 分别乘以 min/max 系数。
  const Vec3 Minimum{
      static_cast<float>(XLane.Minimum) * Tree.CenterCoefficient.X,
      static_cast<float>(YLane.Minimum) * Tree.CenterCoefficient.Y,
      static_cast<float>(ZLane.Minimum) * Tree.CenterCoefficient.Z,
  };
  const Vec3 Maximum{
      static_cast<float>(XLane.Maximum) * Tree.ExtentCoefficient.X,
      static_cast<float>(YLane.Maximum) * Tree.ExtentCoefficient.Y,
      static_cast<float>(ZLane.Maximum) * Tree.ExtentCoefficient.Z,
  };
  return {Minimum, Maximum};
}

TriangleHit RaycastBv4(const Ray &LocalRay,
                       const TriangleMeshGeometry &Mesh,
                       bool CullBackfaces,
                       bool NegativeDeterminant,
                        QueryControl *Control,
                        MeshQueryWorkspace &Workspace)
{
  if (Mesh.Bv4.Blocks.Count == 0)
  {
    if (Mesh.Triangles.TriangleCount >= 16U)
    {
      TriangleHit Result{};
      Result.Status = QueryStatus::Undefined;
      return Result;
    }
    return RaycastTriangleRange(
        LocalRay, Mesh.Triangles, 0,
        static_cast<std::uint32_t>(Mesh.Triangles.TriangleCount),
        CullBackfaces, NegativeDeterminant, {}, Control, Workspace);
  }
  const bool LazyBlocks = Mesh.Bv4.LoadBlock != nullptr;
  if (Mesh.Bv4.Blocks.Data == nullptr && !LazyBlocks)
  {
    TriangleHit Result{};
    Result.Status = QueryStatus::Undefined;
    return Result;
  }

  auto &Stack = Workspace.Bv4Stack;
  Stack.reserve(1U);
  Stack.push_back(Mesh.Bv4.InitData);
  TriangleHit Closest{};
  std::size_t Visits = 0;
  const std::size_t VisitBudget =
      Mesh.Bv4.Blocks.Count >
              (std::numeric_limits<std::size_t>::max() - 1U) / 4U
          ? std::numeric_limits<std::size_t>::max()
          : Mesh.Bv4.Blocks.Count * 4U + 1U;
  while (!Stack.empty())
  {
    if (!ContinueQuery(Control))
    {
      Closest.Status = QueryStatus::Undefined;
      return Closest;
    }
    const std::uint32_t ParentData = Stack.back();
    Stack.pop_back();
    ++Visits;
    if (Visits > VisitBudget)
    {
      Closest.Status = QueryStatus::Undefined;
      return Closest;
    }

    const std::uint32_t ChildOffset = ParentData >> 11U;
    const std::uint32_t ChildType = (ParentData >> 1U) & 3U;
    const std::uint32_t ChildCount = 2U + ChildType;
    if (ChildCount > 4U || ChildOffset % 4U != 0U)
    {
      Closest.Status = QueryStatus::Undefined;
      return Closest;
    }
    const std::size_t BlockIndex = ChildOffset / 4U;
    if (BlockIndex >= Mesh.Bv4.Blocks.Count)
    {
      Closest.Status = QueryStatus::Undefined;
      return Closest;
    }
    Bv4QuantizedBlock LazyBlock{};
    const Bv4QuantizedBlock *Block = nullptr;
    if (LazyBlocks)
    {
      if (!Mesh.Bv4.LoadBlock(
              Mesh.Bv4.ProviderContext,
              static_cast<std::uint32_t>(BlockIndex), &LazyBlock))
      {
        Closest.Status = QueryStatus::Undefined;
        return Closest;
      }
      Block = &LazyBlock;
    }
    else
    {
      Block = &Mesh.Bv4.Blocks.Data[BlockIndex];
    }

    std::array<bool, 4> Intersects{};
    Ray ClippedRay = LocalRay;
    if (Closest.Status == QueryStatus::Hit)
    {
      ClippedRay.MaxDistance = Closest.LocalDistance;
    }
    for (std::uint32_t Slot = 0; Slot < ChildCount; ++Slot)
    {
      const RayHit BoundsHit =
          RaycastAabb(ClippedRay, DecodeBv4Bounds(Mesh.Bv4, *Block, Slot));
      if (BoundsHit.Status == QueryStatus::Undefined)
      {
        Closest.Status = QueryStatus::Undefined;
        return Closest;
      }
      Intersects[Slot] = BoundsHit.Status == QueryStatus::Hit;
    }

    for (std::size_t Reverse = ChildCount; Reverse != 0; --Reverse)
    {
      const std::size_t Slot = Reverse - 1U;
      const std::uint32_t Data = Block->Data[Slot];
      if (!Intersects[Slot] || (Data & 1U) == 0U)
      {
        continue;
      }
      const std::uint32_t TriangleCount = (Data >> 1U) & 0xFU;
      const std::uint32_t FirstTriangle = Data >> 5U;
      Closest = RaycastTriangleRange(
          LocalRay, Mesh.Triangles, FirstTriangle, TriangleCount,
          CullBackfaces, NegativeDeterminant, Closest, Control, Workspace);
      if (Closest.Status == QueryStatus::Undefined)
      {
        return Closest;
      }
    }

    const std::array<std::uint8_t, 4> Order =
        DecodeBv4PnsOrder(*Block, LocalRay.Direction);
    for (std::size_t Reverse = Order.size(); Reverse != 0; --Reverse)
    {
      const std::size_t Slot = Order[Reverse - 1U];
      if (Slot < ChildCount && Intersects[Slot] &&
          (Block->Data[Slot] & 1U) == 0U)
      {
        Stack.push_back(Block->Data[Slot]);
      }
    }
  }
  return Closest;
}

RayHit ConvertMeshHit(const Ray &QueryRay, const GeometryView &Geometry,
                      const MeshRay &LocalRay,
                      const TriangleHit &LocalHit) noexcept
{
  if (LocalHit.Status == QueryStatus::Undefined)
  {
    return UndefinedHit();
  }
  if (LocalHit.Status != QueryStatus::Hit)
  {
    return {};
  }
  const float WorldDistance =
      LocalHit.LocalDistance / LocalRay.DirectionLength;
  if (!InDistanceRange(WorldDistance, QueryRay.MaxDistance))
  {
    return {};
  }
  Vec3 WorldNormal = TransformMeshNormal(
      Geometry.WorldPose, Geometry.TriangleMesh.Scale,
      LocalHit.LocalNormal);
  if (!IsFinite(WorldNormal) || !(LengthSquared(WorldNormal) > 0.0f))
  {
    return UndefinedHit();
  }
  if (Geometry.TriangleMesh.DoubleSided &&
      Dot(WorldNormal, QueryRay.Direction) > 0.0f)
  {
    WorldNormal = Multiply(WorldNormal, -1.0f);
  }
  return MakeHit(QueryRay, WorldDistance, WorldNormal,
                 LocalHit.FaceIndex);
}

RayHit RaycastTriangleMesh(const Ray &QueryRay,
                           const GeometryView &Geometry,
                           bool HitBackfaces,
                           QueryControl *Control)
{
  const TriangleMeshGeometry &Mesh = Geometry.TriangleMesh;
  if (Mesh.Triangles.TriangleCount == 0)
  {
    return {};
  }
  if (Mesh.Triangles.TriangleCount > UINT32_MAX ||
      (Mesh.Triangles.Vertices.Data == nullptr &&
       Mesh.Triangles.LoadVertex == nullptr) ||
      (Mesh.Triangles.Indices == nullptr &&
       Mesh.Triangles.LoadIndices == nullptr))
  {
    return UndefinedHit();
  }
  MeshRay LocalRay{};
  if (!BuildMeshRay(QueryRay, Geometry.WorldPose, Mesh.Scale, &LocalRay))
  {
    return UndefinedHit();
  }
  const bool CullBackfaces = !(Mesh.DoubleSided || HitBackfaces);
  TriangleHit LocalHit{};
  try
  {
    MeshWorkspaceLease Lease(Control);
    if (Mesh.TreeType == TriangleTreeType::RTree)
    {
      LocalHit = RaycastRTree(LocalRay.Local, Mesh, CullBackfaces,
                              LocalRay.NegativeDeterminant, Control, Lease.Get());
    }
    else
    {
      LocalHit = RaycastBv4(LocalRay.Local, Mesh, CullBackfaces,
                            LocalRay.NegativeDeterminant, Control, Lease.Get());
    }
  }
  catch (const std::bad_alloc &)
  {
    if (Control != nullptr) Control->Failure = Error::OutOfMemory;
    return UndefinedHit();
  }
  catch (const std::length_error &)
  {
    if (Control != nullptr) Control->Failure = Error::OutOfMemory;
    return UndefinedHit();
  }
  return ConvertMeshHit(QueryRay, Geometry, LocalRay, LocalHit);
}

} // namespace

std::uint32_t DecodeRTreeLeafCount(std::uint32_t Data) noexcept
{
  return ((Data >> 1U) & 0xFU) + 1U;
}

std::uint32_t DecodeRTreeLeafFirst(std::uint32_t Data) noexcept
{
  return Data >> 5U;
}

std::array<std::uint8_t, 4> DecodeBv4PnsOrder(
    const Bv4QuantizedBlock &Block, const Vec3 &Direction) noexcept
{
  static constexpr std::array<std::array<std::uint8_t, 4>, 8> Orders = {{
      {{0, 1, 2, 3}},
      {{0, 1, 3, 2}},
      {{1, 0, 2, 3}},
      {{1, 0, 3, 2}},
      {{2, 3, 0, 1}},
      {{3, 2, 0, 1}},
      {{2, 3, 1, 0}},
      {{3, 2, 1, 0}},
  }};
  const std::uint32_t DirectionIndex =
      (std::signbit(Direction.Z) ? 1U : 0U) |
      (std::signbit(Direction.Y) ? 2U : 0U) |
      (std::signbit(Direction.X) ? 4U : 0U);
  const std::uint32_t DirectionMask = 1U << (3U + DirectionIndex);
  const std::uint32_t PnsIndex =
      ((Block.Data[0] & DirectionMask) != 0U ? 4U : 0U) |
      ((Block.Data[1] & DirectionMask) != 0U ? 2U : 0U) |
      ((Block.Data[2] & DirectionMask) != 0U ? 1U : 0U);
  return Orders[PnsIndex];
}

namespace
{

bool MultiplySizeWithoutOverflow(std::size_t Left, std::size_t Right,
                                 std::size_t *Output) noexcept
{
  if (Left != 0 &&
      Right > std::numeric_limits<std::size_t>::max() / Left)
  {
    return false;
  }
  *Output = Left * Right;
  return true;
}

struct HeightFieldRay
{
  Ray Local{};
  Quat PoseRotation{};
};

bool BuildHeightFieldRay(const Ray &QueryRay, const Pose &WorldPose,
                         HeightFieldRay *Output) noexcept
{
  if (!ResolvePoseRotation(WorldPose, &Output->PoseRotation))
  {
    return false;
  }
  Output->Local = QueryRay;
  Output->Local.Origin = RotateInverse(
      Output->PoseRotation,
      Subtract(QueryRay.Origin, WorldPose.Position));
  Output->Local.Direction =
      RotateInverse(Output->PoseRotation, QueryRay.Direction);
  return IsDefinedRay(Output->Local);
}

Vec3 HeightFieldVertex(const HeightFieldGeometry &HeightField,
                       std::uint32_t Row,
                       std::uint32_t Column,
                       const HeightFieldSample &Sample) noexcept
{
  return {
      static_cast<float>(Row) * HeightField.RowScale,
      static_cast<float>(Sample.Height) * HeightField.HeightScale,
      static_cast<float>(Column) * HeightField.ColumnScale,
  };
}

bool LoadHeightFieldCell(
    const HeightFieldGeometry &HeightField, std::uint32_t Row,
    std::uint32_t Column,
    std::array<HeightFieldSample, 4> *Output) noexcept
{
  if (Output == nullptr)
  {
    return false;
  }
  *Output = {};
  if (HeightField.RowCount < 2U || HeightField.ColumnCount < 2U ||
      Row >= HeightField.RowCount - 1U ||
      Column >= HeightField.ColumnCount - 1U)
  {
    return false;
  }
  if (HeightField.LoadCell != nullptr)
  {
    if (HeightField.LoadCell(HeightField.ProviderContext, Row, Column,
                             Output))
    {
      return true;
    }
    *Output = {};
    return false;
  }
  const std::size_t FirstIndex =
      static_cast<std::size_t>(Row) * HeightField.SampleColumnStride +
      Column;
  const std::size_t NextRowIndex =
      FirstIndex + HeightField.SampleColumnStride;
  if (HeightField.Samples.Data == nullptr ||
      NextRowIndex >= HeightField.Samples.Count ||
      HeightField.Samples.Count - NextRowIndex < 2U)
  {
    return false;
  }
  *Output = {
      HeightField.Samples.Data[FirstIndex],
      HeightField.Samples.Data[FirstIndex + 1U],
      HeightField.Samples.Data[NextRowIndex],
      HeightField.Samples.Data[NextRowIndex + 1U],
  };
  return true;
}

TriangleHit RaycastHeightFieldTriangle(
    const Ray &LocalRay, const std::array<Vec3, 3> &Vertices,
    std::uint32_t FaceIndex, bool CullBackfaces) noexcept
{
  const Vec3 EdgeOne = Subtract(Vertices[1], Vertices[0]);
  const Vec3 EdgeTwo = Subtract(Vertices[2], Vertices[0]);
  const Vec3 DirectionCross = Cross(LocalRay.Direction, EdgeTwo);
  const float Determinant = Dot(EdgeOne, DirectionCross);
  if ((CullBackfaces && Determinant <= KTriangleEpsilon) ||
      (!CullBackfaces && std::fabs(Determinant) <= KTriangleEpsilon))
  {
    return {};
  }
  const float InverseDeterminant = 1.0f / Determinant;
  const Vec3 OriginDelta = Subtract(LocalRay.Origin, Vertices[0]);
  const float U = Dot(OriginDelta, DirectionCross) * InverseDeterminant;
  if (U < 0.0f || U > 1.0f)
  {
    return {};
  }
  const Vec3 OriginCross = Cross(OriginDelta, EdgeOne);
  const float V = Dot(LocalRay.Direction, OriginCross) * InverseDeterminant;
  if (V < 0.0f || U + V > 1.0f)
  {
    return {};
  }
  const float Distance = Dot(EdgeTwo, OriginCross) * InverseDeterminant;
  if (!InDistanceRange(Distance, LocalRay.MaxDistance))
  {
    return {};
  }
  Vec3 Normal{};
  if (!Normalize(Cross(EdgeOne, EdgeTwo), &Normal))
  {
    TriangleHit Result{};
    Result.Status = QueryStatus::Undefined;
    return Result;
  }
  if (!CullBackfaces && Dot(Normal, LocalRay.Direction) > 0.0f)
  {
    Normal = Multiply(Normal, -1.0f);
  }
  TriangleHit Result{};
  Result.Status = QueryStatus::Hit;
  Result.LocalDistance = Distance;
  Result.LocalNormal = Normal;
  Result.FaceIndex = FaceIndex;
  return Result;
}

TriangleHit RaycastHeightFieldCell(const Ray &LocalRay,
                                   const HeightFieldGeometry &HeightField,
                                   std::uint32_t Row,
                                   std::uint32_t Column,
                                   bool CullBackfaces) noexcept
{
  const std::size_t SampleIndex =
      static_cast<std::size_t>(Row) *
          HeightField.SampleColumnStride +
      Column;
  std::array<HeightFieldSample, 4> Samples{};
  if (!LoadHeightFieldCell(HeightField, Row, Column, &Samples))
  {
    TriangleHit Result{};
    Result.Status = QueryStatus::Undefined;
    return Result;
  }
  const HeightFieldSample &Sample = Samples[0];
  const Vec3 Vertex0 = HeightFieldVertex(HeightField, Row, Column, Samples[0]);
  const Vec3 Vertex1 =
      HeightFieldVertex(HeightField, Row, Column + 1U, Samples[1]);
  const Vec3 Vertex2 =
      HeightFieldVertex(HeightField, Row + 1U, Column, Samples[2]);
  const Vec3 Vertex3 =
      HeightFieldVertex(HeightField, Row + 1U, Column + 1U, Samples[3]);
  if (!IsFinite(Vertex0) || !IsFinite(Vertex1) || !IsFinite(Vertex2) ||
      !IsFinite(Vertex3))
  {
    TriangleHit Result{};
    Result.Status = QueryStatus::Undefined;
    return Result;
  }
  const bool ZerothVertexShared = (Sample.Material0 & 0x80U) != 0U;
  std::array<std::array<Vec3, 3>, 2> Faces =
      ZerothVertexShared
          ? std::array<std::array<Vec3, 3>, 2>{{
                {{Vertex2, Vertex0, Vertex3}},
                {{Vertex1, Vertex3, Vertex0}},
            }}
          : std::array<std::array<Vec3, 3>, 2>{{
                {{Vertex0, Vertex1, Vertex2}},
                {{Vertex3, Vertex2, Vertex1}},
            }};
  if ((HeightField.RowScale < 0.0f) !=
      (HeightField.ColumnScale < 0.0f))
  {
    std::swap(Faces[0][1], Faces[0][2]);
    std::swap(Faces[1][1], Faces[1][2]);
  }
  const std::array<std::uint8_t, 2> Materials = {
      static_cast<std::uint8_t>(Sample.Material0 & 0x7FU),
      static_cast<std::uint8_t>(Sample.Material1 & 0x7FU),
  };

  TriangleHit Closest{};
  for (std::size_t Face = 0; Face < Faces.size(); ++Face)
  {
    if (Materials[Face] == 0x7FU)
    {
      continue;
    }
    Ray ClippedRay = LocalRay;
    if (Closest.Status == QueryStatus::Hit)
    {
      ClippedRay.MaxDistance = Closest.LocalDistance;
    }
    if (SampleIndex > (UINT32_MAX - Face) / 2U)
    {
      Closest.Status = QueryStatus::Undefined;
      return Closest;
    }
    const std::size_t FaceValue = SampleIndex * 2U + Face;
    const TriangleHit Candidate = RaycastHeightFieldTriangle(
        ClippedRay, Faces[Face], static_cast<std::uint32_t>(FaceValue),
        CullBackfaces);
    MergeMeshHit(Candidate, &Closest);
    if (Closest.Status == QueryStatus::Undefined)
    {
      return Closest;
    }
  }
  return Closest;
}

std::uint32_t GridCell(float Coordinate, std::uint32_t PointCount,
                       int Step) noexcept
{
  const float Maximum = static_cast<float>(PointCount - 1U);
  float Adjusted = std::clamp(Coordinate, 0.0f, Maximum);
  if (Adjusted == Maximum ||
      (Step < 0 && Adjusted == std::floor(Adjusted) && Adjusted > 0.0f))
  {
    Adjusted = std::nextafter(Adjusted,
                              -std::numeric_limits<float>::infinity());
  }
  const float Cell = std::floor(Adjusted);
  return static_cast<std::uint32_t>(
      std::clamp(Cell, 0.0f, static_cast<float>(PointCount - 2U)));
}

RayHit RaycastHeightField(const Ray &QueryRay,
                          const GeometryView &Geometry,
                          bool HitBackfaces,
                          QueryControl *Control)
{
  const HeightFieldGeometry &HeightField = Geometry.HeightField;
  std::size_t LastRowOffset = 0;
  std::size_t RequiredSampleCount = 0;
  const float NativeColumnCount = HeightField.NbColumns;
  if (HeightField.RowCount < 2U || HeightField.ColumnCount < 2U ||
      HeightField.SampleStride < sizeof(HeightFieldSample) ||
      HeightField.DataFormat != KVisibilityHeightFieldFormatS16Tm ||
      !std::isfinite(NativeColumnCount) ||
      std::floor(NativeColumnCount) != NativeColumnCount ||
      NativeColumnCount < 2.0f ||
      NativeColumnCount <
          static_cast<float>(HeightField.ColumnCount) ||
      !std::isfinite(HeightField.ConvexEdgeThreshold) ||
      HeightField.ConvexEdgeThreshold < 0.0f ||
      HeightField.SampleColumnStride < HeightField.ColumnCount ||
      !MultiplySizeWithoutOverflow(HeightField.RowCount - 1U,
                                   HeightField.SampleColumnStride,
                                   &LastRowOffset) ||
      LastRowOffset >
          std::numeric_limits<std::size_t>::max() -
              HeightField.ColumnCount ||
      (HeightField.LoadCell == nullptr &&
       (HeightField.Samples.Data == nullptr ||
        (RequiredSampleCount = LastRowOffset + HeightField.ColumnCount,
         RequiredSampleCount > HeightField.Samples.Count))) ||
      !std::isfinite(HeightField.HeightScale) ||
      !std::isfinite(HeightField.RowScale) ||
      !std::isfinite(HeightField.ColumnScale) ||
      HeightField.RowScale == 0.0f ||
      HeightField.ColumnScale == 0.0f ||
      !HeightField.LocalBoundsDefined ||
      !IsValidAabb(HeightField.LocalBounds))
  {
    return UndefinedHit();
  }

  HeightFieldRay LocalRay{};
  if (!BuildHeightFieldRay(QueryRay, Geometry.WorldPose, &LocalRay))
  {
    return UndefinedHit();
  }

  Aabb Bounds = HeightField.LocalBounds;
  const Vec3 Center = Multiply(Add(Bounds.Minimum, Bounds.Maximum), 0.5f);
  const Vec3 Extents =
      Multiply(Subtract(Bounds.Maximum, Bounds.Minimum), 0.505f);
  Bounds.Minimum = Subtract(Center, Extents);
  Bounds.Maximum = Add(Center, Extents);
  const RayHit BoundsHit = RaycastAabb(LocalRay.Local, Bounds);
  if (BoundsHit.Status != QueryStatus::Hit)
  {
    return BoundsHit;
  }

  const float GridOriginX =
      LocalRay.Local.Origin.X / HeightField.RowScale;
  const float GridOriginZ =
      LocalRay.Local.Origin.Z / HeightField.ColumnScale;
  const float GridDirectionX =
      LocalRay.Local.Direction.X / HeightField.RowScale;
  const float GridDirectionZ =
      LocalRay.Local.Direction.Z / HeightField.ColumnScale;
  const int StepX =
      GridDirectionX > 0.0f ? 1 : (GridDirectionX < 0.0f ? -1 : 0);
  const int StepZ =
      GridDirectionZ > 0.0f ? 1 : (GridDirectionZ < 0.0f ? -1 : 0);
  const float CurrentDistance =
      std::max(0.0f, BoundsHit.Distance - KRaySurfaceOffset);
  const Vec3 StartPoint =
      Add(LocalRay.Local.Origin,
          Multiply(LocalRay.Local.Direction, CurrentDistance));
  const float StartGridX = StartPoint.X / HeightField.RowScale;
  const float StartGridZ = StartPoint.Z / HeightField.ColumnScale;
  std::uint32_t Row =
      GridCell(StartGridX, HeightField.RowCount, StepX);
  std::uint32_t Column =
      GridCell(StartGridZ, HeightField.ColumnCount, StepZ);

  const float Infinity = std::numeric_limits<float>::infinity();
  float NextX = Infinity;
  float NextZ = Infinity;
  float DeltaX = Infinity;
  float DeltaZ = Infinity;
  if (StepX != 0)
  {
    const float Boundary = static_cast<float>(
        StepX > 0 ? Row + 1U : Row);
    NextX = (Boundary - GridOriginX) / GridDirectionX;
    DeltaX = std::fabs(1.0f / GridDirectionX);
  }
  if (StepZ != 0)
  {
    const float Boundary = static_cast<float>(
        StepZ > 0 ? Column + 1U : Column);
    NextZ = (Boundary - GridOriginZ) / GridDirectionZ;
    DeltaZ = std::fabs(1.0f / GridDirectionZ);
  }

  std::size_t CellCount = 0;
  if (!MultiplySizeWithoutOverflow(HeightField.RowCount - 1U,
                                   HeightField.ColumnCount - 1U,
                                   &CellCount))
  {
    return UndefinedHit();
  }
  const std::size_t VisitBudget = CellCount + 1U;
  TriangleHit Closest{};
  for (std::size_t Visit = 0; Visit < VisitBudget; ++Visit)
  {
    if (!ContinueQuery(Control))
    {
      return UndefinedHit();
    }
    Ray ClippedRay = LocalRay.Local;
    if (Closest.Status == QueryStatus::Hit)
    {
      ClippedRay.MaxDistance = Closest.LocalDistance;
    }
    const TriangleHit CellHit = RaycastHeightFieldCell(
        ClippedRay, HeightField, Row, Column,
        !(HeightField.DoubleSided || HitBackfaces));
    MergeMeshHit(CellHit, &Closest);
    if (Closest.Status == QueryStatus::Undefined)
    {
      return UndefinedHit();
    }

    const float NextDistance = std::min(NextX, NextZ);
    if (!std::isfinite(NextDistance) ||
        NextDistance > LocalRay.Local.MaxDistance ||
        (Closest.Status == QueryStatus::Hit &&
         NextDistance > Closest.LocalDistance))
    {
      break;
    }
    const bool AdvanceX = NextX <= NextZ;
    const bool AdvanceZ = NextZ <= NextX;
    if (AdvanceX)
    {
      const std::int64_t NextRow =
          static_cast<std::int64_t>(Row) + StepX;
      if (NextRow < 0 ||
          NextRow >= static_cast<std::int64_t>(HeightField.RowCount - 1U))
      {
        break;
      }
      Row = static_cast<std::uint32_t>(NextRow);
      NextX += DeltaX;
    }
    if (AdvanceZ)
    {
      const std::int64_t NextColumn =
          static_cast<std::int64_t>(Column) + StepZ;
      if (NextColumn < 0 ||
          NextColumn >=
              static_cast<std::int64_t>(HeightField.ColumnCount - 1U))
      {
        break;
      }
      Column = static_cast<std::uint32_t>(NextColumn);
      NextZ += DeltaZ;
    }
  }

  if (Closest.Status != QueryStatus::Hit)
  {
    return {};
  }
  const Vec3 WorldNormal = Rotate(LocalRay.PoseRotation,
                                  Closest.LocalNormal);
  return MakeHit(QueryRay, Closest.LocalDistance, WorldNormal,
                 Closest.FaceIndex);
}

} // namespace

RayHit RaycastGeometry(const Ray &QueryRay,
                       const GeometryView &Geometry,
                       bool HitBackfaces,
                       QueryControl *Control)
{
  if (!IsDefinedRay(QueryRay))
  {
    if (Control != nullptr) Control->Failure = Error::InvalidArgument;
    return UndefinedHit();
  }
  if (!ContinueQuery(Control))
  {
    return UndefinedHit();
  }
  switch (Geometry.Type)
  {
  case GeometryType::ConvexMesh:
    return RaycastConvex(QueryRay, Geometry, Control);
  case GeometryType::TriangleMesh:
    return RaycastTriangleMesh(QueryRay, Geometry, HitBackfaces, Control);
  case GeometryType::HeightField:
    return RaycastHeightField(QueryRay, Geometry, HitBackfaces, Control);
  }
  if (Control != nullptr) Control->Failure = Error::UnsupportedGeometry;
  return UndefinedHit();
}

} // namespace visibility
} // namespace physx_pack
