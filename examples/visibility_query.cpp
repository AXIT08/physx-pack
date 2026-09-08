#include "physx_pack/visibility_query.h"

#include "physx_pack/geometry_reader.h"
#include "physx_pack/pack_reader.h"
#include "physx_pack/ray_query.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace
{

namespace detail = physx_pack::detail;
namespace visibility = physx_pack::visibility;

bool IsFinite(const physx_pack::Vec3 &Value) noexcept
{
  return std::isfinite(Value.X) && std::isfinite(Value.Y) &&
         std::isfinite(Value.Z);
}

bool IsValidRay(const physx_pack::Ray &Ray) noexcept
{
  return IsFinite(Ray.Origin) && IsFinite(Ray.Direction) &&
         std::isfinite(Ray.MaxDistance) && Ray.MaxDistance > 0.0f;
}

bool IsValidPose(const physx_pack::Pose &Pose) noexcept
{
  return IsFinite(Pose.Position) && std::isfinite(Pose.RotationX) &&
         std::isfinite(Pose.RotationY) && std::isfinite(Pose.RotationZ) &&
         std::isfinite(Pose.RotationW);
}

bool IsValidScale(const physx_pack::Vec3 &Scale) noexcept
{
  return IsFinite(Scale) && Scale.X != 0.0f && Scale.Y != 0.0f &&
         Scale.Z != 0.0f;
}

bool SameTarget(const physx_pack::TargetId &Left,
                const physx_pack::TargetId &Right) noexcept
{
  return Left.PathId == Right.PathId && Left.Origin == Right.Origin &&
         Left.BundleId == Right.BundleId &&
         Left.SerializedFile == Right.SerializedFile;
}

visibility::Pose ToVisibilityPose(const physx_pack::Pose &Pose) noexcept
{
  visibility::Pose Result{};
  Result.Position = {Pose.Position.X, Pose.Position.Y, Pose.Position.Z};
  Result.Rotation = {Pose.RotationX, Pose.RotationY, Pose.RotationZ,
                     Pose.RotationW};
  return Result;
}

visibility::MeshScale ToMeshScale(const physx_pack::Vec3 &Scale) noexcept
{
  visibility::MeshScale Result{};
  Result.Scale = {Scale.X, Scale.Y, Scale.Z};
  return Result;
}

bool BindInstance(detail::PackReader &Package,
                  const physx_pack::TargetInstance &Instance,
                  detail::GeometryReader &Provider,
                  visibility::GeometryView *Geometry) noexcept
{
  if (Geometry == nullptr || Instance.Id.Origin.empty() ||
      Instance.Id.BundleId.empty() || Instance.Id.SerializedFile.empty() ||
      !IsValidPose(Instance.WorldPose) || !IsValidScale(Instance.Scale))
  {
    return false;
  }
  detail::SourceKey Source{Instance.Id.Origin, Instance.Id.BundleId,
                                   Instance.Id.SerializedFile, Instance.Id.PathId};
  std::uint32_t Slot = detail::PackReader::KInvalidGeometrySlot;
  if (!Package.LookupSource(Source, &Slot))
  {
    return false;
  }
  detail::GeometryKind Kind = detail::GeometryKind::Unknown;
  if (!Package.GetGeometryKind(Slot, &Kind))
  {
    return false;
  }
  visibility::GeometryType Type = visibility::GeometryType::TriangleMesh;
  switch (Kind)
  {
  case detail::GeometryKind::TriangleMesh:
    break;
  case detail::GeometryKind::ConvexMesh:
    Type = visibility::GeometryType::ConvexMesh;
    break;
  case detail::GeometryKind::HeightField:
    Type = visibility::GeometryType::HeightField;
    break;
  default:
    return false;
  }
  visibility::GeometryView Result{};
  Result.WorldPose = ToVisibilityPose(Instance.WorldPose);
  if (Type == visibility::GeometryType::TriangleMesh)
  {
    Result.TriangleMesh.Scale = ToMeshScale(Instance.Scale);
    Result.TriangleMesh.DoubleSided = Instance.DoubleSided;
  }
  else if (Type == visibility::GeometryType::ConvexMesh)
  {
    Result.Convex.Scale = ToMeshScale(Instance.Scale);
  }
  else
  {
    Result.HeightField.HeightScale = Instance.Scale.Y;
    Result.HeightField.RowScale = Instance.Scale.X;
    Result.HeightField.ColumnScale = Instance.Scale.Z;
    Result.HeightField.DoubleSided = Instance.DoubleSided;
  }
  if (!Provider.Bind(Package, Slot, Type, &Result))
  {
    *Geometry = {};
    return false;
  }
  Result.WorldPose = ToVisibilityPose(Instance.WorldPose);
  if (Type == visibility::GeometryType::TriangleMesh)
  {
    Result.TriangleMesh.Scale = ToMeshScale(Instance.Scale);
    Result.TriangleMesh.DoubleSided = Instance.DoubleSided;
  }
  else if (Type == visibility::GeometryType::ConvexMesh)
  {
    Result.Convex.Scale = ToMeshScale(Instance.Scale);
  }
  else
  {
    auto &Height = Result.HeightField;
    Height.HeightScale = Instance.Scale.Y;
    Height.RowScale = Instance.Scale.X;
    Height.ColumnScale = Instance.Scale.Z;
    Height.DoubleSided = Instance.DoubleSided;
    if (Height.LocalBoundsDefined)
    {
      const auto Unit = Height.LocalBounds;
      const visibility::Vec3 First{Unit.Minimum.X * Height.RowScale,
                                   Unit.Minimum.Y * Height.HeightScale,
                                   Unit.Minimum.Z * Height.ColumnScale};
      const visibility::Vec3 Second{Unit.Maximum.X * Height.RowScale,
                                    Unit.Maximum.Y * Height.HeightScale,
                                    Unit.Maximum.Z * Height.ColumnScale};
      Height.LocalBounds = {
          {std::min(First.X, Second.X), std::min(First.Y, Second.Y),
           std::min(First.Z, Second.Z)},
          {std::max(First.X, Second.X), std::max(First.Y, Second.Y),
           std::max(First.Z, Second.Z)}};
      Height.LocalBoundsDefined =
          visibility::IsFinite(Height.LocalBounds.Minimum) &&
          visibility::IsFinite(Height.LocalBounds.Maximum);
    }
  }
  *Geometry = Result;
  return true;
}

} // namespace

namespace physx_pack
{

class VisibilityQuery::Impl final
{
public:
  ~Impl() noexcept { Close(); }

  bool Open(const char *Path) noexcept
  {
    Close();
    return Path != nullptr && *Path != '\0' && Package.Open(Path);
  }

  void Close() noexcept { Package.Close(); }
  bool IsOpen() const noexcept { return Package.IsOpen(); }

  VisibilityState Check(const TargetInstance &Target,
                        const SceneSnapshot &Scene,
                        const Ray &QueryRay) noexcept
  {
    if (!IsOpen() || Target.Id.Origin.empty() || Target.Id.BundleId.empty() ||
        Target.Id.SerializedFile.empty() || !IsValidPose(Target.WorldPose) ||
        !IsValidScale(Target.Scale) || !IsValidRay(QueryRay))
    {
      return VisibilityState::Unknown;
    }
    bool TargetSeen = false;
    bool UnknownInstance = false;
    float TargetDistance = std::numeric_limits<float>::infinity();
    float BlockingDistance = std::numeric_limits<float>::infinity();
    detail::GeometryReader Provider;
    for (const auto &Instance : Scene.Instances)
    {
      visibility::GeometryView Geometry{};
      if (!BindInstance(Package, Instance, Provider, &Geometry))
      {
        UnknownInstance = true;
        continue;
      }
      visibility::Ray VisibilityRay{};
      VisibilityRay.Origin = {QueryRay.Origin.X, QueryRay.Origin.Y,
                              QueryRay.Origin.Z};
      VisibilityRay.Direction = {QueryRay.Direction.X, QueryRay.Direction.Y,
                                 QueryRay.Direction.Z};
      VisibilityRay.MaxDistance = QueryRay.MaxDistance;
      const visibility::RayHit Hit = visibility::RaycastGeometry(
          VisibilityRay, Geometry, Instance.DoubleSided);
      if (Hit.Status == visibility::QueryStatus::Undefined)
      {
        UnknownInstance = true;
        continue;
      }
      if (Hit.Status != visibility::QueryStatus::Hit)
      {
        continue;
      }
      if (SameTarget(Instance.Id, Target.Id))
      {
        TargetSeen = true;
        TargetDistance = std::min(TargetDistance, Hit.Distance);
      }
      else
      {
        BlockingDistance = std::min(BlockingDistance, Hit.Distance);
      }
    }
    if (!TargetSeen || UnknownInstance)
    {
      return VisibilityState::Unknown;
    }
    return BlockingDistance <= TargetDistance ? VisibilityState::Blocked
                                               : VisibilityState::Visible;
  }

private:
  detail::PackReader Package;
};

VisibilityQuery::VisibilityQuery() noexcept : Impl_(std::make_unique<Impl>())
{
}

VisibilityQuery::VisibilityQuery(const char *PackPath) noexcept
    : VisibilityQuery()
{
  Open(PackPath);
}

VisibilityQuery::~VisibilityQuery() noexcept = default;
VisibilityQuery::VisibilityQuery(VisibilityQuery &&) noexcept = default;
VisibilityQuery &VisibilityQuery::operator=(VisibilityQuery &&) noexcept =
    default;

bool VisibilityQuery::Open() noexcept
{
  return Open(detail::KPackPackPath);
}

bool VisibilityQuery::Open(const char *PackPath) noexcept
{
  return Impl_ != nullptr && Impl_->Open(PackPath);
}

void VisibilityQuery::Close() noexcept
{
  if (Impl_ != nullptr)
  {
    Impl_->Close();
  }
}

bool VisibilityQuery::IsOpen() const noexcept
{
  return Impl_ != nullptr && Impl_->IsOpen();
}

VisibilityState VisibilityQuery::Check(const TargetInstance &Target,
                                       const SceneSnapshot &Scene,
                                       const Ray &QueryRay) noexcept
{
  return Impl_ == nullptr ? VisibilityState::Unknown
                          : Impl_->Check(Target, Scene, QueryRay);
}

} // namespace physx_pack
