#ifndef PHYSX_PACK_RUNTIME_H
#define PHYSX_PACK_RUNTIME_H
#include "detail/geometry_reader.h"
#include "detail/pack_reader.h"
#include "physx_pack/visibility_query.h"
#include <algorithm>
#include <cmath>
#include <vector>

namespace physx_pack::detail
{
namespace vis = physx_pack::visibility;
struct PackageData
{
  explicit PackageData(std::size_t Budget) : Reader(Budget), Geometry(Reader) {}
  PackReader Reader;
  GeometryStore Geometry;
};
struct PreparedInstance
{
  InstanceDesc Desc;
  std::uint32_t Slot = UINT32_MAX;
  vis::GeometryType Type = vis::GeometryType::TriangleMesh;
  Error ResourceError = Error::MissingResource;
  vis::Aabb LocalBounds{}, WorldBounds{};
  bool LocalBoundsDefined = false, WorldBoundsDefined = false;
};
struct BvhNode
{
  vis::Aabb Bounds{};
  std::uint32_t Begin = 0, Count = 0, Left = 0, Right = 0;
};
struct SceneData
{
  std::shared_ptr<PackageData> Package;
  std::vector<PreparedInstance> Instances;
  std::vector<std::uint32_t> Order, Unbounded;
  std::vector<BvhNode> Nodes;
  SceneStats Stats{};
};
inline bool Finite(const Vec3 &V) noexcept
{
  return std::isfinite(V.X) && std::isfinite(V.Y) && std::isfinite(V.Z);
}
inline vis::Vec3 Vector(const Vec3 &V) noexcept
{
  return {V.X, V.Y, V.Z};
}
inline vis::Pose GeometryPose(const Pose &P) noexcept
{
  return {Vector(P.Position), {P.RotationX, P.RotationY, P.RotationZ, P.RotationW}};
}
inline bool ValidBounds(const vis::Aabb &B) noexcept
{
  return vis::IsFinite(B.Minimum) && vis::IsFinite(B.Maximum) && B.Minimum.X <= B.Maximum.X &&
         B.Minimum.Y <= B.Maximum.Y && B.Minimum.Z <= B.Maximum.Z;
}
inline vis::Aabb MergeBounds(const vis::Aabb &A, const vis::Aabb &B) noexcept
{
  return {{std::min(A.Minimum.X, B.Minimum.X), std::min(A.Minimum.Y, B.Minimum.Y),
           std::min(A.Minimum.Z, B.Minimum.Z)},
          {std::max(A.Maximum.X, B.Maximum.X), std::max(A.Maximum.Y, B.Maximum.Y),
           std::max(A.Maximum.Z, B.Maximum.Z)}};
}
inline vis::Aabb ScaledBounds(const vis::Aabb &B, const Vec3 &S) noexcept
{
  const vis::Vec3 A{B.Minimum.X * S.X, B.Minimum.Y * S.Y, B.Minimum.Z * S.Z};
  const vis::Vec3 C{B.Maximum.X * S.X, B.Maximum.Y * S.Y, B.Maximum.Z * S.Z};
  return {{std::min(A.X, C.X), std::min(A.Y, C.Y), std::min(A.Z, C.Z)},
          {std::max(A.X, C.X), std::max(A.Y, C.Y), std::max(A.Z, C.Z)}};
}
inline bool Transient(Error E) noexcept
{
  return E == Error::BudgetExceeded || E == Error::OutOfMemory || E == Error::IoError;
}
} // namespace physx_pack::detail
#endif
