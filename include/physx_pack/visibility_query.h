#ifndef PHYSX_PACK_PUBLIC_VISIBILITY_QUERY_H
#define PHYSX_PACK_PUBLIC_VISIBILITY_QUERY_H
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>

namespace physx_pack
{
inline constexpr unsigned ApiVersion = 2;
using InstanceId = std::uint64_t;
enum class VisibilityState
{
  Visible,
  Blocked,
  Unknown
};
enum class RaycastState
{
  Miss,
  Hit,
  Unknown
};
enum class Error
{
  None,
  InvalidArgument,
  NotOpen,
  IoError,
  CorruptData,
  UnsupportedGeometry,
  MissingResource,
  MissingVariant,
  BudgetExceeded,
  OutOfMemory,
  TargetNotFound,
  TargetNotHit,
};
enum class MeshVariant
{
  Automatic,
  Triangle,
  Convex
};
struct Vec3
{
  float X = 0, Y = 0, Z = 0;
};
struct Pose
{
  Vec3 Position{};
  float RotationX = 0, RotationY = 0, RotationZ = 0, RotationW = 1;
};
struct Ray
{
  Vec3 Origin{}, Direction{};
  float MaxDistance = 0;
};
struct ResourceKey
{
  std::string Origin, BundleId, SerializedFile;
  std::uint64_t PathId = 0;
  bool operator==(const ResourceKey &Other) const noexcept
  {
    return PathId == Other.PathId && Origin == Other.Origin && BundleId == Other.BundleId &&
           SerializedFile == Other.SerializedFile;
  }
};
struct InstanceDesc
{
  InstanceId Id = 0;
  ResourceKey Resource{};
  Pose WorldPose{};
  Vec3 Scale{1, 1, 1};
  bool DoubleSided = false;
  MeshVariant MeshType = MeshVariant::Automatic;
};
struct QueryRequest
{
  InstanceId Target = 0;
  Ray QueryRay{};
};
struct RaycastRequest
{
  Ray QueryRay{};
  // Optional instance excluded from the scene trace (typically the target
  // entity itself in a camera-to-bone visibility query).
  InstanceId IgnoreInstance = 0;
};
struct RaycastResult
{
  RaycastState State = RaycastState::Unknown;
  Error Reason = Error::NotOpen;
  float Distance = std::numeric_limits<float>::infinity();
  InstanceId HitInstance = 0;
};
struct VisibilityResult
{
  VisibilityState State = VisibilityState::Unknown;
  Error Reason = Error::NotOpen;
  float TargetDistance = std::numeric_limits<float>::infinity();
  InstanceId BlockingInstance = 0;
  float BlockingDistance = std::numeric_limits<float>::infinity();
  InstanceId ProblemInstance = 0;
};
struct PackageOptions
{
  std::size_t CacheBudgetBytes = 128U * 1024U * 1024U;
};
struct PackageInfo
{
  std::uint64_t FileBytes = 0, SourceCount = 0, BlockCount = 0, PreloadCount = 0, IndexBytes = 0;
};
struct CacheStats
{
  std::uint64_t BudgetBytes = 0, ResidentBytes = 0, PeakResidentBytes = 0;
  std::uint64_t TemporaryBytes = 0, PeakTemporaryBytes = 0;
  std::uint64_t Hits = 0, Misses = 0, Evictions = 0, BytesDecoded = 0;
  std::uint64_t GeometryMetadataBytes = 0;
};
struct SceneStats
{
  std::uint64_t Generation = 0, InstanceCount = 0, BvhBuilds = 0, BvhRefits = 0;
  std::uint64_t ResourceResolutions = 0, UnboundedInstances = 0;
};
struct QueryStats
{
  std::uint64_t Queries = 0, GeometryTests = 0, BoundsTests = 0;
};
namespace detail
{
struct PackageData;
struct SceneData;
} // namespace detail

class Package final
{
public:
  static std::shared_ptr<Package> Open(const char *Path, const PackageOptions &Options = {},
                                       Error *Failure = nullptr) noexcept;
  ~Package() noexcept;
  Package(const Package &) = delete;
  Package &operator=(const Package &) = delete;
  PackageInfo GetInfo() const noexcept;
  CacheStats GetCacheStats() const noexcept;

private:
  explicit Package(std::shared_ptr<detail::PackageData> Data) noexcept;
  std::shared_ptr<detail::PackageData> Data_;
  friend class Scene;
};
class SceneSnapshot final
{
public:
  SceneSnapshot() noexcept = default;
  explicit operator bool() const noexcept
  {
    return static_cast<bool>(Data_);
  }
  std::uint64_t Generation() const noexcept;
  std::size_t InstanceCount() const noexcept;

private:
  std::shared_ptr<const detail::SceneData> Data_;
  friend class Scene;
  friend class QueryContext;
};
class Scene final
{
public:
  explicit Scene(std::shared_ptr<Package> Package);
  ~Scene() noexcept;
  Scene(const Scene &) = delete;
  Scene &operator=(const Scene &) = delete;
  // Transactions are serialized; snapshots remain valid across updates.
  Error ApplyUpdates(const InstanceDesc *Upserts, std::size_t UpsertCount,
                     const InstanceId *Removals = nullptr, std::size_t RemovalCount = 0) noexcept;
  SceneSnapshot GetSnapshot() const noexcept;
  SceneStats GetStats() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> Impl_;
};
class QueryContext final
{
public:
  QueryContext();
  ~QueryContext() noexcept;
  QueryContext(const QueryContext &) = delete;
  QueryContext &operator=(const QueryContext &) = delete;
  // A context belongs to one worker; different contexts can share snapshots.
  VisibilityResult Check(const SceneSnapshot &Snapshot, const QueryRequest &Request) noexcept;
  Error CheckBatch(const SceneSnapshot &Snapshot, const QueryRequest *Requests, std::size_t Count,
                   VisibilityResult *Results) noexcept;
  Error TraceBatch(const SceneSnapshot &Snapshot, const RaycastRequest *Requests,
                   std::size_t Count, RaycastResult *Results) noexcept;
  QueryStats GetStats() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> Impl_;
};
const char *ToString(Error Value) noexcept;
const char *ToString(VisibilityState Value) noexcept;
const char *ToString(RaycastState Value) noexcept;
} // namespace physx_pack
#endif
