#ifndef PHYSX_PACK_RAY_QUERY_H
#define PHYSX_PACK_RAY_QUERY_H

#include "physx_pack/ray_query_types.h"
#include "physx_pack/visibility_diagnostics.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <type_traits>
#include <vector>

namespace physx_pack
{
namespace visibility
{

inline constexpr std::size_t KVisibilityBoneCount = KPlayerBoneCount;
inline constexpr float KVisibilityConvexEndEpsilon = 1.0e-5f;
inline constexpr float KVisibilityPlaneEpsilon = 1.0e-7f;
inline constexpr std::uint32_t KVisibilityRTreePageStride = 0x70U;
inline constexpr std::uint32_t KVisibilityHeightFieldFormatS16Tm = 1U;

struct Vec3
{
  float X = 0.0f;
  float Y = 0.0f;
  float Z = 0.0f;
};

struct Quat
{
  float X = 0.0f;
  float Y = 0.0f;
  float Z = 0.0f;
  float W = 1.0f;
};

template <typename T>
struct ArrayView
{
  const T *Data = nullptr;
  std::size_t Count = 0;
};

struct Ray
{
  Vec3 Origin{};
  Vec3 Direction{};
  float MaxDistance = 0.0f;
};

struct PreparedAabbRay
{
  Ray Source{};
  std::array<float, 3> Origin{};
  std::array<float, 3> InverseDirection{};
  std::array<bool, 3> Parallel{};
  bool Defined = false;
};

struct Aabb
{
  Vec3 Minimum{};
  Vec3 Maximum{};
};

struct QueryRTreeEntry
{
  bool IsPage = false;
  std::uint32_t Value = 0;
  Aabb Bounds{};
  bool HasBounds = false;
};

struct MeshQueryWorkspace
{
  std::vector<QueryRTreeEntry> RTreeStack;
  std::vector<std::uint32_t> Bv4Stack;
  std::vector<std::uint32_t> RangeIndices;
};

struct QueryScratch
{
  std::vector<std::unique_ptr<MeshQueryWorkspace>> Workspaces;
  std::size_t Depth = 0;
};

struct Pose
{
  Vec3 Position{};
  Quat Rotation{};
};

struct MeshScale
{
  Vec3 Scale{1.0f, 1.0f, 1.0f};
  Quat Rotation{};
};

enum class QueryStatus : std::uint8_t
{
  Miss = 0,
  Hit,
  Undefined,
};

struct QueryControl
{
  void *Context = nullptr;
  bool (*Continue)(void *Context) = nullptr;
  bool Interrupted = false;
  QueryScratch *Scratch = nullptr;
};

struct RayHit
{
  QueryStatus Status = QueryStatus::Miss;
  float Distance = 0.0f;
  Vec3 Position{};
  Vec3 Normal{};
  std::uint32_t FaceIndex = UINT32_MAX;
};

struct SphereGeometry
{
  float Radius = 0.0f;
};

struct PlaneGeometry
{
  Vec3 Normal{1.0f, 0.0f, 0.0f};
  float Distance = 0.0f;
};

struct CapsuleGeometry
{
  float Radius = 0.0f;
  float HalfHeight = 0.0f;
};

struct BoxGeometry
{
  Vec3 HalfExtents{};
};

struct ConvexPlane
{
  Vec3 Normal{};
  float Distance = 0.0f;
};

struct ConvexGeometry
{
  ArrayView<ConvexPlane> Planes{};
  MeshScale Scale{};
};

enum class TriangleIndexWidth : std::uint8_t
{
  Index16 = 0,
  Index32,
};

struct TriangleData
{
  ArrayView<Vec3> Vertices{};
  const void *Indices = nullptr;
  std::size_t TriangleCount = 0;
  TriangleIndexWidth IndexWidth = TriangleIndexWidth::Index16;
  float Epsilon = 1.0e-7f;
  // 生产远端 Mesh 采用按遍历需求加载的回调；本地 fixture 继续使用上面的数组。
  void *ProviderContext = nullptr;
  bool (*LoadIndices)(void *Context, std::uint32_t TriangleIndex,
                      std::array<std::uint32_t, 3> *Output) = nullptr;
  bool (*LoadIndexRange)(void *Context, std::uint32_t FirstTriangle,
                         std::uint32_t TriangleCount,
                         std::uint32_t *Output) = nullptr;
  bool (*LoadVertex)(void *Context, std::uint32_t VertexIndex,
                     Vec3 *Output) = nullptr;
};

struct RTreePage
{
  std::array<Aabb, 4> Bounds{};
  std::array<std::uint32_t, 4> Data{};
  std::uint8_t ValidMask = 0;
};

struct RTreeView
{
  ArrayView<RTreePage> Pages{};
  std::uint32_t RootPageCount = 0;
  void *ProviderContext = nullptr;
  bool (*LoadPage)(void *Context, std::uint32_t PageIndex,
                   RTreePage *Output) = nullptr;
};

// BV4 的量化节点按轴保存四个交错的 signed min/max lane。
struct Bv4QuantizedLane
{
  std::int16_t Minimum = 0;
  std::int16_t Maximum = 0;
};

static_assert(std::is_standard_layout_v<Bv4QuantizedLane>,
              "BV4 quantized lane must be standard layout");
static_assert(sizeof(Bv4QuantizedLane) == 0x04,
              "BV4 quantized lane ABI changed");
static_assert(offsetof(Bv4QuantizedLane, Minimum) == 0x00,
              "BV4 quantized lane minimum offset changed");
static_assert(offsetof(Bv4QuantizedLane, Maximum) == 0x02,
              "BV4 quantized lane maximum offset changed");

struct Bv4QuantizedAxis
{
  Bv4QuantizedLane Axis[4]{};
};

static_assert(std::is_standard_layout_v<Bv4QuantizedAxis>,
              "BV4 quantized axis must be standard layout");
static_assert(std::is_trivially_copyable_v<Bv4QuantizedAxis>,
              "BV4 quantized axis must be trivially copyable");
static_assert(sizeof(Bv4QuantizedAxis) == 0x10,
              "BV4 quantized axis ABI changed");
static_assert(offsetof(Bv4QuantizedAxis, Axis) == 0x00,
              "BV4 quantized axis lane offset changed");
static_assert(sizeof(Bv4QuantizedLane) * 4U == 0x10,
              "BV4 quantized axis lane storage changed");
static_assert(offsetof(Bv4QuantizedAxis, Axis) +
                      offsetof(Bv4QuantizedLane, Minimum) ==
                  0x00,
              "BV4 lane 0 minimum ordering changed");
static_assert(offsetof(Bv4QuantizedAxis, Axis) +
                      sizeof(Bv4QuantizedLane) +
                      offsetof(Bv4QuantizedLane, Maximum) ==
                  0x06,
              "BV4 lane 1 maximum ordering changed");
static_assert(offsetof(Bv4QuantizedAxis, Axis) +
                      sizeof(Bv4QuantizedLane) * 3U +
                      offsetof(Bv4QuantizedLane, Maximum) ==
                  0x0E,
              "BV4 lane 3 maximum ordering changed");

struct Bv4QuantizedBlock
{
  Bv4QuantizedAxis X{};
  Bv4QuantizedAxis Y{};
  Bv4QuantizedAxis Z{};
  std::array<std::uint32_t, 4> Data{};
};

static_assert(sizeof(Bv4QuantizedBlock) == 0x40,
              "BV4 quantized swizzled block ABI changed");
static_assert(std::is_standard_layout_v<Bv4QuantizedBlock>,
              "BV4 quantized block must be standard layout");
static_assert(std::is_trivially_copyable_v<Bv4QuantizedBlock>,
              "BV4 quantized block must be trivially copyable");
static_assert(offsetof(Bv4QuantizedBlock, X) == 0x00,
              "BV4 quantized X offset changed");
static_assert(offsetof(Bv4QuantizedBlock, Y) == 0x10,
              "BV4 quantized Y offset changed");
static_assert(offsetof(Bv4QuantizedBlock, Z) == 0x20,
              "BV4 quantized Z offset changed");
static_assert(offsetof(Bv4QuantizedBlock, Data) == 0x30,
              "BV4 quantized child data offset changed");

struct Bv4View
{
  ArrayView<Bv4QuantizedBlock> Blocks{};
  std::uint32_t InitData = 0;
  Vec3 CenterCoefficient{1.0f, 1.0f, 1.0f};
  Vec3 ExtentCoefficient{1.0f, 1.0f, 1.0f};
  void *ProviderContext = nullptr;
  bool (*LoadBlock)(void *Context, std::uint32_t BlockIndex,
                    Bv4QuantizedBlock *Output) = nullptr;
};

enum class TriangleTreeType : std::uint8_t
{
  RTree = 0,
  Bv4,
};

struct TriangleMeshGeometry
{
  TriangleData Triangles{};
  TriangleTreeType TreeType = TriangleTreeType::RTree;
  RTreeView RTree{};
  Bv4View Bv4{};
  MeshScale Scale{};
  bool DoubleSided = false;
};

struct HeightFieldSample
{
  std::int16_t Height = 0;
  std::uint8_t Material0 = 0;
  std::uint8_t Material1 = 0;
};

static_assert(sizeof(HeightFieldSample) == 4,
              "HeightField sample ABI changed");

// 四角顺序固定为 [row,col]、[row,col+1]、[row+1,col]、[row+1,col+1]。
// Provider 必须清零输出并在内部处理异常，失败只使当前 ray 未定义。
using HeightFieldLoadCellFn = bool (*)(
    void *Context, std::uint32_t Row, std::uint32_t Column,
    std::array<HeightFieldSample, 4> *Output) noexcept;

struct HeightFieldGeometry
{
  std::uint32_t RowCount = 0;
  std::uint32_t ColumnCount = 0;
  std::uint32_t SampleColumnStride = 0;
  std::uint32_t SampleStride = sizeof(HeightFieldSample);
  ArrayView<HeightFieldSample> Samples{};
  void *ProviderContext = nullptr;
  HeightFieldLoadCellFn LoadCell = nullptr;
  float HeightScale = 1.0f;
  float RowScale = 1.0f;
  float ColumnScale = 1.0f;
  float NbColumns = 0.0f;
  float ConvexEdgeThreshold = 0.0f;
  std::uint16_t DataFlags = 0;
  std::uint32_t DataFormat = KVisibilityHeightFieldFormatS16Tm;
  Aabb LocalBounds{};
  bool LocalBoundsDefined = false;
  bool DoubleSided = false;
};

enum class GeometryType : std::uint8_t
{
  Sphere = 0,
  Plane,
  Capsule,
  Box,
  ConvexMesh,
  TriangleMesh,
  HeightField,
};

struct GeometryView
{
  GeometryType Type = GeometryType::Sphere;
  Pose WorldPose{};
  SphereGeometry Sphere{};
  PlaneGeometry Plane{};
  CapsuleGeometry Capsule{};
  BoxGeometry Box{};
  ConvexGeometry Convex{};
  TriangleMeshGeometry TriangleMesh{};
  HeightFieldGeometry HeightField{};
};

inline constexpr std::uint32_t KVisibilityStaticQueryFlag = 0x1U;
inline constexpr std::uint32_t KVisibilityDynamicQueryFlag = 0x2U;
inline constexpr std::uint32_t KVisibilityTriggerShapeFlag = 0x4U;

struct ShapeView
{
  Aabb WorldBounds{};
  GeometryView Geometry{};
  std::uint32_t FilterWord0 = 0;
  std::uint32_t ShapeFlags = 0;
  std::uint64_t Payload = 0;
};

struct PrunerView
{
  ArrayView<ShapeView> Shapes{};
};

struct SceneView
{
  PrunerView Static{};
  PrunerView Dynamic{};
  PrunerView Compound{};
};

struct QueryFilter
{
  std::uint32_t LayerMask = UINT32_MAX;
  std::uint32_t QueryFlags =
      KVisibilityStaticQueryFlag | KVisibilityDynamicQueryFlag;
  bool IncludeTriggers = false;
  bool HitBackfaces = false;
};

enum class QueryTriggerInteraction : std::int32_t
{
  UseGlobal = 0,
  Ignore = 1,
  Collide = 2,
};

struct QueryParameters
{
  std::uint32_t LayerMask = UINT32_MAX;
  std::int32_t HitMultipleFaces = 0;
  QueryTriggerInteraction HitTriggers = QueryTriggerInteraction::Ignore;
  std::int32_t HitBackfaces = 0;
};

static_assert(sizeof(QueryParameters) == 0x10,
              "query parameters ABI changed");

struct RaycastCommand
{
  Vec3 Origin{};
  Vec3 Direction{};
  std::int32_t SceneHandle = 0;
  float Distance = 0.0f;
  QueryParameters Query{};
};

static_assert(sizeof(RaycastCommand) == 0x30,
              "raycast command ABI changed");
static_assert(offsetof(RaycastCommand, Origin) == 0x00,
              "raycast command origin offset changed");
static_assert(offsetof(RaycastCommand, Direction) == 0x0C,
              "raycast command direction offset changed");
static_assert(offsetof(RaycastCommand, SceneHandle) == 0x18,
              "raycast command scene offset changed");
static_assert(offsetof(RaycastCommand, Distance) == 0x1C,
              "raycast command distance offset changed");
static_assert(offsetof(RaycastCommand, Query) == 0x20,
              "raycast command query offset changed");

enum class PrunerKind : std::uint8_t
{
  None = 0,
  Static,
  Dynamic,
  Compound,
};

struct SceneHit
{
  QueryStatus Status = QueryStatus::Miss;
  RayHit Hit{};
  PrunerKind Pruner = PrunerKind::None;
  std::size_t ShapeIndex = 0;
  std::uint64_t Payload = 0;
#if PHYSX_PACK_DIAGNOSTICS
  std::size_t VisitOrdinal = 0;
#endif
};

struct ManagedRaycastHit
{
  Vec3 Point{};
  Vec3 Normal{};
  std::uint32_t FaceIndex = 0;
  float Distance = 0.0f;
  float U = 0.0f;
  float V = 0.0f;
  std::int32_t ColliderInstanceId = 0;
};

static_assert(sizeof(ManagedRaycastHit) == 0x2C,
              "managed raycast hit ABI changed");
static_assert(offsetof(ManagedRaycastHit, Point) == 0x00,
              "managed raycast point offset changed");
static_assert(offsetof(ManagedRaycastHit, Normal) == 0x0C,
              "managed raycast normal offset changed");
static_assert(offsetof(ManagedRaycastHit, FaceIndex) == 0x18,
              "managed raycast face offset changed");
static_assert(offsetof(ManagedRaycastHit, Distance) == 0x1C,
              "managed raycast distance offset changed");
static_assert(offsetof(ManagedRaycastHit, U) == 0x20,
              "managed raycast u offset changed");
static_assert(offsetof(ManagedRaycastHit, V) == 0x24,
              "managed raycast v offset changed");
static_assert(offsetof(ManagedRaycastHit, ColliderInstanceId) == 0x28,
              "managed raycast collider offset changed");

enum class ExposedPriority : std::uint8_t
{
  Head = 0,
  Chest,
};

struct ExposedBoneCandidate
{
  Vec3 Position{};
  bool Valid = false;
  bool Visible = false;
  bool InsideAimFov = false;
};

inline constexpr std::array<std::size_t, KVisibilityBoneCount>
    KExposedHeadPriority = {
        0,
        1,
        14,
        15,
        16,
        2,
        8,
        3,
        9,
        4,
        10,
        5,
        11,
        6,
        12,
        7,
        13,
        17,
        21,
        18,
        22,
        19,
        23,
        20,
        24,
};

inline constexpr std::array<std::size_t, KVisibilityBoneCount>
    KExposedChestPriority = {
        14,
        15,
        16,
        1,
        0,
        2,
        8,
        3,
        9,
        4,
        10,
        5,
        11,
        6,
        12,
        7,
        13,
        17,
        21,
        18,
        22,
        19,
        23,
        20,
        24,
};

constexpr bool IsCompleteBonePriority(
    const std::array<std::size_t, KVisibilityBoneCount> &Priority)
{
  std::array<bool, KVisibilityBoneCount> Seen{};
  for (std::size_t Slot : Priority)
  {
    if (Slot >= Seen.size() || Seen[Slot])
    {
      return false;
    }
    Seen[Slot] = true;
  }
  for (bool Present : Seen)
  {
    if (!Present)
    {
      return false;
    }
  }
  return true;
}

static_assert(IsCompleteBonePriority(KExposedHeadPriority),
              "head exposed priority must be a complete permutation");
static_assert(IsCompleteBonePriority(KExposedChestPriority),
              "chest exposed priority must be a complete permutation");

bool IsFinite(const Vec3 &Value) noexcept;
bool IsDefinedRay(const Ray &Value) noexcept;

RayHit RaycastAabb(const Ray &QueryRay, const Aabb &Bounds) noexcept;
PreparedAabbRay PrepareAabbRay(const Ray &QueryRay) noexcept;
RayHit RaycastAabb(const PreparedAabbRay &Prepared, float MaxDistance,
                   const Aabb &Bounds) noexcept;
RayHit RaycastGeometry(const Ray &QueryRay,
                       const GeometryView &Geometry,
                       bool HitBackfaces = false,
                       QueryControl *Control = nullptr);
SceneHit RaycastScene(const Ray &QueryRay, const SceneView &Scene,
                      const QueryFilter &Filter,
                      QueryControl *Control = nullptr);
QueryStatus RaycastCommandClosest(
    const RaycastCommand &Command, const SceneView &Scene,
    bool GlobalQueriesHitTriggers, std::size_t CommandIndex,
    std::size_t MaxHits, ManagedRaycastHit *Results,
    std::size_t ResultCount, QueryControl *Control = nullptr);

std::uint32_t DecodeRTreeLeafCount(std::uint32_t Data) noexcept;
std::uint32_t DecodeRTreeLeafFirst(std::uint32_t Data) noexcept;
std::array<std::uint8_t, 4> DecodeBv4PnsOrder(
    const Bv4QuantizedBlock &Block, const Vec3 &Direction) noexcept;

int SelectExposedBoneSlot(
    const std::array<ExposedBoneCandidate, KVisibilityBoneCount> &Candidates,
    ExposedPriority Priority) noexcept;

} // namespace visibility
} // namespace physx_pack

#endif // PHYSX_PACK_RAY_QUERY_H
