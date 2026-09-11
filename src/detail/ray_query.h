#ifndef PHYSX_PACK_RAY_QUERY_H
#define PHYSX_PACK_RAY_QUERY_H

#include "detail/visibility_diagnostics.h"
#include "physx_pack/visibility_query.h"

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
  Error Failure = Error::None;
};

struct RayHit
{
  QueryStatus Status = QueryStatus::Miss;
  float Distance = 0.0f;
  Vec3 Position{};
  Vec3 Normal{};
  std::uint32_t FaceIndex = UINT32_MAX;
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
  // Query leases provide direct reads from their retained immutable block.
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
  TriangleMesh,
  ConvexMesh,
  HeightField,
};

struct GeometryView
{
  GeometryType Type = GeometryType::TriangleMesh;
  Pose WorldPose{};
  ConvexGeometry Convex{};
  TriangleMeshGeometry TriangleMesh{};
  HeightFieldGeometry HeightField{};
};

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
std::uint32_t DecodeRTreeLeafCount(std::uint32_t Data) noexcept;
std::uint32_t DecodeRTreeLeafFirst(std::uint32_t Data) noexcept;
std::array<std::uint8_t, 4> DecodeBv4PnsOrder(
    const Bv4QuantizedBlock &Block, const Vec3 &Direction) noexcept;

} // namespace visibility
} // namespace physx_pack

#endif // PHYSX_PACK_RAY_QUERY_H
