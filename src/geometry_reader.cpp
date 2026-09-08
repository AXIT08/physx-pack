#include "physx_pack/geometry_reader.h"

#include "physx_pack/geometry_format.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

namespace physx_pack
{
namespace detail
{
namespace
{

namespace vis = visibility;

bool Range(const BlockView &Block, std::uint64_t Offset,
           std::uint64_t Count, std::uint64_t Stride = 1U) noexcept
{
  return Stride != 0U && Offset <= Block.Size() &&
         Count <= (Block.Size() - Offset) / Stride;
}

std::uint32_t U32(const BlockView &Block, std::size_t Offset) noexcept
{
  const std::uint8_t *Bytes = Block.Data() + Offset;
  return static_cast<std::uint32_t>(Bytes[0]) |
         (static_cast<std::uint32_t>(Bytes[1]) << 8U) |
         (static_cast<std::uint32_t>(Bytes[2]) << 16U) |
         (static_cast<std::uint32_t>(Bytes[3]) << 24U);
}

std::uint64_t U64(const BlockView &Block, std::size_t Offset) noexcept
{
  return U32(Block, Offset) |
         (static_cast<std::uint64_t>(U32(Block, Offset + 4U)) << 32U);
}

std::int32_t I32(const BlockView &Block, std::size_t Offset) noexcept
{
  const std::uint32_t Bits = U32(Block, Offset);
  std::int32_t Value = 0;
  std::memcpy(&Value, &Bits, sizeof(Value));
  return Value;
}

float F32(const BlockView &Block, std::size_t Offset) noexcept
{
  const std::uint32_t Bits = U32(Block, Offset);
  float Value = 0.0f;
  std::memcpy(&Value, &Bits, sizeof(Value));
  return Value;
}

vis::Vec3 Vector(const BlockView &Block, std::size_t Offset) noexcept
{
  return {F32(Block, Offset), F32(Block, Offset + 4U), F32(Block, Offset + 8U)};
}

std::int16_t I16(const std::uint8_t *Bytes) noexcept
{
  const std::uint16_t Bits = static_cast<std::uint16_t>(Bytes[0]) |
                             static_cast<std::uint16_t>(Bytes[1] << 8U);
  std::int16_t Value = 0;
  std::memcpy(&Value, &Bits, sizeof(Value));
  return Value;
}

} // namespace

struct GeometryReader::State
{
  PackReader *Package = nullptr;
  BlockView Block;
  std::uint32_t VertexCount = 0;
  std::uint32_t TriangleCount = 0;
  std::uint32_t NodeCount = 0;
  std::uint32_t IndexBytes = 0;
  std::uint64_t VertexOffset = 0;
  std::uint64_t IndexOffset = 0;
  std::uint64_t NodeOffset = 0;
  std::vector<vis::ConvexPlane> Planes;
  std::uint32_t Rows = 0;
  std::uint32_t Columns = 0;
  std::uint32_t TileRows = 0;
  std::uint32_t TileColumns = 0;
  std::uint64_t TileOffset = 0;
  void *ContinuationContext = nullptr;
  bool (*Continuation)(void *) = nullptr;

  bool AcquireBlock(std::uint32_t Id, BlockView *Output)
  {
    *Output = {};
    return (Continuation == nullptr || Continuation(ContinuationContext)) &&
           Package != nullptr && Package->AcquireBlock(Id, Output);
  }

  static bool LoadIndices(void *Context, std::uint32_t TriangleIndex,
                          std::array<std::uint32_t, 3> *Output) noexcept
  {
    if (Output == nullptr)
    {
      return false;
    }
    *Output = {};
    try
    {
      const auto *Self = static_cast<const State *>(Context);
      if (Self == nullptr || TriangleIndex >= Self->TriangleCount)
      {
        return false;
      }
      const std::uint64_t Offset =
          Self->IndexOffset +
          static_cast<std::uint64_t>(TriangleIndex) * 3U * Self->IndexBytes;
      if (!Range(Self->Block, Offset, 3U, Self->IndexBytes))
      {
        return false;
      }
      std::array<std::uint32_t, 3> Result{};
      for (std::size_t Index = 0; Index < Result.size(); ++Index)
      {
        const std::size_t At =
            static_cast<std::size_t>(Offset + Index * Self->IndexBytes);
        Result[Index] = Self->IndexBytes == 4U
                            ? U32(Self->Block, At)
                            : static_cast<std::uint32_t>(
                                  Self->Block.Data()[At] |
                                  (Self->Block.Data()[At + 1U] << 8U));
        if (Result[Index] >= Self->VertexCount)
        {
          return false;
        }
      }
      *Output = Result;
      return true;
    }
    catch (...)
    {
      return false;
    }
  }

  static bool LoadVertex(void *Context, std::uint32_t VertexIndex,
                         vis::Vec3 *Output) noexcept
  {
    if (Output == nullptr)
    {
      return false;
    }
    *Output = {};
    try
    {
      const auto *Self = static_cast<const State *>(Context);
      if (Self == nullptr || VertexIndex >= Self->VertexCount)
      {
        return false;
      }
      const std::uint64_t Offset =
          Self->VertexOffset + static_cast<std::uint64_t>(VertexIndex) * 12U;
      if (!Range(Self->Block, Offset, 12U))
      {
        return false;
      }
      const vis::Vec3 Value =
          Vector(Self->Block, static_cast<std::size_t>(Offset));
      if (!vis::IsFinite(Value))
      {
        return false;
      }
      *Output = Value;
      return true;
    }
    catch (...)
    {
      return false;
    }
  }

  static bool LoadPage(void *Context, std::uint32_t PageIndex,
                       vis::RTreePage *Output) noexcept
  {
    if (Output == nullptr)
    {
      return false;
    }
    *Output = {};
    try
    {
      const auto *Self = static_cast<const State *>(Context);
      if (Self == nullptr || PageIndex >= Self->NodeCount)
      {
        return false;
      }
      const std::uint64_t Offset =
          Self->NodeOffset + static_cast<std::uint64_t>(PageIndex) * 112U;
      if (!Range(Self->Block, Offset, 112U))
      {
        return false;
      }
      const std::size_t Base = static_cast<std::size_t>(Offset);
      vis::RTreePage Result{};
      for (std::size_t Lane = 0; Lane < 4U; ++Lane)
      {
        const std::size_t At = Base + Lane * 4U;
        const vis::Vec3 Minimum{F32(Self->Block, At),
                                F32(Self->Block, At + 16U),
                                F32(Self->Block, At + 32U)};
        const vis::Vec3 Maximum{F32(Self->Block, At + 48U),
                                F32(Self->Block, At + 64U),
                                F32(Self->Block, At + 80U)};
        Result.Bounds[Lane] = {Minimum, Maximum};
        Result.Data[Lane] = U32(Self->Block, At + 96U);
        if (vis::IsFinite(Minimum) && vis::IsFinite(Maximum) &&
            Minimum.X <= Maximum.X && Minimum.Y <= Maximum.Y &&
            Minimum.Z <= Maximum.Z)
        {
          Result.ValidMask |= static_cast<std::uint8_t>(1U << Lane);
        }
      }
      *Output = Result;
      return true;
    }
    catch (...)
    {
      return false;
    }
  }

  static bool LoadBlock(void *Context, std::uint32_t BlockIndex,
                        vis::Bv4QuantizedBlock *Output) noexcept
  {
    if (Output == nullptr)
    {
      return false;
    }
    *Output = {};
    try
    {
      const auto *Self = static_cast<const State *>(Context);
      if (Self == nullptr || BlockIndex >= Self->NodeCount)
      {
        return false;
      }
      const std::uint64_t Offset =
          Self->NodeOffset + static_cast<std::uint64_t>(BlockIndex) * 64U;
      if (!Range(Self->Block, Offset, 64U))
      {
        return false;
      }
      vis::Bv4QuantizedBlock Result{};
      std::array<vis::Bv4QuantizedAxis *, 3> Axes{&Result.X, &Result.Y,
                                                  &Result.Z};
      const std::size_t Base = static_cast<std::size_t>(Offset);
      for (std::size_t Axis = 0; Axis < Axes.size(); ++Axis)
      {
        for (std::size_t Lane = 0; Lane < 4U; ++Lane)
        {
          const std::uint8_t *At =
              Self->Block.Data() + Base + Axis * 16U + Lane * 4U;
          Axes[Axis]->Axis[Lane] = {I16(At), I16(At + 2U)};
        }
      }
      for (std::size_t Lane = 0; Lane < 4U; ++Lane)
      {
        Result.Data[Lane] = U32(Self->Block, Base + 48U + Lane * 4U);
      }
      *Output = Result;
      return true;
    }
    catch (...)
    {
      return false;
    }
  }

  static bool LoadCell(void *Context, std::uint32_t Row, std::uint32_t Column,
                       std::array<vis::HeightFieldSample, 4> *Output) noexcept
  {
    if (Output == nullptr)
    {
      return false;
    }
    *Output = {};
    try
    {
      auto *Self = static_cast<State *>(Context);
      if (Self == nullptr || Self->Package == nullptr || Self->Rows < 2U ||
          Self->Columns < 2U || Row >= Self->Rows - 1U ||
          Column >= Self->Columns - 1U)
      {
        return false;
      }
      // 每个单元最多固定四个租约，调用结束后由包的 LRU 决定保留或回收。
      std::array<BlockView, 4> Tiles{};
      std::array<std::uint32_t, 4> TileIds{};
      std::size_t TileCount = 0;
      std::array<vis::HeightFieldSample, 4> Result{};
      for (std::size_t Corner = 0; Corner < Result.size(); ++Corner)
      {
        const std::uint32_t SampleRow = Row + (Corner >= 2U ? 1U : 0U);
        const std::uint32_t SampleColumn =
            Column + static_cast<std::uint32_t>(Corner % 2U);
        const std::uint32_t TileRow = SampleRow / KPackTerrainTileSide;
        const std::uint32_t TileColumn =
            SampleColumn / KPackTerrainTileSide;
        const std::uint64_t TileOrdinal =
            static_cast<std::uint64_t>(TileRow) * Self->TileColumns +
            TileColumn;
        const std::uint64_t DirectoryAt = Self->TileOffset + TileOrdinal * 4U;
        if (!Range(Self->Block, DirectoryAt, 4U))
        {
          return false;
        }
        const std::uint32_t TileId =
            U32(Self->Block, static_cast<std::size_t>(DirectoryAt));
        std::size_t Lease = 0;
        while (Lease < TileCount && TileIds[Lease] != TileId)
        {
          ++Lease;
        }
        if (Lease == TileCount)
        {
          if (TileCount == Tiles.size() ||
              !Self->AcquireBlock(TileId, &Tiles[Lease]))
          {
            return false;
          }
          TileIds[Lease] = TileId;
          ++TileCount;
        }
        const std::uint32_t TileHeight =
            std::min(KPackTerrainTileSide,
                     Self->Rows - TileRow * KPackTerrainTileSide);
        const std::uint32_t TileWidth =
            std::min(KPackTerrainTileSide,
                     Self->Columns - TileColumn * KPackTerrainTileSide);
        const std::uint64_t TileBytes =
            static_cast<std::uint64_t>(TileHeight) * TileWidth * 4U;
        const std::uint64_t SampleAt =
            (static_cast<std::uint64_t>(SampleRow % KPackTerrainTileSide) *
                 TileWidth +
             SampleColumn % KPackTerrainTileSide) *
            4U;
        if (Tiles[Lease].Size() != TileBytes ||
            !Range(Tiles[Lease], SampleAt, 4U))
        {
          return false;
        }
        const std::uint8_t *At =
            Tiles[Lease].Data() + static_cast<std::size_t>(SampleAt);
        Result[Corner] = {I16(At), At[2], At[3]};
      }
      *Output = Result;
      return true;
    }
    catch (...)
    {
      return false;
    }
  }

  bool BindTriangle(vis::GeometryView *Output)
  {
    if (!Range(Block, 0U, KPackTriangleHeaderBytes) ||
        U32(Block, 0U) != KPackTriangleMagic)
    {
      return false;
    }
    const std::uint32_t TreeKind = U32(Block, 4U);
    VertexCount = U32(Block, 8U);
    TriangleCount = U32(Block, 12U);
    NodeCount = U32(Block, 16U);
    const std::uint32_t Root = U32(Block, 20U);
    IndexBytes = U32(Block, 24U);
    const float Epsilon = F32(Block, 28U);
    VertexOffset = U64(Block, 32U);
    IndexOffset = U64(Block, 40U);
    NodeOffset = U64(Block, 48U);
    if (TreeKind > 1U || (IndexBytes != 2U && IndexBytes != 4U) ||
        !std::isfinite(Epsilon) || Epsilon < 0.0f ||
        VertexOffset < KPackTriangleHeaderBytes ||
        IndexOffset < KPackTriangleHeaderBytes ||
        NodeOffset < KPackTriangleHeaderBytes ||
        !Range(Block, VertexOffset, VertexCount, 12U) ||
        !Range(Block, IndexOffset, TriangleCount, IndexBytes * 3U) ||
        !Range(Block, NodeOffset, NodeCount, TreeKind == 0U ? 112U : 64U))
    {
      return false;
    }
    auto &Geometry = Output->TriangleMesh;
    Geometry.Triangles = {};
    Geometry.Triangles.Vertices.Count = VertexCount;
    Geometry.Triangles.TriangleCount = TriangleCount;
    Geometry.Triangles.IndexWidth = IndexBytes == 2U
                                        ? vis::TriangleIndexWidth::Index16
                                        : vis::TriangleIndexWidth::Index32;
    Geometry.Triangles.Epsilon = Epsilon;
    Geometry.Triangles.ProviderContext = this;
    Geometry.Triangles.LoadIndices = &LoadIndices;
    Geometry.Triangles.LoadVertex = &LoadVertex;
    Geometry.RTree = {};
    Geometry.Bv4 = {};
    if (TreeKind == 0U)
    {
      if (Root > NodeCount)
      {
        return false;
      }
      Geometry.TreeType = vis::TriangleTreeType::RTree;
      Geometry.RTree.Pages.Count = NodeCount;
      Geometry.RTree.RootPageCount = Root;
      Geometry.RTree.ProviderContext = this;
      Geometry.RTree.LoadPage = &LoadPage;
    }
    else
    {
      Geometry.TreeType = vis::TriangleTreeType::Bv4;
      Geometry.Bv4.Blocks.Count = NodeCount;
      Geometry.Bv4.InitData = Root;
      Geometry.Bv4.CenterCoefficient = Vector(Block, 56U);
      Geometry.Bv4.ExtentCoefficient = Vector(Block, 68U);
      if (!vis::IsFinite(Geometry.Bv4.CenterCoefficient) ||
          !vis::IsFinite(Geometry.Bv4.ExtentCoefficient))
      {
        return false;
      }
      Geometry.Bv4.ProviderContext = this;
      Geometry.Bv4.LoadBlock = &LoadBlock;
    }
    return true;
  }

  bool BindConvex(vis::GeometryView *Output)
  {
    if (!Range(Block, 0U, KPackConvexHeaderBytes) ||
        U32(Block, 0U) != KPackConvexMagic)
    {
      return false;
    }
    const std::uint32_t Count = U32(Block, 4U);
    const std::uint64_t Offset = U64(Block, 8U);
    if (Count == 0U || Offset < KPackConvexHeaderBytes ||
        !Range(Block, Offset, Count, 16U))
    {
      return false;
    }
    Planes.resize(Count);
    for (std::size_t Index = 0; Index < Count; ++Index)
    {
      const std::size_t At = static_cast<std::size_t>(Offset) + Index * 16U;
      const vis::Vec3 Normal = Vector(Block, At);
      const float Distance = F32(Block, At + 12U);
      if (!vis::IsFinite(Normal) || !std::isfinite(Distance))
      {
        return false;
      }
      Planes[Index] = {Normal, Distance};
    }
    Output->Convex.Planes = {Planes.data(), Planes.size()};
    return true;
  }

  bool BindHeightField(vis::GeometryView *Output)
  {
    if (!Range(Block, 0U, KPackHeightFieldHeaderBytes) ||
        U32(Block, 0U) != KPackHeightFieldMagic)
    {
      return false;
    }
    Rows = U32(Block, 4U);
    Columns = U32(Block, 8U);
    const std::int32_t MinimumHeight = I32(Block, 16U);
    const std::int32_t MaximumHeight = I32(Block, 20U);
    TileRows = U32(Block, 24U);
    TileColumns = U32(Block, 28U);
    TileOffset = U64(Block, 32U);
    if (Rows < 2U || Columns < 2U ||
        U32(Block, 12U) != KPackTerrainTileSide ||
        MinimumHeight < std::numeric_limits<std::int16_t>::min() ||
        MaximumHeight > std::numeric_limits<std::int16_t>::max() ||
        MinimumHeight > MaximumHeight ||
        TileRows != 1U + (Rows - 1U) / KPackTerrainTileSide ||
        TileColumns != 1U + (Columns - 1U) / KPackTerrainTileSide ||
        TileOffset < KPackHeightFieldHeaderBytes ||
        !Range(Block, TileOffset,
               static_cast<std::uint64_t>(TileRows) * TileColumns, 4U))
    {
      return false;
    }
    auto &Geometry = Output->HeightField;
    const float RowEnd = static_cast<float>(Rows - 1U) * Geometry.RowScale;
    const float ColumnEnd =
        static_cast<float>(Columns - 1U) * Geometry.ColumnScale;
    const float Height0 =
        static_cast<float>(MinimumHeight) * Geometry.HeightScale;
    const float Height1 =
        static_cast<float>(MaximumHeight) * Geometry.HeightScale;
    if (!std::isfinite(RowEnd) || !std::isfinite(ColumnEnd) ||
        !std::isfinite(Height0) || !std::isfinite(Height1))
    {
      return false;
    }
    Geometry.RowCount = Rows;
    Geometry.ColumnCount = Columns;
    Geometry.SampleColumnStride = Columns;
    Geometry.SampleStride = 4U;
    Geometry.ConvexEdgeThreshold = 4.0f;
    Geometry.DataFlags = 0U;
    Geometry.Samples = {};
    Geometry.ProviderContext = this;
    Geometry.LoadCell = &LoadCell;
    Geometry.NbColumns = static_cast<float>(Columns);
    Geometry.DataFormat = vis::KVisibilityHeightFieldFormatS16Tm;
    Geometry.LocalBounds = {{std::min(0.0f, RowEnd), std::min(Height0, Height1),
                             std::min(0.0f, ColumnEnd)},
                            {std::max(0.0f, RowEnd), std::max(Height0, Height1),
                             std::max(0.0f, ColumnEnd)}};
    Geometry.LocalBoundsDefined = true;
    return true;
  }
};

GeometryReader::GeometryReader() noexcept = default;
GeometryReader::~GeometryReader() noexcept = default;
GeometryReader::GeometryReader(
    GeometryReader &&) noexcept = default;
GeometryReader &GeometryReader::operator=(
    GeometryReader &&) noexcept = default;

bool GeometryReader::Bind(PackReader &Package,
                                    std::uint32_t GeometrySlot,
                                    visibility::GeometryType Requested,
                                    visibility::GeometryView *Output,
                                    void *ContinuationContext,
                                    bool (*Continue)(void *)) noexcept
{
  State_.reset();
  if (Output == nullptr)
  {
    return false;
  }
  visibility::GeometryView Result = *Output;
  *Output = {};
  try
  {
    if (Requested != vis::GeometryType::TriangleMesh &&
        Requested != vis::GeometryType::ConvexMesh &&
        Requested != vis::GeometryType::HeightField)
    {
      return false;
    }
    auto Data = std::make_unique<State>();
    Data->Package = &Package;
    Data->ContinuationContext = ContinuationContext;
    Data->Continuation = Continue;
    if (!Data->AcquireBlock(GeometrySlot, &Data->Block))
    {
      return false;
    }
    Result.Type = Requested;
    bool Defined = false;
    if (Requested == vis::GeometryType::HeightField)
    {
      Defined = Data->BindHeightField(&Result);
    }
    else
    {
      if (!Range(Data->Block, 0U, KPackMeshRootBytes) ||
          U32(Data->Block, 0U) != KPackMeshRootMagic)
      {
        return false;
      }
      const std::uint32_t BlockId = U32(
          Data->Block, Requested == vis::GeometryType::TriangleMesh ? 4U : 8U);
      Data->Block = {};
      if (BlockId == PackReader::KInvalidGeometrySlot ||
          !Data->AcquireBlock(BlockId, &Data->Block))
      {
        return false;
      }
      Defined = Requested == vis::GeometryType::TriangleMesh
                    ? Data->BindTriangle(&Result)
                    : Data->BindConvex(&Result);
    }
    if (!Defined)
    {
      return false;
    }
    State_ = std::move(Data);
    *Output = Result;
    return true;
  }
  catch (...)
  {
    return false;
  }
}

} // namespace detail
} // namespace physx_pack
