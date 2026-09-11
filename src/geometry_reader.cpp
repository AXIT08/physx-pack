#include "detail/geometry_reader.h"
#include "detail/geometry_format.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstring>
#include <limits>
#include <mutex>
#include <new>
#include <unordered_map>
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
  const auto *Bytes = Block.Data() + Offset;
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
  const auto Bits = U32(Block, Offset);
  std::int32_t Value;
  std::memcpy(&Value, &Bits, sizeof(Value));
  return Value;
}

float F32(const BlockView &Block, std::size_t Offset) noexcept
{
  const auto Bits = U32(Block, Offset);
  float Value;
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
  std::int16_t Value;
  std::memcpy(&Value, &Bits, sizeof(Value));
  return Value;
}

struct GeometryMetadata
{
  std::uint32_t BlockId = 0;
  std::uint64_t BlockBytes = 0;
  vis::GeometryView View{};
  GeometryDescription Description{};
  std::uint32_t VertexCount = 0;
  std::uint32_t TriangleCount = 0;
  std::uint32_t NodeCount = 0;
  std::uint32_t IndexBytes = 0;
  std::uint64_t VertexOffset = 0;
  std::uint64_t IndexOffset = 0;
  std::uint64_t NodeOffset = 0;
  std::uint32_t Rows = 0;
  std::uint32_t Columns = 0;
  std::uint32_t TileColumns = 0;
  std::uint64_t TileOffset = 0;
  std::vector<vis::ConvexPlane> Planes;

  Error ParseTriangle(const BlockView &Block)
  {
    if (!Range(Block, 0, KPackTriangleHeaderBytes) || U32(Block, 0) != KPackTriangleMagic)
      return Error::CorruptData;
    const auto TreeKind = U32(Block, 4);
    VertexCount = U32(Block, 8);
    TriangleCount = U32(Block, 12);
    NodeCount = U32(Block, 16);
    const auto Root = U32(Block, 20);
    IndexBytes = U32(Block, 24);
    const float Epsilon = F32(Block, 28);
    VertexOffset = U64(Block, 32);
    IndexOffset = U64(Block, 40);
    NodeOffset = U64(Block, 48);
    if (TreeKind > 1U || (IndexBytes != 2U && IndexBytes != 4U) ||
        !std::isfinite(Epsilon) || Epsilon < 0.0f ||
        VertexOffset < KPackTriangleHeaderBytes || IndexOffset < KPackTriangleHeaderBytes ||
        NodeOffset < KPackTriangleHeaderBytes ||
        !Range(Block, VertexOffset, VertexCount, 12U) ||
        !Range(Block, IndexOffset, TriangleCount, IndexBytes * 3U) ||
        !Range(Block, NodeOffset, NodeCount, TreeKind == 0U ? 112U : 64U))
      return Error::CorruptData;
    for (std::uint64_t Index = 0; Index < static_cast<std::uint64_t>(TriangleCount) * 3U; ++Index)
    {
      const auto At = static_cast<std::size_t>(IndexOffset + Index * IndexBytes);
      const auto Value = IndexBytes == 4U ? U32(Block, At)
          : static_cast<std::uint32_t>(Block.Data()[At] | (Block.Data()[At + 1U] << 8U));
      if (Value >= VertexCount) return Error::CorruptData;
    }
    auto &Mesh = View.TriangleMesh;
    Mesh.Triangles.Vertices.Count = VertexCount;
    Mesh.Triangles.TriangleCount = TriangleCount;
    Mesh.Triangles.IndexWidth = IndexBytes == 2U ? vis::TriangleIndexWidth::Index16
                                                : vis::TriangleIndexWidth::Index32;
    Mesh.Triangles.Epsilon = Epsilon;
    if (TreeKind == 0U)
    {
      if (Root > NodeCount) return Error::CorruptData;
      Mesh.TreeType = vis::TriangleTreeType::RTree;
      Mesh.RTree.Pages.Count = NodeCount;
      Mesh.RTree.RootPageCount = Root;
    }
    else
    {
      Mesh.TreeType = vis::TriangleTreeType::Bv4;
      Mesh.Bv4.Blocks.Count = NodeCount;
      Mesh.Bv4.InitData = Root;
      Mesh.Bv4.CenterCoefficient = Vector(Block, 56);
      Mesh.Bv4.ExtentCoefficient = Vector(Block, 68);
      if (!vis::IsFinite(Mesh.Bv4.CenterCoefficient) || !vis::IsFinite(Mesh.Bv4.ExtentCoefficient))
        return Error::CorruptData;
    }
    // Compute reliable asset bounds once, independent of instance transforms.
    for (std::uint32_t Index = 0; Index < VertexCount; ++Index)
    {
      const auto Position = Vector(Block, static_cast<std::size_t>(VertexOffset) + Index * 12ULL);
      if (!vis::IsFinite(Position)) return Error::CorruptData;
      auto &Bounds = Description.LocalBounds;
      if (Index == 0) Bounds = {Position, Position};
      else
      {
        Bounds.Minimum = {std::min(Bounds.Minimum.X, Position.X), std::min(Bounds.Minimum.Y, Position.Y),
                          std::min(Bounds.Minimum.Z, Position.Z)};
        Bounds.Maximum = {std::max(Bounds.Maximum.X, Position.X), std::max(Bounds.Maximum.Y, Position.Y),
                          std::max(Bounds.Maximum.Z, Position.Z)};
      }
    }
    Description.BoundsDefined = VertexCount != 0;
    if (Description.BoundsDefined)
    {
      // Triangle tests permit a barycentric epsilon beyond their edges. A scene
      // broad phase must retain those hits, including float rounding at bounds.
      auto Expand = [Epsilon](float *Minimum, float *Maximum) {
        const double Extent = static_cast<double>(*Maximum) - *Minimum;
        const double Padding = 2.0 * Epsilon * Extent;
        const double Lower = static_cast<double>(*Minimum) - Padding;
        const double Upper = static_cast<double>(*Maximum) + Padding;
        const double Limit = std::numeric_limits<float>::max();
        if (Lower < -Limit || Upper > Limit) return false;
        const float Negative = -std::numeric_limits<float>::infinity();
        const float Positive = std::numeric_limits<float>::infinity();
        const float Lo = std::nextafter(static_cast<float>(Lower), Negative);
        const float Hi = std::nextafter(static_cast<float>(Upper), Positive);
        if (!std::isfinite(Lo) || !std::isfinite(Hi)) return false;
        *Minimum = Lo;
        *Maximum = Hi;
        return true;
      };
      auto &Bounds = Description.LocalBounds;
      Description.BoundsDefined = Expand(&Bounds.Minimum.X, &Bounds.Maximum.X) &&
                                  Expand(&Bounds.Minimum.Y, &Bounds.Maximum.Y) &&
                                  Expand(&Bounds.Minimum.Z, &Bounds.Maximum.Z);
    }
    return Error::None;
  }

  Error ParseConvex(const BlockView &Block)
  {
    if (!Range(Block, 0, KPackConvexHeaderBytes) || U32(Block, 0) != KPackConvexMagic)
      return Error::CorruptData;
    const auto Count = U32(Block, 4);
    const auto Offset = U64(Block, 8);
    if (!Count || Offset < KPackConvexHeaderBytes || !Range(Block, Offset, Count, 16U))
      return Error::CorruptData;
    Planes.resize(Count);
    for (std::size_t Index = 0; Index < Count; ++Index)
    {
      const auto At = static_cast<std::size_t>(Offset) + Index * 16U;
      const auto Normal = Vector(Block, At);
      const float Distance = F32(Block, At + 12U);
      if (!vis::IsFinite(Normal) || !std::isfinite(Distance) ||
          (Normal.X == 0.0f && Normal.Y == 0.0f && Normal.Z == 0.0f))
        return Error::CorruptData;
      Planes[Index] = {Normal, Distance};
    }
    View.Convex.Planes = {Planes.data(), Planes.size()};
    // Plane intersections are not needed for query correctness; no guessed AABB.
    Description.BoundsDefined = false;
    return Error::None;
  }

  Error ParseHeightField(const BlockView &Block)
  {
    if (!Range(Block, 0, KPackHeightFieldHeaderBytes) || U32(Block, 0) != KPackHeightFieldMagic)
      return Error::CorruptData;
    Rows = U32(Block, 4);
    Columns = U32(Block, 8);
    const auto Minimum = I32(Block, 16);
    const auto Maximum = I32(Block, 20);
    const auto TileRows = U32(Block, 24);
    TileColumns = U32(Block, 28);
    TileOffset = U64(Block, 32);
    if (Rows < 2U || Columns < 2U || U32(Block, 12) != KPackTerrainTileSide ||
        Minimum < std::numeric_limits<std::int16_t>::min() ||
        Maximum > std::numeric_limits<std::int16_t>::max() || Minimum > Maximum ||
        TileRows != 1U + (Rows - 1U) / KPackTerrainTileSide ||
        TileColumns != 1U + (Columns - 1U) / KPackTerrainTileSide ||
        TileOffset < KPackHeightFieldHeaderBytes ||
        !Range(Block, TileOffset, static_cast<std::uint64_t>(TileRows) * TileColumns, 4U))
      return Error::CorruptData;
    auto &Height = View.HeightField;
    Height.RowCount = Rows;
    Height.ColumnCount = Columns;
    Height.SampleColumnStride = Columns;
    Height.SampleStride = 4;
    Height.ConvexEdgeThreshold = 4.0f;
    Height.NbColumns = static_cast<float>(Columns);
    Height.DataFormat = vis::KVisibilityHeightFieldFormatS16Tm;
    Description.LocalBounds = {{0.0f, static_cast<float>(Minimum), 0.0f},
                               {static_cast<float>(Rows - 1U), static_cast<float>(Maximum),
                                static_cast<float>(Columns - 1U)}};
    Description.BoundsDefined = true;
    Height.LocalBounds = Description.LocalBounds;
    Height.LocalBoundsDefined = true;
    return Error::None;
  }
};

} // namespace

struct GeometryReader::State
{
  PackReader *Package = nullptr;
  std::shared_ptr<const GeometryMetadata> Metadata;
  BlockView Block;
  Error Failure = Error::None;

  bool Fail(Error Reason = Error::CorruptData) noexcept
  {
    if (Failure == Error::None) Failure = Reason;
    return false;
  }

  bool Acquire(std::uint32_t Id, BlockView *Output) noexcept
  {
    Error Reason = Error::None;
    if (Package == nullptr) return Fail(Error::NotOpen);
    return Package->AcquireBlock(Id, Output, &Reason) ||
           Fail(Reason == Error::None ? Error::CorruptData : Reason);
  }

  static bool LoadIndices(void *Context, std::uint32_t TriangleIndex,
                          std::array<std::uint32_t, 3> *Output) noexcept
  {
    if (Output) *Output = {};
    auto *Self = static_cast<State *>(Context);
    if (!Self) return false;
    if (!Output || !Self->Metadata) return Self->Fail();
    *Output = {};
    const auto &Data = *Self->Metadata;
    if (TriangleIndex >= Data.TriangleCount) return Self->Fail();
    const auto Offset = Data.IndexOffset + static_cast<std::uint64_t>(TriangleIndex) * 3U * Data.IndexBytes;
    if (!Range(Self->Block, Offset, 3U, Data.IndexBytes)) return Self->Fail();
    for (std::size_t Index = 0; Index < 3; ++Index)
    {
      const auto At = static_cast<std::size_t>(Offset + Index * Data.IndexBytes);
      const auto Value = Data.IndexBytes == 4U ? U32(Self->Block, At)
          : static_cast<std::uint32_t>(Self->Block.Data()[At] | (Self->Block.Data()[At + 1U] << 8U));
      if (Value >= Data.VertexCount) { *Output = {}; return Self->Fail(); }
      (*Output)[Index] = Value;
    }
    return true;
  }

  static bool LoadVertex(void *Context, std::uint32_t VertexIndex, vis::Vec3 *Output) noexcept
  {
    if (Output) *Output = {};
    auto *Self = static_cast<State *>(Context);
    if (!Self) return false;
    if (!Output || !Self->Metadata) return Self->Fail();
    *Output = {};
    const auto &Data = *Self->Metadata;
    const auto At = Data.VertexOffset + static_cast<std::uint64_t>(VertexIndex) * 12U;
    if (VertexIndex >= Data.VertexCount || !Range(Self->Block, At, 12U)) return Self->Fail();
    const auto Value = Vector(Self->Block, static_cast<std::size_t>(At));
    if (!vis::IsFinite(Value)) return Self->Fail();
    *Output = Value;
    return true;
  }

  static bool LoadPage(void *Context, std::uint32_t PageIndex, vis::RTreePage *Output) noexcept
  {
    if (Output) *Output = {};
    auto *Self = static_cast<State *>(Context);
    if (!Self) return false;
    if (!Output || !Self->Metadata) return Self->Fail();
    *Output = {};
    const auto &Data = *Self->Metadata;
    const auto Offset = Data.NodeOffset + static_cast<std::uint64_t>(PageIndex) * 112U;
    if (PageIndex >= Data.NodeCount || !Range(Self->Block, Offset, 112U)) return Self->Fail();
    const auto Base = static_cast<std::size_t>(Offset);
    for (std::size_t Lane = 0; Lane < 4; ++Lane)
    {
      const auto At = Base + Lane * 4U;
      const vis::Vec3 Minimum{F32(Self->Block, At), F32(Self->Block, At + 16U), F32(Self->Block, At + 32U)};
      const vis::Vec3 Maximum{F32(Self->Block, At + 48U), F32(Self->Block, At + 64U), F32(Self->Block, At + 80U)};
      Output->Bounds[Lane] = {Minimum, Maximum};
      Output->Data[Lane] = U32(Self->Block, At + 96U);
      if (vis::IsFinite(Minimum) && vis::IsFinite(Maximum) &&
          Minimum.X <= Maximum.X && Minimum.Y <= Maximum.Y && Minimum.Z <= Maximum.Z)
        Output->ValidMask |= static_cast<std::uint8_t>(1U << Lane);
    }
    return true;
  }

  static bool LoadBlock(void *Context, std::uint32_t BlockIndex, vis::Bv4QuantizedBlock *Output) noexcept
  {
    if (Output) *Output = {};
    auto *Self = static_cast<State *>(Context);
    if (!Self) return false;
    if (!Output || !Self->Metadata) return Self->Fail();
    *Output = {};
    const auto &Data = *Self->Metadata;
    const auto Offset = Data.NodeOffset + static_cast<std::uint64_t>(BlockIndex) * 64U;
    if (BlockIndex >= Data.NodeCount || !Range(Self->Block, Offset, 64U)) return Self->Fail();
    std::array<vis::Bv4QuantizedAxis *, 3> Axes{&Output->X, &Output->Y, &Output->Z};
    const auto Base = static_cast<std::size_t>(Offset);
    for (std::size_t Axis = 0; Axis < Axes.size(); ++Axis)
      for (std::size_t Lane = 0; Lane < 4; ++Lane)
      {
        const auto *At = Self->Block.Data() + Base + Axis * 16U + Lane * 4U;
        Axes[Axis]->Axis[Lane] = {I16(At), I16(At + 2)};
      }
    for (std::size_t Lane = 0; Lane < 4; ++Lane)
      Output->Data[Lane] = U32(Self->Block, Base + 48U + Lane * 4U);
    return true;
  }

  static bool LoadCell(void *Context, std::uint32_t Row, std::uint32_t Column,
                       std::array<vis::HeightFieldSample, 4> *Output) noexcept
  {
    if (Output) *Output = {};
    auto *Self = static_cast<State *>(Context);
    if (!Self) return false;
    if (!Output || !Self->Metadata) return Self->Fail();
    *Output = {};
    const auto &Data = *Self->Metadata;
    if (Data.Rows < 2 || Data.Columns < 2 || Row >= Data.Rows - 1 || Column >= Data.Columns - 1)
      return Self->Fail();
    // A cell can cross at most four tile boundaries. Pin only those local leases.
    std::array<BlockView, 4> Tiles{};
    std::array<std::uint32_t, 4> TileIds{};
    std::array<vis::HeightFieldSample, 4> Samples{};
    std::size_t TileCount = 0;
    for (std::size_t Corner = 0; Corner < 4; ++Corner)
    {
      const auto SampleRow = Row + (Corner >= 2 ? 1U : 0U);
      const auto SampleColumn = Column + static_cast<std::uint32_t>(Corner % 2);
      const auto TileRow = SampleRow / KPackTerrainTileSide;
      const auto TileColumn = SampleColumn / KPackTerrainTileSide;
      const auto Ordinal = static_cast<std::uint64_t>(TileRow) * Data.TileColumns + TileColumn;
      const auto DirectoryAt = Data.TileOffset + Ordinal * 4U;
      if (!Range(Self->Block, DirectoryAt, 4U)) return Self->Fail();
      const auto TileId = U32(Self->Block, static_cast<std::size_t>(DirectoryAt));
      std::size_t Lease = 0;
      while (Lease < TileCount && TileIds[Lease] != TileId) ++Lease;
      if (Lease == TileCount)
      {
        if (TileCount == Tiles.size() || !Self->Acquire(TileId, &Tiles[Lease]))
        { *Output = {}; return false; }
        TileIds[Lease] = TileId;
        ++TileCount;
      }
      const auto Height = std::min(KPackTerrainTileSide, Data.Rows - TileRow * KPackTerrainTileSide);
      const auto Width = std::min(KPackTerrainTileSide, Data.Columns - TileColumn * KPackTerrainTileSide);
      const auto TileBytes = static_cast<std::uint64_t>(Height) * Width * 4U;
      const auto At = (static_cast<std::uint64_t>(SampleRow % KPackTerrainTileSide) * Width +
                       SampleColumn % KPackTerrainTileSide) * 4U;
      if (Tiles[Lease].Size() != TileBytes || !Range(Tiles[Lease], At, 4U))
      { *Output = {}; return Self->Fail(); }
      const auto *Bytes = Tiles[Lease].Data() + static_cast<std::size_t>(At);
      Samples[Corner] = {I16(Bytes), Bytes[2], Bytes[3]};
    }
    *Output = Samples;
    return true;
  }
};

struct GeometryStore::State
{
  explicit State(PackReader &Value) noexcept : Package(Value) {}
  PackReader &Package;
  std::mutex Mutex;
  std::unordered_map<std::uint64_t, std::shared_ptr<const GeometryMetadata>> Metadata;
  std::atomic<std::uint64_t> Bytes{0};

  Error Get(std::uint32_t Slot, vis::GeometryType Requested,
            std::shared_ptr<const GeometryMetadata> *Output)
  {
    if (Requested != vis::GeometryType::TriangleMesh && Requested != vis::GeometryType::ConvexMesh &&
        Requested != vis::GeometryType::HeightField) return Error::UnsupportedGeometry;
    const auto Key = (static_cast<std::uint64_t>(Slot) << 8U) | static_cast<std::uint8_t>(Requested);
    std::lock_guard<std::mutex> Lock(Mutex);
    if (!Package.IsOpen()) return Error::NotOpen;
    const auto Found = Metadata.find(Key);
    if (Found != Metadata.end()) { *Output = Found->second; return Error::None; }
    Error Reason = Error::None;
    BlockView Block;
    if (!Package.AcquireBlock(Slot, &Block, &Reason))
      return Reason == Error::None ? Error::CorruptData : Reason;
    GeometryKind Kind = GeometryKind::Unknown;
    if (!Package.GetGeometryKind(Slot, &Kind)) return Error::UnsupportedGeometry;
    const bool Mesh = Kind == GeometryKind::TriangleMesh || Kind == GeometryKind::ConvexMesh;
    if ((Requested == vis::GeometryType::HeightField && Kind != GeometryKind::HeightField) ||
        (Requested != vis::GeometryType::HeightField && !Mesh)) return Error::UnsupportedGeometry;
    std::uint32_t BlockId = Slot;
    if (Mesh)
    {
      if (!Range(Block, 0, KPackMeshRootBytes) || U32(Block, 0) != KPackMeshRootMagic)
        return Error::CorruptData;
      BlockId = U32(Block, Requested == vis::GeometryType::TriangleMesh ? 4U : 8U);
      if (BlockId == PackReader::KInvalidGeometrySlot) return Error::MissingVariant;
      Block = {};
      if (!Package.AcquireBlock(BlockId, &Block, &Reason))
        return Reason == Error::None ? Error::CorruptData : Reason;
    }
    auto Entry = std::make_shared<GeometryMetadata>();
    Entry->BlockId = BlockId;
    Entry->BlockBytes = Block.Size();
    Entry->View.Type = Requested;
    if (Requested == vis::GeometryType::TriangleMesh) Reason = Entry->ParseTriangle(Block);
    else if (Requested == vis::GeometryType::ConvexMesh) Reason = Entry->ParseConvex(Block);
    else Reason = Entry->ParseHeightField(Block);
    if (Reason != Error::None) return Reason;
    const auto Cost = sizeof(GeometryMetadata) + Entry->Planes.capacity() * sizeof(vis::ConvexPlane) +
                      sizeof(decltype(Metadata)::value_type);
    Metadata.emplace(Key, Entry);
    Bytes.fetch_add(Cost, std::memory_order_relaxed);
    *Output = std::move(Entry);
    return Error::None;
  }
};

GeometryReader::GeometryReader() noexcept = default;
GeometryReader::~GeometryReader() noexcept = default;
GeometryReader::GeometryReader(GeometryReader &&) noexcept = default;
GeometryReader &GeometryReader::operator=(GeometryReader &&) noexcept = default;

void GeometryReader::Reset() noexcept
{
  if (!State_) return;
  State_->Block = {};
  State_->Metadata.reset();
  State_->Package = nullptr;
  State_->Failure = Error::None;
}

Error GeometryReader::LastError() const noexcept
{
  return State_ ? State_->Failure : Error::None;
}

GeometryStore::GeometryStore(PackReader &Package) noexcept : State_(new (std::nothrow) State(Package)) {}
GeometryStore::~GeometryStore() noexcept = default;

Error GeometryStore::Describe(std::uint32_t Slot, vis::GeometryType Requested,
                              GeometryDescription *Output) noexcept
{
  if (!Output) return Error::InvalidArgument;
  *Output = {};
  if (!State_) return Error::OutOfMemory;
  try
  {
    std::shared_ptr<const GeometryMetadata> Metadata;
    const auto Result = State_->Get(Slot, Requested, &Metadata);
    if (Result == Error::None) *Output = Metadata->Description;
    return Result;
  }
  catch (const std::bad_alloc &) { return Error::OutOfMemory; }
  catch (...) { return Error::CorruptData; }
}

Error GeometryStore::Bind(std::uint32_t Slot, vis::GeometryType Requested,
                          GeometryReader &Lease, vis::GeometryView *Output) noexcept
{
  Lease.Reset();
  if (!Output) return Error::InvalidArgument;
  *Output = {};
  if (!State_) return Error::OutOfMemory;
  if (!Lease.State_) Lease.State_.reset(new (std::nothrow) GeometryReader::State());
  if (!Lease.State_) return Error::OutOfMemory;
  auto &Data = *Lease.State_;
  try
  {
    const auto Result = State_->Get(Slot, Requested, &Data.Metadata);
    if (Result != Error::None) { Data.Failure = Result; return Result; }
    Data.Package = &State_->Package;
    if (!Data.Acquire(Data.Metadata->BlockId, &Data.Block)) return Data.Failure;
    if (Data.Block.Size() != Data.Metadata->BlockBytes)
    { Data.Fail(); return Data.Failure; }
    *Output = Data.Metadata->View;
    if (Requested == vis::GeometryType::TriangleMesh)
    {
      auto &Mesh = Output->TriangleMesh;
      Mesh.Triangles.ProviderContext = &Data;
      Mesh.Triangles.LoadIndices = &GeometryReader::State::LoadIndices;
      Mesh.Triangles.LoadVertex = &GeometryReader::State::LoadVertex;
      Mesh.RTree.ProviderContext = &Data;
      Mesh.RTree.LoadPage = &GeometryReader::State::LoadPage;
      Mesh.Bv4.ProviderContext = &Data;
      Mesh.Bv4.LoadBlock = &GeometryReader::State::LoadBlock;
    }
    else if (Requested == vis::GeometryType::HeightField)
    {
      Output->HeightField.ProviderContext = &Data;
      Output->HeightField.LoadCell = &GeometryReader::State::LoadCell;
    }
    return Error::None;
  }
  catch (const std::bad_alloc &) { Data.Failure = Error::OutOfMemory; }
  catch (...) { Data.Failure = Error::CorruptData; }
  Data.Block = {};
  Data.Metadata.reset();
  return Data.Failure;
}

std::uint64_t GeometryStore::MetadataBytes() const noexcept
{
  return State_ ? State_->Bytes.load(std::memory_order_relaxed) : 0U;
}

} // namespace detail
} // namespace physx_pack
