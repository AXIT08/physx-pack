#include "detail/pack_reader.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cstdlib>
#include <fcntl.h>
#include <limits>
#include <new>
#include <sys/stat.h>
#include <tuple>
#include <unistd.h>
#include <utility>
#include <zlib.h>

namespace physx_pack
{
namespace detail
{
namespace
{
constexpr std::array<unsigned char, 8> KPackMagic{'X', 'C', 'P', 'H', 'Y', 'S', 'X', 0};
constexpr std::size_t KMaximumTableCount = 1U << 20U;
constexpr std::size_t KMaximumPreloadCount = 1U << 22U;
constexpr std::size_t KMaximumStringPoolSize = 8U * 1024U * 1024U;
constexpr std::size_t KReadChunk = 64U * 1024U;
std::uint32_t LoadU32(const unsigned char *Value) noexcept
{
  return static_cast<std::uint32_t>(Value[0]) | (static_cast<std::uint32_t>(Value[1]) << 8U) |
         (static_cast<std::uint32_t>(Value[2]) << 16U) | (static_cast<std::uint32_t>(Value[3]) << 24U);
}
std::uint64_t LoadU64(const unsigned char *Value) noexcept
{
  return LoadU32(Value) | (static_cast<std::uint64_t>(LoadU32(Value + 4)) << 32U);
}
bool AddWithin(std::uint64_t Offset, std::uint64_t Size, std::uint64_t Limit) noexcept
{
  return Offset <= Limit && Size <= Limit - Offset;
}
bool ParseBundleId(std::string_view Text, std::uint64_t &Output) noexcept
{
  Output = 0;
  const std::size_t Begin = Text.size() == 18 && Text[0] == '0' &&
      (Text[1] == 'x' || Text[1] == 'X') ? 2 : 0;
  if (Text.size() - Begin != 16) return false;
  for (std::size_t Index = Begin; Index < Text.size(); ++Index)
  {
    const char Character = Text[Index];
    unsigned Digit = 0;
    if (Character >= '0' && Character <= '9') Digit = Character - '0';
    else if (Character >= 'a' && Character <= 'f') Digit = Character - 'a' + 10;
    else if (Character >= 'A' && Character <= 'F') Digit = Character - 'A' + 10;
    else return false;
    Output = (Output << 4U) | Digit;
  }
  return true;
}
struct BundleKey
{
  std::string_view Text;
  std::uint64_t Number = 0;
  bool Numeric = false;
  bool operator==(const BundleKey &Other) const noexcept
  {
    return Numeric == Other.Numeric && (Numeric ? Number == Other.Number : Text == Other.Text);
  }
};
struct BundleKeyHash
{
  std::size_t operator()(const BundleKey &Key) const noexcept
  {
    return Key.Numeric ? std::hash<std::uint64_t>{}(Key.Number) : std::hash<std::string_view>{}(Key.Text);
  }
};
struct PreloadRow { std::uint32_t Bundle, Ordinal, Slot; };

// Zlib workspace and its input window are temporary memory. Decoded memory is
// separately budgeted, including buffers retained by external block pins.
union AllocationHeader { std::max_align_t Alignment; std::size_t Bytes; };
voidpf ZAllocate(voidpf Opaque, uInt Count, uInt Size) noexcept
{
  if (Size && Count > (std::numeric_limits<std::size_t>::max() - sizeof(AllocationHeader)) / Size)
    return nullptr;
  const std::size_t Bytes = static_cast<std::size_t>(Count) * Size + sizeof(AllocationHeader);
  auto *Header = static_cast<AllocationHeader *>(std::calloc(1, Bytes));
  if (!Header) return nullptr;
  Header->Bytes = Bytes;
  auto &Stats = *static_cast<CacheStats *>(Opaque);
  Stats.TemporaryBytes += Bytes;
  Stats.PeakTemporaryBytes = std::max(Stats.PeakTemporaryBytes, Stats.TemporaryBytes);
  return Header + 1;
}
void ZFree(voidpf Opaque, voidpf Address) noexcept
{
  if (!Address) return;
  auto *Header = static_cast<AllocationHeader *>(Address) - 1;
  static_cast<CacheStats *>(Opaque)->TemporaryBytes -= Header->Bytes;
  std::free(Header);
}
struct TemporaryWindow
{
  CacheStats &Stats;
  explicit TemporaryWindow(CacheStats &Value) : Stats(Value)
  {
    Stats.TemporaryBytes += KReadChunk;
    Stats.PeakTemporaryBytes = std::max(Stats.PeakTemporaryBytes, Stats.TemporaryBytes);
  }
  ~TemporaryWindow() { Stats.TemporaryBytes -= KReadChunk; }
};
struct InflateCleanup { z_stream &Stream; ~InflateCleanup() { inflateEnd(&Stream); } };
} // namespace

PackReader::PackReader(std::size_t Budget) noexcept : CacheBudget_(Budget) {}
PackReader::~PackReader() noexcept { Close(); }
PackReader::PackReader(PackReader &&Other) noexcept { *this = std::move(Other); }
PackReader &PackReader::operator=(PackReader &&Other) noexcept
{
  if (this == &Other) return *this;
  Close();
  std::scoped_lock Lock(CacheMutex_, Other.CacheMutex_);
  PackFd_ = std::exchange(Other.PackFd_, -1);
  PackSize_ = std::exchange(Other.PackSize_, 0);
  OpenError_ = std::exchange(Other.OpenError_, Error::NotOpen);
  Info_ = std::exchange(Other.Info_, PackageInfo{});
  Strings_ = std::move(Other.Strings_);
  Slots_ = std::move(Other.Slots_);
  Sources_ = std::move(Other.Sources_);
  Bundles_ = std::move(Other.Bundles_);
  NumericBundles_ = std::move(Other.NumericBundles_);
  NamedBundles_ = std::move(Other.NamedBundles_);
  Preloads_ = std::move(Other.Preloads_);
  Cache_ = std::move(Other.Cache_);
  CacheLru_ = std::move(Other.CacheLru_);
  Budget_ = std::move(Other.Budget_);
  Stats_ = std::exchange(Other.Stats_, CacheStats{});
  CacheBudget_ = Other.CacheBudget_;
  return *this;
}
bool PackReader::SourceView::operator<(const SourceView &Other) const noexcept
{
  return std::tie(Origin, BundleId, SerializedFile, PathId) <
         std::tie(Other.Origin, Other.BundleId, Other.SerializedFile, Other.PathId);
}
bool PackReader::SourceView::operator==(const SourceView &Other) const noexcept
{
  return PathId == Other.PathId && Origin == Other.Origin && BundleId == Other.BundleId &&
         SerializedFile == Other.SerializedFile;
}
void PackReader::ResetIndexes() noexcept
{
  std::vector<SourceRecord>().swap(Sources_);
  std::vector<Slot>().swap(Slots_);
  std::vector<BundleRecord>().swap(Bundles_);
  std::vector<std::uint32_t>().swap(NumericBundles_);
  std::vector<std::uint32_t>().swap(NamedBundles_);
  std::vector<BundlePreloadEntry>().swap(Preloads_);
  std::vector<char>().swap(Strings_);
  Info_ = {};
}
void PackReader::Close() noexcept
{
  std::lock_guard<std::mutex> Lock(CacheMutex_);
  if (PackFd_ >= 0) close(PackFd_);
  PackFd_ = -1;
  PackSize_ = 0;
  OpenError_ = Error::NotOpen;
  ResetIndexes();
  Cache_.clear();
  CacheLru_.clear();
  Stats_ = {};
  if (Budget_) Budget_->Peak.store(Budget_->Resident.load());
}
bool PackReader::Open() noexcept { return Open(KPackPackPath); }
bool PackReader::Open(const char *Path) noexcept
{
  Close();
  Error Failure = Error::None;
  if (!Path || !*Path) Failure = Error::InvalidArgument;
  else
  {
    try
    {
      if (!Budget_) Budget_ = std::make_shared<DecodedBudget>();
      Failure = OpenPack(Path) ? ReadPackIndex() : Error::IoError;
    }
    catch (const std::bad_alloc &) { Failure = Error::OutOfMemory; }
    catch (...) { Failure = Error::CorruptData; }
  }
  if (Failure != Error::None) Close();
  OpenError_ = Failure;
  return Failure == Error::None;
}
bool PackReader::OpenPack(const char *Path) noexcept
{
  const int Fd = open(Path, O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
  if (Fd < 0) return false;
  struct stat FileStat{};
  if (fstat(Fd, &FileStat) != 0 || !S_ISREG(FileStat.st_mode) || FileStat.st_size < 0)
  { close(Fd); return false; }
  PackFd_ = Fd;
  PackSize_ = static_cast<std::uint64_t>(FileStat.st_size);
  return true;
}
bool PackReader::ReadAt(void *Buffer, std::size_t Size, std::uint64_t Offset) const noexcept
{
  if (PackFd_ < 0 || (!Buffer && Size) || !RangeWithinFile(Offset, Size)) return false;
  std::size_t Done = 0;
  while (Done < Size)
  {
    const std::uint64_t Position = Offset + Done;
    if (Position > static_cast<std::uint64_t>(std::numeric_limits<off_t>::max())) return false;
    const ssize_t Result = pread(PackFd_, static_cast<unsigned char *>(Buffer) + Done,
                                 Size - Done, static_cast<off_t>(Position));
    if (Result > 0) Done += static_cast<std::size_t>(Result);
    else if (Result < 0 && errno == EINTR) continue;
    else return false;
  }
  return true;
}
bool PackReader::RangeWithinFile(std::uint64_t Offset, std::uint64_t Size) const noexcept
{
  return AddWithin(Offset, Size, PackSize_);
}

Error PackReader::ReadPackIndex()
{
  if (!RangeWithinFile(0, KPackHeaderSize)) return Error::CorruptData;
  std::array<unsigned char, KPackHeaderSize> Header{};
  if (!ReadAt(Header.data(), Header.size(), 0)) return Error::IoError;
  if (!std::equal(KPackMagic.begin(), KPackMagic.end(), Header.begin())) return Error::CorruptData;
  const auto HeaderBytes = LoadU32(Header.data() + 8);
  if (HeaderBytes < KPackHeaderSize || HeaderBytes > PackSize_) return Error::CorruptData;
  const auto SlotOffset = LoadU64(Header.data() + 24), SourceOffset = LoadU64(Header.data() + 40);
  const auto PreloadOffset = LoadU64(Header.data() + 56), DataOffset = LoadU64(Header.data() + 72);
  const auto DataSize = LoadU64(Header.data() + 80), StringOffset = LoadU64(Header.data() + 88);
  const auto StringSize = LoadU64(Header.data() + 96);
  const auto SlotCount = LoadU32(Header.data() + 32), SlotStride = LoadU32(Header.data() + 36);
  const auto SourceCount = LoadU32(Header.data() + 48), SourceStride = LoadU32(Header.data() + 52);
  const auto PreloadCount = LoadU32(Header.data() + 64), PreloadStride = LoadU32(Header.data() + 68);
  const std::uint64_t SlotBytes = static_cast<std::uint64_t>(SlotCount) * SlotStride;
  const std::uint64_t SourceBytes = static_cast<std::uint64_t>(SourceCount) * SourceStride;
  const std::uint64_t PreloadBytes = static_cast<std::uint64_t>(PreloadCount) * PreloadStride;
  if (SlotCount > KMaximumTableCount || SourceCount > KMaximumTableCount || PreloadCount > KMaximumPreloadCount ||
      SlotStride < KPackSlotStride || SourceStride < KPackSourceStride || PreloadStride < KPackPreloadStride ||
      StringSize > KMaximumStringPoolSize || !RangeWithinFile(DataOffset, DataSize) ||
      !RangeWithinFile(StringOffset, StringSize) || !RangeWithinFile(SlotOffset, SlotBytes) ||
      !RangeWithinFile(SourceOffset, SourceBytes) || !RangeWithinFile(PreloadOffset, PreloadBytes))
    return Error::CorruptData;
  // Offsets, not physical table order, define the v1 directories. Empty tables
  // may use offset zero; populated metadata and data regions must not overlap.
  const std::array<std::pair<std::uint64_t, std::uint64_t>, 6> Ranges{{
      {0, HeaderBytes}, {SlotOffset, SlotBytes}, {SourceOffset, SourceBytes},
      {PreloadOffset, PreloadBytes}, {StringOffset, StringSize}, {DataOffset, DataSize}}};
  for (std::size_t A = 0; A < Ranges.size(); ++A)
    for (std::size_t B = A + 1; B < Ranges.size(); ++B)
      if (Ranges[A].second && Ranges[B].second &&
          Ranges[A].first < Ranges[B].first + Ranges[B].second &&
          Ranges[B].first < Ranges[A].first + Ranges[A].second) return Error::CorruptData;

  Strings_.resize(static_cast<std::size_t>(StringSize));
  if (!ReadAt(Strings_.data(), Strings_.size(), StringOffset)) return Error::IoError;
  auto ReadString = [&](const unsigned char *Record, std::string_view &Value)
  {
    const auto At = LoadU32(Record), Length = LoadU32(Record + 4);
    if (!AddWithin(At, Length, StringSize)) return false;
    Value = Length ? std::string_view(Strings_.data() + At, Length) : std::string_view{};
    return true;
  };
  auto ReadTable = [&](std::uint64_t Offset, std::uint32_t Count, std::uint32_t Stride,
                       std::size_t RecordSize, auto Consume) -> bool
  {
    std::array<unsigned char, KReadChunk> Window{};
    for (std::uint32_t Index = 0; Index < Count;)
    {
      if (Stride > Window.size())
      {
        if (!ReadAt(Window.data(), RecordSize, Offset + static_cast<std::uint64_t>(Index) * Stride)) return false;
        Consume(Window.data());
        ++Index;
      }
      else
      {
        const auto Batch = std::min<std::uint32_t>(Count - Index, static_cast<std::uint32_t>(Window.size() / Stride));
        if (!ReadAt(Window.data(), static_cast<std::size_t>(Batch) * Stride,
                    Offset + static_cast<std::uint64_t>(Index) * Stride)) return false;
        for (std::uint32_t Item = 0; Item < Batch; ++Item) Consume(Window.data() + Item * Stride);
        Index += Batch;
      }
    }
    return true;
  };
  Slots_.reserve(SlotCount);
  if (!ReadTable(SlotOffset, SlotCount, SlotStride, KPackSlotStride, [&](const unsigned char *Record)
  {
    Slot Value;
    Value.Id = LoadU32(Record);
    if (Value.Id == KInvalidGeometrySlot) return;
    Value.Kind = static_cast<GeometryKind>(LoadU32(Record + 4));
    Value.Codec = LoadU32(Record + 8);
    Value.Offset = LoadU64(Record + 16);
    Value.CompressedSize = LoadU64(Record + 24);
    Value.RawSize = LoadU64(Record + 32);
    if (!RangeWithinFile(Value.Offset, Value.CompressedSize) || Value.RawSize > KPackCacheBudget ||
        Value.CompressedSize > KPackCacheBudget ||
        (DataSize && (Value.Offset < DataOffset || !AddWithin(Value.Offset, Value.CompressedSize, DataOffset + DataSize))) ||
        (Value.Codec == 0 && Value.RawSize != Value.CompressedSize)) Value.State = Error::CorruptData;
    else if (Value.Codec > 1 || static_cast<std::uint32_t>(Value.Kind) > 8) Value.State = Error::UnsupportedGeometry;
    Slots_.push_back(Value);
  })) return Error::IoError;
  std::sort(Slots_.begin(), Slots_.end(), [](const Slot &A, const Slot &B) { return A.Id < B.Id; });
  std::size_t Written = 0;
  for (const auto &Value : Slots_)
  {
    if (Written && Slots_[Written - 1].Id == Value.Id)
    {
      auto &Previous = Slots_[Written - 1];
      if (std::tie(Previous.Kind, Previous.Codec, Previous.Offset, Previous.CompressedSize, Previous.RawSize) !=
          std::tie(Value.Kind, Value.Codec, Value.Offset, Value.CompressedSize, Value.RawSize)) Previous.State = Error::CorruptData;
    }
    else Slots_[Written++] = Value;
  }
  Slots_.resize(Written);
  Sources_.reserve(SourceCount);
  if (!ReadTable(SourceOffset, SourceCount, SourceStride, KPackSourceStride, [&](const unsigned char *Record)
  {
    SourceRecord Value;
    if (!ReadString(Record, Value.Key.Origin) || !ReadString(Record + 8, Value.Key.BundleId) ||
        !ReadString(Record + 16, Value.Key.SerializedFile)) return;
    Value.Key.PathId = LoadU64(Record + 24);
    Value.GeometrySlot = LoadU32(Record + 32);
    Sources_.push_back(Value);
  })) return Error::IoError;
  std::sort(Sources_.begin(), Sources_.end(), [](const SourceRecord &A, const SourceRecord &B) { return A.Key < B.Key; });
  Written = 0;
  for (const auto &Value : Sources_)
  {
    if (Written && Sources_[Written - 1].Key == Value.Key)
    {
      auto &Previous = Sources_[Written - 1];
      if (Previous.GeometrySlot != Value.GeometrySlot) Previous.State = Error::CorruptData;
    }
    else Sources_[Written++] = Value;
  }
  Sources_.resize(Written);

  std::vector<PreloadRow> Rows;
  Rows.reserve(PreloadCount);
  std::unordered_map<BundleKey, std::uint32_t, BundleKeyHash> BundleLookup;
  if (!ReadTable(PreloadOffset, PreloadCount, PreloadStride, KPackPreloadStride, [&](const unsigned char *Record)
  {
    BundleKey Key;
    if (!ReadString(Record, Key.Text)) return;
    Key.Numeric = ParseBundleId(Key.Text, Key.Number);
    const auto Inserted = BundleLookup.emplace(Key, static_cast<std::uint32_t>(Bundles_.size()));
    const auto Expected = LoadU64(Record + 16);
    if (Inserted.second)
    {
      BundleRecord Bundle;
      Bundle.Name = Key.Text;
      Bundle.NumericId = Key.Number;
      Bundle.Numeric = Key.Numeric;
      Bundle.Directory.ExpectedCount = Expected;
      Bundle.Directory.CountDefined = Expected <= KMaximumPreloadCount;
      Bundles_.push_back(Bundle);
    }
    auto &Directory = Bundles_[Inserted.first->second].Directory;
    if (Directory.ExpectedCount != Expected) Directory.CountDefined = false;
    const auto Ordinal = LoadU32(Record + 8), GeometrySlot = LoadU32(Record + 12);
    if (Ordinal >= Expected) Directory.CountDefined = false;
    const Slot *Definition = FindSlot(GeometrySlot);
    Rows.push_back({Inserted.first->second, Ordinal,
                   Definition && Definition->State == Error::None ? GeometrySlot : KInvalidGeometrySlot});
  })) return Error::IoError;
  std::sort(Rows.begin(), Rows.end(), [](const PreloadRow &A, const PreloadRow &B)
  { return std::tie(A.Bundle, A.Ordinal) < std::tie(B.Bundle, B.Ordinal); });
  Preloads_.reserve(Rows.size());
  std::size_t At = 0;
  for (std::size_t BundleIndex = 0; BundleIndex < Bundles_.size(); ++BundleIndex)
  {
    const auto Begin = Preloads_.size();
    while (At < Rows.size() && Rows[At].Bundle == BundleIndex)
    {
      const auto Row = Rows[At++];
      if (Preloads_.size() > Begin && Preloads_.back().Ordinal == Row.Ordinal)
      {
        if (Preloads_.back().GeometrySlot != Row.Slot) Preloads_.back().GeometrySlot = KInvalidGeometrySlot;
      }
      else Preloads_.push_back({Row.Ordinal, Row.Slot});
    }
    auto &Bundle = Bundles_[BundleIndex];
    const auto Count = Preloads_.size() - Begin;
    Bundle.Directory.Entries = BundlePreloadEntries(Count ? Preloads_.data() + Begin : nullptr, Count);
    (Bundle.Numeric ? NumericBundles_ : NamedBundles_).push_back(static_cast<std::uint32_t>(BundleIndex));
  }
  std::sort(NumericBundles_.begin(), NumericBundles_.end(), [&](auto A, auto B) { return Bundles_[A].NumericId < Bundles_[B].NumericId; });
  std::sort(NamedBundles_.begin(), NamedBundles_.end(), [&](auto A, auto B) { return Bundles_[A].Name < Bundles_[B].Name; });
  Info_.FileBytes = PackSize_;
  Info_.SourceCount = SourceCount;
  Info_.BlockCount = SlotCount;
  Info_.PreloadCount = PreloadCount;
  Info_.IndexBytes = Strings_.capacity() + Slots_.capacity() * sizeof(Slot) + Sources_.capacity() * sizeof(SourceRecord) +
      Bundles_.capacity() * sizeof(BundleRecord) + Preloads_.capacity() * sizeof(BundlePreloadEntry) +
      (NumericBundles_.capacity() + NamedBundles_.capacity()) * sizeof(std::uint32_t);
  return Error::None;
}

const PackReader::Slot *PackReader::FindSlot(std::uint32_t Id) const noexcept
{
  const auto It = std::lower_bound(Slots_.begin(), Slots_.end(), Id,
                                 [](const Slot &Value, std::uint32_t Key) { return Value.Id < Key; });
  return It == Slots_.end() || It->Id != Id ? nullptr : &*It;
}
Error PackReader::ResolveSource(const SourceKey &Key, std::uint32_t *GeometrySlot) const noexcept
{
  if (!GeometrySlot) return Error::InvalidArgument;
  *GeometrySlot = KInvalidGeometrySlot;
  if (!IsOpen()) return Error::NotOpen;
  if (Key.Origin.empty() || Key.BundleId.empty() || Key.SerializedFile.empty()) return Error::InvalidArgument;
  const SourceView View{Key.Origin, Key.BundleId, Key.SerializedFile, Key.PathId};
  const auto It = std::lower_bound(Sources_.begin(), Sources_.end(), View,
                                 [](const SourceRecord &Value, const SourceView &Search) { return Value.Key < Search; });
  if (It == Sources_.end() || !(It->Key == View)) return Error::MissingResource;
  if (It->State != Error::None) return It->State;
  const auto *Definition = FindSlot(It->GeometrySlot);
  if (!Definition) return Error::MissingResource;
  if (Definition->State != Error::None) return Definition->State;
  *GeometrySlot = It->GeometrySlot;
  return Error::None;
}
bool PackReader::LookupSource(const SourceKey &Key, std::uint32_t *GeometrySlot) const noexcept
{
  return ResolveSource(Key, GeometrySlot) == Error::None;
}
const PackReader::BundleRecord *PackReader::FindBundle(std::string_view Name) const noexcept
{
  std::uint64_t Numeric = 0;
  if (ParseBundleId(Name, Numeric))
  {
    const auto It = std::lower_bound(NumericBundles_.begin(), NumericBundles_.end(), Numeric,
                                   [&](auto Index, auto Value) { return Bundles_[Index].NumericId < Value; });
    return It == NumericBundles_.end() || Bundles_[*It].NumericId != Numeric ? nullptr : &Bundles_[*It];
  }
  const auto It = std::lower_bound(NamedBundles_.begin(), NamedBundles_.end(), Name,
                                 [&](auto Index, auto Value) { return Bundles_[Index].Name < Value; });
  return It == NamedBundles_.end() || Bundles_[*It].Name != Name ? nullptr : &Bundles_[*It];
}
PreloadStatus PackReader::LookupPreload(const std::string &BundleId, std::uint32_t Ordinal,
                                      std::uint32_t *GeometrySlot) const noexcept
{
  if (GeometrySlot) *GeometrySlot = KInvalidGeometrySlot;
  if (!GeometrySlot || !IsOpen()) return PreloadStatus::Unresolved;
  const auto *Bundle = FindBundle(BundleId);
  if (!Bundle) return PreloadStatus::Missing;
  const auto &Directory = Bundle->Directory;
  if (!Directory.CountDefined) return PreloadStatus::Unresolved;
  if (Ordinal >= Directory.ExpectedCount) return PreloadStatus::Missing;
  const auto It = std::lower_bound(Directory.Entries.begin(), Directory.Entries.end(), Ordinal,
                                 [](const auto &Entry, auto Search) { return Entry.Ordinal < Search; });
  if (It == Directory.Entries.end() || It->Ordinal != Ordinal) return PreloadStatus::Missing;
  if (It->GeometrySlot == KInvalidGeometrySlot) return PreloadStatus::Unresolved;
  *GeometrySlot = It->GeometrySlot;
  return PreloadStatus::Bound;
}
bool PackReader::BundlePreloadCount(const std::string &BundleId, std::uint64_t *Output) const noexcept
{
  if (Output) *Output = 0;
  if (!Output || !IsOpen()) return false;
  const auto *Bundle = FindBundle(BundleId);
  if (!Bundle || !Bundle->Directory.CountDefined) return false;
  *Output = Bundle->Directory.ExpectedCount;
  return true;
}
const BundlePreloadDirectory *PackReader::GetBundlePreloadDirectory(std::uint64_t Id) const noexcept
{
  if (!IsOpen()) return nullptr;
  const auto It = std::lower_bound(NumericBundles_.begin(), NumericBundles_.end(), Id,
                                 [&](auto Index, auto Value) { return Bundles_[Index].NumericId < Value; });
  return It == NumericBundles_.end() || Bundles_[*It].NumericId != Id ? nullptr : &Bundles_[*It].Directory;
}
bool PackReader::GetGeometryKind(std::uint32_t Id, GeometryKind *Output) const noexcept
{
  if (Output) *Output = GeometryKind::Unknown;
  if (!Output || !IsOpen()) return false;
  const auto *Definition = FindSlot(Id);
  if (!Definition || Definition->State != Error::None) return false;
  *Output = Definition->Kind;
  return true;
}
CacheStats PackReader::Stats() const noexcept
{
  std::lock_guard<std::mutex> Lock(CacheMutex_);
  auto Result = Stats_;
  Result.BudgetBytes = CacheBudget_;
  if (Budget_)
  {
    Result.ResidentBytes = Budget_->Resident.load();
    Result.PeakResidentBytes = Budget_->Peak.load();
  }
  return Result;
}
bool PackReader::ReserveCacheSpace(std::size_t Size) noexcept
{
  if (Size > CacheBudget_) return false;
  auto It = CacheLru_.begin();
  while (Budget_->Resident.load() > CacheBudget_ - Size && It != CacheLru_.end())
  {
    const auto Entry = Cache_.find(*It);
    if (Entry->second.Data.use_count() > 1) { ++It; continue; }
    Cache_.erase(Entry);
    It = CacheLru_.erase(It);
    ++Stats_.Evictions;
  }
  return Budget_->Resident.load() <= CacheBudget_ - Size;
}
Error PackReader::DecodeBlock(const Slot &Value, std::vector<std::uint8_t> &Output)
{
  if (Value.Codec == 0)
    return ReadAt(Output.data(), Output.size(), Value.Offset) ? Error::None : Error::IoError;
  std::array<unsigned char, KReadChunk> Input{};
  TemporaryWindow Window(Stats_);
  z_stream Stream{};
  Stream.zalloc = ZAllocate;
  Stream.zfree = ZFree;
  Stream.opaque = &Stats_;
  const int Started = inflateInit(&Stream);
  if (Started != Z_OK) return Started == Z_MEM_ERROR ? Error::OutOfMemory : Error::CorruptData;
  InflateCleanup Cleanup{Stream};
  unsigned char Overflow = 0;
  Stream.next_out = Output.empty() ? &Overflow : Output.data();
  Stream.avail_out = Output.empty() ? 1U : static_cast<uInt>(Output.size());
  std::uint64_t Read = 0;
  for (;;)
  {
    if (Stream.avail_in == 0 && Read < Value.CompressedSize)
    {
      const auto Bytes = static_cast<std::size_t>(std::min<std::uint64_t>(Input.size(), Value.CompressedSize - Read));
      if (!ReadAt(Input.data(), Bytes, Value.Offset + Read)) return Error::IoError;
      Read += Bytes;
      Stream.next_in = Input.data();
      Stream.avail_in = static_cast<uInt>(Bytes);
    }
    const auto BeforeIn = Stream.total_in, BeforeOut = Stream.total_out;
    const int Result = inflate(&Stream, Z_NO_FLUSH);
    if (Stream.total_out > Value.RawSize) return Error::CorruptData;
    if (Result == Z_STREAM_END)
      return Stream.total_out == Value.RawSize && Read == Value.CompressedSize && Stream.avail_in == 0 ?
          Error::None : Error::CorruptData;
    if (Result != Z_OK) return Result == Z_MEM_ERROR ? Error::OutOfMemory : Error::CorruptData;
    if (Stream.total_in == BeforeIn && Stream.total_out == BeforeOut) return Error::CorruptData;
    if (Stream.avail_out == 0) { Stream.next_out = &Overflow; Stream.avail_out = 1; }
  }
}
bool PackReader::AcquireBlock(std::uint32_t Id, BlockView *Output, Error *Failure) noexcept
{
  auto Fail = [&](Error Value) { if (Failure) *Failure = Value; return false; };
  if (Failure) *Failure = Error::None;
  if (!Output) return Fail(Error::InvalidArgument);
  Output->Storage.reset();
  if (!IsOpen()) return Fail(Error::NotOpen);
  std::lock_guard<std::mutex> Lock(CacheMutex_);
  try
  {
    const auto *Value = FindSlot(Id);
    if (!Value) return Fail(Error::MissingResource);
    if (Value->State != Error::None) return Fail(Value->State);
    const auto Cached = Cache_.find(Id);
    if (Cached != Cache_.end())
    {
      CacheLru_.splice(CacheLru_.end(), CacheLru_, Cached->second.Lru);
      Output->Storage = Cached->second.Data;
      ++Stats_.Hits;
      return true;
    }
    ++Stats_.Misses;
    const auto Size = static_cast<std::size_t>(Value->RawSize);
    if (!ReserveCacheSpace(Size)) return Fail(Error::BudgetExceeded);
    auto *Allocation = new std::vector<std::uint8_t>(Size);
    const auto Budget = Budget_;
    const auto Resident = Budget->Resident.fetch_add(Size) + Size;
    auto Peak = Budget->Peak.load();
    while (Resident > Peak && !Budget->Peak.compare_exchange_weak(Peak, Resident)) {}
    std::shared_ptr<std::vector<std::uint8_t>> Decoded(Allocation, [Budget, Size](auto *Data)
    { delete Data; Budget->Resident.fetch_sub(Size); });
    const auto DecodedError = DecodeBlock(*Value, *Decoded);
    if (DecodedError != Error::None) return Fail(DecodedError);
    Stats_.BytesDecoded += Size;
    CacheLru_.push_back(Id);
    try { Cache_.emplace(Id, CacheEntry{Decoded, std::prev(CacheLru_.end())}); }
    catch (...) { CacheLru_.pop_back(); throw; }
    Output->Storage = std::move(Decoded);
    return true;
  }
  catch (const std::bad_alloc &) { return Fail(Error::OutOfMemory); }
  catch (...) { return Fail(Error::CorruptData); }
}
} // namespace detail
} // namespace physx_pack
