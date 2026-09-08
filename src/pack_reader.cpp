#include "physx_pack/pack_reader.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <fcntl.h>
#include <limits>
#include <sys/stat.h>
#include <unistd.h>
#include <utility>

#include <zlib.h>

namespace physx_pack
{
namespace detail
{
namespace
{

constexpr std::array<unsigned char, 8> KPackMagic{'X', 'C', 'P', 'H',
                                                  'Y', 'S', 'X', 0};
constexpr std::size_t KMaximumTableCount = 1U << 20U;
constexpr std::size_t KMaximumStringPoolSize = 8U * 1024U * 1024U;
constexpr std::uint32_t KCodecRaw = 0U;
constexpr std::uint32_t KCodecZlib = 1U;

std::uint32_t LoadU32(const unsigned char *Value)
{
  return static_cast<std::uint32_t>(Value[0]) |
         (static_cast<std::uint32_t>(Value[1]) << 8U) |
         (static_cast<std::uint32_t>(Value[2]) << 16U) |
         (static_cast<std::uint32_t>(Value[3]) << 24U);
}

std::uint64_t LoadU64(const unsigned char *Value)
{
  std::uint64_t Result = 0;
  for (unsigned Index = 0; Index < 8U; ++Index)
  {
    Result |= static_cast<std::uint64_t>(Value[Index]) << (Index * 8U);
  }
  return Result;
}

bool AddWithin(std::uint64_t Offset, std::uint64_t Size, std::uint64_t Limit)
{
  return Offset <= Limit && Size <= Limit - Offset;
}

bool MultiplyWithin(std::uint64_t Left, std::uint64_t Right,
                    std::uint64_t Limit, std::uint64_t *Result)
{
  if (Result == nullptr || (Left != 0 && Right > Limit / Left))
  {
    return false;
  }
  const std::uint64_t Product = Left * Right;
  if (Product > Limit)
  {
    return false;
  }
  *Result = Product;
  return true;
}

bool ParseBundleId(const std::string &Text, std::uint64_t &Output) noexcept
{
  Output = 0;
  if (Text.size() != 16U)
  {
    return false;
  }
  for (const char Character : Text)
  {
    std::uint64_t Digit = 0;
    if (Character >= '0' && Character <= '9')
    {
      Digit = static_cast<std::uint64_t>(Character - '0');
    }
    else if (Character >= 'a' && Character <= 'f')
    {
      Digit = static_cast<std::uint64_t>(Character - 'a' + 10);
    }
    else
    {
      return false;
    }
    Output = (Output << 4U) | Digit;
  }
  return true;
}

} // namespace

PackReader::PackReader(
    std::size_t CacheBudget) noexcept
    : CacheBudget_(std::min(CacheBudget, KPackCacheBudget))
{
}

PackReader::~PackReader() noexcept { Close(); }

PackReader::PackReader(
    PackReader &&Other) noexcept
    : PackFd_(Other.PackFd_), PackSize_(Other.PackSize_),
      Slots_(std::move(Other.Slots_)), SlotIndex_(std::move(Other.SlotIndex_)),
      SourceIndex_(std::move(Other.SourceIndex_)),
      PreloadIndex_(std::move(Other.PreloadIndex_)),
      BundlePreloadCounts_(std::move(Other.BundlePreloadCounts_)),
      NumericBundlePreloads_(std::move(Other.NumericBundlePreloads_)),
      Cache_(std::move(Other.Cache_)), CacheLru_(std::move(Other.CacheLru_)),
      CachedBytes_(Other.CachedBytes_),
      CacheBudget_(Other.CacheBudget_)
          PHYSX_PACK_STAT(, Stats_(Other.Stats_))
{
  Other.PackFd_ = -1;
  Other.PackSize_ = 0;
  Other.CachedBytes_ = 0;
  PHYSX_PACK_STAT(Other.Stats_ = {};)
}

PackReader &
PackReader::operator=(PackReader &&Other) noexcept
{
  if (this != &Other)
  {
    Close();
    PackFd_ = Other.PackFd_;
    PackSize_ = Other.PackSize_;
    Slots_ = std::move(Other.Slots_);
    SlotIndex_ = std::move(Other.SlotIndex_);
    SourceIndex_ = std::move(Other.SourceIndex_);
    PreloadIndex_ = std::move(Other.PreloadIndex_);
    BundlePreloadCounts_ = std::move(Other.BundlePreloadCounts_);
    NumericBundlePreloads_ = std::move(Other.NumericBundlePreloads_);
    Cache_ = std::move(Other.Cache_);
    CacheLru_ = std::move(Other.CacheLru_);
    CachedBytes_ = Other.CachedBytes_;
    CacheBudget_ = Other.CacheBudget_;
    PHYSX_PACK_STAT(Stats_ = Other.Stats_;)
    Other.PackFd_ = -1;
    Other.PackSize_ = 0;
    Other.CachedBytes_ = 0;
    PHYSX_PACK_STAT(Other.Stats_ = {};)
  }
  return *this;
}

std::size_t PackReader::SourceKeyHash::operator()(
    const SourceKey &Key) const noexcept
{
  std::size_t Hash = std::hash<std::string>{}(Key.Origin);
  Hash ^= std::hash<std::string>{}(Key.BundleId) + 0x9E3779B9U + (Hash << 6U) +
          (Hash >> 2U);
  Hash ^= std::hash<std::string>{}(Key.SerializedFile) + 0x9E3779B9U +
          (Hash << 6U) + (Hash >> 2U);
  Hash ^= std::hash<std::uint64_t>{}(Key.PathId) + 0x9E3779B9U + (Hash << 6U) +
          (Hash >> 2U);
  return Hash;
}

std::size_t PackReader::PreloadKeyHash::operator()(
    const PreloadKey &Key) const noexcept
{
  std::size_t Hash = std::hash<std::string>{}(Key.BundleId);
  Hash ^= std::hash<std::uint32_t>{}(Key.Ordinal) + 0x9E3779B9U + (Hash << 6U) +
          (Hash >> 2U);
  return Hash;
}

void PackReader::ResetIndexes() noexcept
{
  Slots_.clear();
  SlotIndex_.clear();
  SourceIndex_.clear();
  PreloadIndex_.clear();
  BundlePreloadCounts_.clear();
  NumericBundlePreloads_.clear();
  Cache_.clear();
  CacheLru_.clear();
  CachedBytes_ = 0;
}

void PackReader::Close() noexcept
{
  if (PackFd_ >= 0)
  {
    close(PackFd_);
    PackFd_ = -1;
  }
  PackSize_ = 0;
  ResetIndexes();
  PHYSX_PACK_STAT(Stats_ = {};)
}

bool PackReader::Open() noexcept
{
  return Open(KPackPackPath);
}

bool PackReader::Open(const char *PackPath) noexcept
{
  Close();
  if (PackPath == nullptr)
  {
    return false;
  }
  try
  {
    if (!OpenPack(PackPath) || !ReadPackIndex())
    {
      Close();
      return false;
    }
    return true;
  }
  catch (...)
  {
    Close();
    return false;
  }
}

bool PackReader::OpenPack(const char *Path) noexcept
{
  const int Fd = open(Path, O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
  if (Fd < 0)
  {
    return false;
  }
  struct stat FileStat{};
  if (fstat(Fd, &FileStat) != 0 || !S_ISREG(FileStat.st_mode) ||
      FileStat.st_size < 0)
  {
    close(Fd);
    return false;
  }
  PackFd_ = Fd;
  PackSize_ = static_cast<std::uint64_t>(FileStat.st_size);
  return true;
}

bool PackReader::ReadAt(void *Buffer, std::size_t Size,
                                      std::uint64_t Offset) const noexcept
{
  if (PackFd_ < 0 || Buffer == nullptr ||
      Offset > static_cast<std::uint64_t>(std::numeric_limits<off_t>::max()))
  {
    return false;
  }
  std::size_t Done = 0;
  while (Done < Size)
  {
    if (Done > std::numeric_limits<std::uint64_t>::max() - Offset)
    {
      return false;
    }
    const std::uint64_t Position = Offset + Done;
    if (Position >
        static_cast<std::uint64_t>(std::numeric_limits<off_t>::max()))
    {
      return false;
    }
    const ssize_t Result =
        pread(PackFd_, static_cast<unsigned char *>(Buffer) + Done, Size - Done,
              static_cast<off_t>(Position));
    if (Result > 0)
    {
      Done += static_cast<std::size_t>(Result);
      continue;
    }
    if (Result < 0 && errno == EINTR)
    {
      continue;
    }
    return false;
  }
  return true;
}

bool PackReader::RangeWithinFile(
    std::uint64_t Offset, std::uint64_t Size) const noexcept
{
  return AddWithin(Offset, Size, PackSize_);
}

bool PackReader::ReadPackIndex()
{
  if (!RangeWithinFile(0, KPackHeaderSize))
  {
    return false;
  }
  std::array<unsigned char, KPackHeaderSize> Header{};
  if (!ReadAt(Header.data(), Header.size(), 0) ||
      !std::equal(KPackMagic.begin(), KPackMagic.end(), Header.begin()))
  {
    return false;
  }

  const std::uint32_t HeaderBytes = LoadU32(Header.data() + 8U);
  if (HeaderBytes < KPackHeaderSize || HeaderBytes > PackSize_)
  {
    return false;
  }
  // 保留字节只供格式扩展，不进行完整性、资源代际或版本门禁。
  const std::uint64_t SlotOffset = LoadU64(Header.data() + 24U);
  const std::uint32_t SlotCount = LoadU32(Header.data() + 32U);
  const std::uint32_t SlotStride = LoadU32(Header.data() + 36U);
  const std::uint64_t SourceOffset = LoadU64(Header.data() + 40U);
  const std::uint32_t SourceCount = LoadU32(Header.data() + 48U);
  const std::uint32_t SourceStride = LoadU32(Header.data() + 52U);
  const std::uint64_t PreloadOffset = LoadU64(Header.data() + 56U);
  const std::uint32_t PreloadCount = LoadU32(Header.data() + 64U);
  const std::uint32_t PreloadStride = LoadU32(Header.data() + 68U);
  const std::uint64_t DataOffset = LoadU64(Header.data() + 72U);
  const std::uint64_t DataSize = LoadU64(Header.data() + 80U);
  const std::uint64_t StringOffset = LoadU64(Header.data() + 88U);
  const std::uint64_t StringSize = LoadU64(Header.data() + 96U);

  if (SlotCount > KMaximumTableCount || SourceCount > KMaximumTableCount ||
      PreloadCount > KMaximumTableCount || SlotStride < 40U ||
      SourceStride < 40U || PreloadStride < KPackPreloadStride ||
      !RangeWithinFile(DataOffset, DataSize) ||
      !RangeWithinFile(StringOffset, StringSize) ||
      StringSize > KMaximumStringPoolSize)
  {
    return false;
  }
  std::uint64_t TableBytes = 0;
  if (!MultiplyWithin(SlotCount, SlotStride, PackSize_, &TableBytes) ||
      !RangeWithinFile(SlotOffset, TableBytes) ||
      !MultiplyWithin(SourceCount, SourceStride, PackSize_, &TableBytes) ||
      !RangeWithinFile(SourceOffset, TableBytes) ||
      !MultiplyWithin(PreloadCount, PreloadStride, PackSize_, &TableBytes) ||
      !RangeWithinFile(PreloadOffset, TableBytes))
  {
    return false;
  }

  std::vector<unsigned char> Strings(static_cast<std::size_t>(StringSize));
  if (StringSize != 0 && !ReadAt(Strings.data(), Strings.size(), StringOffset))
  {
    return false;
  }
  Slots_.clear();
  Slots_.reserve(SlotCount);
  SlotIndex_.reserve(SlotCount);

  std::array<unsigned char, KPackSlotStride> Record{};
  for (std::uint32_t Index = 0; Index < SlotCount; ++Index)
  {
    Record.fill(0);
    if (!ReadAt(Record.data(), Record.size(),
                SlotOffset + static_cast<std::uint64_t>(Index) * SlotStride))
    {
      return false;
    }
    const std::uint32_t SlotId = LoadU32(Record.data());
    if (SlotId == KInvalidGeometrySlot)
    {
      continue;
    }
    Slot SlotValue{};
    SlotValue.Kind =
        static_cast<::physx_pack::detail::GeometryKind>(
            LoadU32(Record.data() + 4U));
    SlotValue.Codec = LoadU32(Record.data() + 8U);
    SlotValue.Offset = LoadU64(Record.data() + 16U);
    SlotValue.CompressedSize = LoadU64(Record.data() + 24U);
    SlotValue.RawSize = LoadU64(Record.data() + 32U);
    SlotValue.Defined =
        RangeWithinFile(SlotValue.Offset, SlotValue.CompressedSize) &&
        SlotValue.RawSize <= KPackCacheBudget &&
        SlotValue.CompressedSize <= KPackCacheBudget &&
        (DataSize == 0 || (SlotValue.Offset >= DataOffset &&
                           AddWithin(SlotValue.Offset, SlotValue.CompressedSize,
                                     DataOffset + DataSize)));
    const std::size_t SlotIndex = Slots_.size();
    Slots_.push_back(SlotValue);
    const auto Inserted = SlotIndex_.emplace(SlotId, SlotIndex);
    if (!Inserted.second)
    {
      Inserted.first->second = std::numeric_limits<std::size_t>::max();
    }
  }

  auto ReadString = [&Strings, StringSize](std::uint32_t Offset,
                                           std::uint32_t Length,
                                           std::string *Output) -> bool
  {
    if (Output == nullptr || !AddWithin(Offset, Length, StringSize))
    {
      return false;
    }
    Output->clear();
    if (Length != 0)
    {
      Output->assign(reinterpret_cast<const char *>(Strings.data() + Offset),
                     Length);
    }
    return true;
  };

  for (std::uint32_t Index = 0; Index < SourceCount; ++Index)
  {
    Record.fill(0);
    if (!ReadAt(Record.data(), Record.size(),
                SourceOffset +
                    static_cast<std::uint64_t>(Index) * SourceStride))
    {
      continue;
    }
    SourceKey Key{};
    if (!ReadString(LoadU32(Record.data()), LoadU32(Record.data() + 4U),
                    &Key.Origin) ||
        !ReadString(LoadU32(Record.data() + 8U), LoadU32(Record.data() + 12U),
                    &Key.BundleId) ||
        !ReadString(LoadU32(Record.data() + 16U), LoadU32(Record.data() + 20U),
                    &Key.SerializedFile))
    {
      continue;
    }
    Key.PathId = LoadU64(Record.data() + 24U);
    const std::uint32_t SlotId = LoadU32(Record.data() + 32U);
    const auto Inserted = SourceIndex_.emplace(std::move(Key), SlotId);
    if (!Inserted.second && Inserted.first->second != SlotId)
    {
      Inserted.first->second = KInvalidGeometrySlot;
    }
  }

  for (std::uint32_t Index = 0; Index < PreloadCount; ++Index)
  {
    Record.fill(0);
    if (!ReadAt(Record.data(), KPackPreloadStride,
                PreloadOffset +
                    static_cast<std::uint64_t>(Index) * PreloadStride))
    {
      continue;
    }
    PreloadKey Key{};
    if (!ReadString(LoadU32(Record.data()), LoadU32(Record.data() + 4U),
                    &Key.BundleId))
    {
      continue;
    }
    Key.Ordinal = LoadU32(Record.data() + 8U);
    const std::uint32_t SlotId = LoadU32(Record.data() + 12U);
    const std::uint64_t ExpectedCount = LoadU64(Record.data() + 16U);
    const auto CountEntry =
        BundlePreloadCounts_.emplace(Key.BundleId, ExpectedCount);
    if (!CountEntry.second && CountEntry.first->second != ExpectedCount)
    {
      CountEntry.first->second = UINT64_MAX;
    }
    if (Key.Ordinal >= ExpectedCount)
    {
      continue;
    }
    const auto Inserted = PreloadIndex_.emplace(std::move(Key), SlotId);
    if (!Inserted.second && Inserted.first->second != SlotId)
    {
      Inserted.first->second = KInvalidGeometrySlot;
    }
  }

  NumericBundlePreloads_.reserve(BundlePreloadCounts_.size());
  for (const auto &Bundle : BundlePreloadCounts_)
  {
    std::uint64_t BundleId = 0;
    if (!ParseBundleId(Bundle.first, BundleId))
    {
      continue;
    }
    auto &Directory = NumericBundlePreloads_[BundleId];
    Directory.ExpectedCount = Bundle.second;
    Directory.CountDefined = Bundle.second != UINT64_MAX;
  }
  for (const auto &Preload : PreloadIndex_)
  {
    std::uint64_t BundleId = 0;
    if (!ParseBundleId(Preload.first.BundleId, BundleId))
    {
      continue;
    }
    const auto Bundle = NumericBundlePreloads_.find(BundleId);
    if (Bundle == NumericBundlePreloads_.end())
    {
      continue;
    }
    std::uint32_t GeometrySlot = Preload.second;
    const auto Slot = SlotIndex_.find(GeometrySlot);
    if (Slot == SlotIndex_.end() || Slot->second >= Slots_.size())
    {
      GeometrySlot = KInvalidGeometrySlot;
    }
    Bundle->second.Entries.push_back({Preload.first.Ordinal, GeometrySlot});
  }
  for (auto &Bundle : NumericBundlePreloads_)
  {
    auto &Entries = Bundle.second.Entries;
    std::sort(Entries.begin(), Entries.end(),
              [](const BundlePreloadEntry &Left,
                 const BundlePreloadEntry &Right)
              { return Left.Ordinal < Right.Ordinal; });
  }
  return true;
}

bool PackReader::LookupSource(
    const SourceKey &Key, std::uint32_t *GeometrySlot) noexcept
{
  if (GeometrySlot != nullptr)
  {
    *GeometrySlot = KInvalidGeometrySlot;
  }
  if (!IsOpen() || GeometrySlot == nullptr)
  {
    return false;
  }
  try
  {
    const auto It = SourceIndex_.find(Key);
    if (It == SourceIndex_.end() || It->second == KInvalidGeometrySlot)
    {
      return false;
    }
    const auto SlotIt = SlotIndex_.find(It->second);
    if (SlotIt == SlotIndex_.end() || SlotIt->second >= Slots_.size())
    {
      return false;
    }
    *GeometrySlot = It->second;
    return true;
  }
  catch (...)
  {
    return false;
  }
}

PreloadStatus
PackReader::LookupPreload(const std::string &BundleId,
                                        std::uint32_t Ordinal,
                                        std::uint32_t *GeometrySlot) noexcept
{
  if (GeometrySlot != nullptr)
  {
    *GeometrySlot = KInvalidGeometrySlot;
  }
  PHYSX_PACK_STAT(++Stats_.PreloadLookups;)
  if (!IsOpen() || GeometrySlot == nullptr)
  {
    return PreloadStatus::Unresolved;
  }
  try
  {
    const PreloadKey Key{BundleId, Ordinal};
    const auto It = PreloadIndex_.find(Key);
    if (It == PreloadIndex_.end())
    {
      return PreloadStatus::Missing;
    }
    if (It->second == KInvalidGeometrySlot)
    {
      return PreloadStatus::Unresolved;
    }
    const auto SlotIt = SlotIndex_.find(It->second);
    if (SlotIt == SlotIndex_.end() || SlotIt->second >= Slots_.size())
    {
      return PreloadStatus::Unresolved;
    }
    *GeometrySlot = It->second;
    return PreloadStatus::Bound;
  }
  catch (...)
  {
    return PreloadStatus::Unresolved;
  }
}

bool PackReader::GetGeometryKind(
    std::uint32_t GeometrySlot,
    ::physx_pack::detail::GeometryKind *Output) const noexcept
{
  if (Output != nullptr)
  {
    *Output = GeometryKind::Unknown;
  }
  if (!IsOpen() || Output == nullptr)
  {
    return false;
  }
  const auto It = SlotIndex_.find(GeometrySlot);
  if (It == SlotIndex_.end() || It->second >= Slots_.size())
  {
    return false;
  }
  *Output = Slots_[It->second].Kind;
  return true;
}

bool PackReader::BundlePreloadCount(
    const std::string &BundleId, std::uint64_t *Output) const noexcept
{
  if (Output != nullptr)
  {
    *Output = 0;
  }
  if (!IsOpen() || Output == nullptr)
  {
    return false;
  }
  try
  {
    const auto It = BundlePreloadCounts_.find(BundleId);
    if (It == BundlePreloadCounts_.end() || It->second == UINT64_MAX)
    {
      return false;
    }
    *Output = It->second;
    return true;
  }
  catch (...)
  {
    return false;
  }
}

const BundlePreloadDirectory *
PackReader::GetBundlePreloadDirectory(
    std::uint64_t BundleId) const noexcept
{
  if (!IsOpen())
  {
    return nullptr;
  }
  const auto Found = NumericBundlePreloads_.find(BundleId);
  return Found == NumericBundlePreloads_.end() ? nullptr : &Found->second;
}

bool PackReader::ReserveCacheSpace(std::size_t Size) noexcept
{
  if (Size > CacheBudget_)
  {
    return false;
  }
  auto It = CacheLru_.begin();
  while (CachedBytes_ > CacheBudget_ - Size && It != CacheLru_.end())
  {
    const auto Entry = Cache_.find(*It);
    if (Entry == Cache_.end())
    {
      It = CacheLru_.erase(It);
      continue;
    }
    if (Entry->second.Data.use_count() > 1)
    {
      ++It;
      continue;
    }
    CachedBytes_ -= Entry->second.Bytes;
    Cache_.erase(Entry);
    It = CacheLru_.erase(It);
    PHYSX_PACK_STAT(++Stats_.CacheEvictions;)
  }
  return CachedBytes_ <= CacheBudget_ - Size;
}

bool PackReader::AcquireBlock(std::uint32_t GeometrySlot,
                                            BlockView *Output) noexcept
{
  if (Output != nullptr)
  {
    Output->Storage.reset();
  }
  if (!IsOpen() || Output == nullptr)
  {
    return false;
  }
  try
  {
    const auto SlotIt = SlotIndex_.find(GeometrySlot);
    if (SlotIt == SlotIndex_.end() ||
        SlotIt->second == std::numeric_limits<std::size_t>::max() ||
        SlotIt->second >= Slots_.size())
    {
      return false;
    }
    const auto CacheIt = Cache_.find(GeometrySlot);
    if (CacheIt != Cache_.end())
    {
      CacheLru_.splice(CacheLru_.end(), CacheLru_, CacheIt->second.Lru);
      CacheIt->second.Lru = std::prev(CacheLru_.end());
      Output->Storage = CacheIt->second.Data;
      return true;
    }
    PHYSX_PACK_STAT(++Stats_.CacheMisses;)
    const Slot &SlotValue = Slots_[SlotIt->second];
    if (!SlotValue.Defined ||
        !RangeWithinFile(SlotValue.Offset, SlotValue.CompressedSize))
    {
      return false;
    }
    if (!ReserveCacheSpace(static_cast<std::size_t>(SlotValue.RawSize)))
    {
      return false;
    }
    std::vector<std::uint8_t> Encoded(
        static_cast<std::size_t>(SlotValue.CompressedSize));
    if (!Encoded.empty() &&
        !ReadAt(Encoded.data(), Encoded.size(), SlotValue.Offset))
    {
      return false;
    }
    PHYSX_PACK_STAT(Stats_.BytesRead += SlotValue.CompressedSize;)
    auto Decoded = std::make_shared<std::vector<std::uint8_t>>(
        static_cast<std::size_t>(SlotValue.RawSize));
    if (SlotValue.Codec == KCodecRaw)
    {
      if (SlotValue.CompressedSize != SlotValue.RawSize)
      {
        return false;
      }
      if (!Encoded.empty())
      {
        std::copy(Encoded.begin(), Encoded.end(), Decoded->begin());
      }
    }
    else if (SlotValue.Codec == KCodecZlib)
    {
      uLongf DecodedSize = static_cast<uLongf>(SlotValue.RawSize);
      const int Result =
          uncompress(Decoded->empty() ? nullptr : Decoded->data(), &DecodedSize,
                     Encoded.empty() ? nullptr : Encoded.data(),
                     static_cast<uLong>(Encoded.size()));
      if (Result != Z_OK || DecodedSize != SlotValue.RawSize)
      {
        return false;
      }
    }
    else
    {
      return false;
    }
    PHYSX_PACK_STAT(Stats_.BytesDecoded += SlotValue.RawSize;)
    std::shared_ptr<const std::vector<std::uint8_t>> Immutable = Decoded;
    CacheLru_.push_back(GeometrySlot);
    try
    {
      CacheEntry Entry{Immutable, Decoded->size(), std::prev(CacheLru_.end())};
      Cache_.emplace(GeometrySlot, std::move(Entry));
    }
    catch (...)
    {
      CacheLru_.pop_back();
      throw;
    }
    CachedBytes_ += Decoded->size();
    Output->Storage = std::move(Immutable);
    return true;
  }
  catch (...)
  {
    Output->Storage.reset();
    return false;
  }
}

} // namespace detail
} // namespace physx_pack
