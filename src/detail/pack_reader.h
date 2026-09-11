#ifndef PHYSX_PACK_PACK_READER_H
#define PHYSX_PACK_PACK_READER_H

#include "physx_pack/visibility_query.h"
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace physx_pack
{
namespace detail
{
inline constexpr const char *KPackPackPath = "/data/adb/physx/physx.pack";
inline constexpr std::size_t KPackCacheBudget = 128U * 1024U * 1024U;
inline constexpr std::size_t KPackHeaderSize = 104U;
inline constexpr std::size_t KPackSlotStride = 40U;
inline constexpr std::size_t KPackSourceStride = 40U;
inline constexpr std::size_t KPackPreloadStride = 24U;

enum class GeometryKind : std::uint32_t
{
  Unknown = 0, TriangleMesh = 1, ConvexMesh = 2, HeightField = 3,
  Box = 4, Capsule = 5, Sphere = 6, RawData = 7, HitBox = 8,
};
struct SourceKey
{
  std::string Origin, BundleId, SerializedFile;
  std::uint64_t PathId = 0;
  bool operator==(const SourceKey &Other) const noexcept
  {
    return PathId == Other.PathId && Origin == Other.Origin &&
           BundleId == Other.BundleId && SerializedFile == Other.SerializedFile;
  }
};
enum class PreloadStatus : std::uint8_t { Missing, Bound, Unresolved };
struct BundlePreloadEntry
{
  std::uint32_t Ordinal = 0;
  std::uint32_t GeometrySlot = UINT32_MAX;
};
class BundlePreloadEntries final
{
public:
  BundlePreloadEntries() noexcept = default;
  BundlePreloadEntries(const BundlePreloadEntry *Data, std::size_t Count) noexcept
      : Data_(Count == 0 ? &Empty_ : Data), Count_(Count) {}
  const BundlePreloadEntry *begin() const noexcept { return Data_; }
  const BundlePreloadEntry *end() const noexcept { return Data_ + Count_; }
  const BundlePreloadEntry *data() const noexcept { return Data_; }
  std::size_t size() const noexcept { return Count_; }
  bool empty() const noexcept { return Count_ == 0; }
  const BundlePreloadEntry &operator[](std::size_t Index) const noexcept { return Data_[Index]; }
private:
  inline static constexpr BundlePreloadEntry Empty_{};
  const BundlePreloadEntry *Data_ = &Empty_;
  std::size_t Count_ = 0;
};
struct BundlePreloadDirectory
{
  std::uint64_t ExpectedCount = 0;
  bool CountDefined = false;
  BundlePreloadEntries Entries;
};
struct BlockView
{
  std::shared_ptr<const std::vector<std::uint8_t>> Storage;
  const std::uint8_t *Data() const noexcept
  {
    return Storage == nullptr || Storage->empty() ? nullptr : Storage->data();
  }
  std::size_t Size() const noexcept { return Storage == nullptr ? 0U : Storage->size(); }
  explicit operator bool() const noexcept { return Storage != nullptr; }
};
class PackReader final
{
public:
  PackReader() noexcept = default;
  explicit PackReader(std::size_t CacheBudget) noexcept;
  PackReader(const PackReader &) = delete;
  PackReader &operator=(const PackReader &) = delete;
  PackReader(PackReader &&Other) noexcept;
  PackReader &operator=(PackReader &&Other) noexcept;
  ~PackReader() noexcept;
  // Open/Close/move require exclusive ownership. Opened indexes are immutable;
  // all cache operations and queries support concurrent readers.
  bool Open() noexcept;
  bool Open(const char *PackPath) noexcept;
  void Close() noexcept;
  bool IsOpen() const noexcept { return PackFd_ >= 0; }
  Error OpenError() const noexcept { return OpenError_; }
  PackageInfo Info() const noexcept { return Info_; }
  CacheStats Stats() const noexcept;
  Error ResolveSource(const SourceKey &Key, std::uint32_t *GeometrySlot) const noexcept;
  bool LookupSource(const SourceKey &Key, std::uint32_t *GeometrySlot) const noexcept;
  PreloadStatus LookupPreload(const std::string &BundleId, std::uint32_t Ordinal,
                             std::uint32_t *GeometrySlot) const noexcept;
  bool BundlePreloadCount(const std::string &BundleId, std::uint64_t *Output) const noexcept;
  const BundlePreloadDirectory *GetBundlePreloadDirectory(std::uint64_t BundleId) const noexcept;
  bool GetGeometryKind(std::uint32_t GeometrySlot, GeometryKind *Output) const noexcept;
  bool AcquireBlock(std::uint32_t GeometrySlot, BlockView *Output, Error *Failure = nullptr) noexcept;
  static constexpr std::uint32_t KInvalidGeometrySlot = UINT32_MAX;
private:
  struct SourceView
  {
    std::string_view Origin, BundleId, SerializedFile;
    std::uint64_t PathId = 0;
    bool operator<(const SourceView &Other) const noexcept;
    bool operator==(const SourceView &Other) const noexcept;
  };
  struct SourceRecord
  {
    SourceView Key;
    std::uint32_t GeometrySlot = UINT32_MAX;
    Error State = Error::None;
  };
  struct Slot
  {
    std::uint32_t Id = UINT32_MAX;
    GeometryKind Kind = GeometryKind::Unknown;
    std::uint32_t Codec = 0;
    Error State = Error::None;
    std::uint64_t Offset = 0, CompressedSize = 0, RawSize = 0;
  };
  struct BundleRecord
  {
    std::string_view Name;
    std::uint64_t NumericId = 0;
    bool Numeric = false;
    BundlePreloadDirectory Directory;
  };
  struct CacheEntry
  {
    std::shared_ptr<const std::vector<std::uint8_t>> Data;
    std::list<std::uint32_t>::iterator Lru;
  };
  struct DecodedBudget
  {
    std::atomic<std::uint64_t> Resident{0}, Peak{0};
  };
  bool OpenPack(const char *Path) noexcept;
  Error ReadPackIndex();
  bool ReadAt(void *Buffer, std::size_t Size, std::uint64_t Offset) const noexcept;
  bool RangeWithinFile(std::uint64_t Offset, std::uint64_t Size) const noexcept;
  const Slot *FindSlot(std::uint32_t Id) const noexcept;
  const BundleRecord *FindBundle(std::string_view Name) const noexcept;
  bool ReserveCacheSpace(std::size_t Size) noexcept;
  Error DecodeBlock(const Slot &Value, std::vector<std::uint8_t> &Output);
  void ResetIndexes() noexcept;
  int PackFd_ = -1;
  std::uint64_t PackSize_ = 0;
  Error OpenError_ = Error::NotOpen;
  PackageInfo Info_{};
  std::vector<char> Strings_;
  std::vector<Slot> Slots_;
  std::vector<SourceRecord> Sources_;
  std::vector<BundleRecord> Bundles_;
  std::vector<std::uint32_t> NumericBundles_, NamedBundles_;
  std::vector<BundlePreloadEntry> Preloads_;
  mutable std::mutex CacheMutex_;
  std::unordered_map<std::uint32_t, CacheEntry> Cache_;
  std::list<std::uint32_t> CacheLru_;
  std::shared_ptr<DecodedBudget> Budget_;
  CacheStats Stats_{};
  std::size_t CacheBudget_ = KPackCacheBudget;
};
} // namespace detail
} // namespace physx_pack
#endif
