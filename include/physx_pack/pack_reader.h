#ifndef PHYSX_PACK_PACK_READER_H
#define PHYSX_PACK_PACK_READER_H

#include "physx_pack/visibility_diagnostics.h"

#include <cstddef>
#include <cstdint>
#include <list>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace physx_pack
{
namespace detail
{

inline constexpr const char *KPackPackPath =
    "/data/adb/physx/physx.pack";
inline constexpr std::size_t KPackCacheBudget =
    128U * 1024U * 1024U;

// 小端文件协议：8 字节 magic 后为 header 字节数和三个保留字段。
// 24/40/56 为槽、来源、preload 表的 offset/count/stride；72 为数据
// offset/size，88 为 UTF-8 字符串池 offset/size。槽号不要求连续。
inline constexpr std::size_t KPackHeaderSize = 104U;
inline constexpr std::size_t KPackSlotStride = 40U;
inline constexpr std::size_t KPackSourceStride = 40U;
inline constexpr std::size_t KPackPreloadStride = 24U;

enum class GeometryKind : std::uint32_t
{
  Unknown = 0,
  TriangleMesh = 1,
  ConvexMesh = 2,
  HeightField = 3,
  Box = 4,
  Capsule = 5,
  Sphere = 6,
  RawData = 7,
  HitBox = 8,
};

struct SourceKey
{
  std::string Origin;
  std::string BundleId;
  std::string SerializedFile;
  std::uint64_t PathId = 0;

  bool operator==(const SourceKey &Other) const noexcept
  {
    return PathId == Other.PathId && Origin == Other.Origin &&
           BundleId == Other.BundleId && SerializedFile == Other.SerializedFile;
  }
};

enum class PreloadStatus : std::uint8_t
{
  Missing,
  Bound,
  Unresolved,
};

struct BundlePreloadEntry
{
  std::uint32_t Ordinal = 0;
  std::uint32_t GeometrySlot = UINT32_MAX;
};

struct BundlePreloadDirectory
{
  std::uint64_t ExpectedCount = 0;
  bool CountDefined = false;
  std::vector<BundlePreloadEntry> Entries;
};

struct BlockView
{
  std::shared_ptr<const std::vector<std::uint8_t>> Storage;

  const std::uint8_t *Data() const noexcept
  {
    return Storage == nullptr || Storage->empty() ? nullptr : Storage->data();
  }

  std::size_t Size() const noexcept
  {
    return Storage == nullptr ? 0U : Storage->size();
  }

  explicit operator bool() const noexcept { return Storage != nullptr; }
};

#if PHYSX_PACK_DIAGNOSTICS
struct PackReaderStats
{
  std::uint64_t CacheMisses = 0;
  std::uint64_t BytesRead = 0;
  std::uint64_t BytesDecoded = 0;
  std::uint64_t PreloadLookups = 0;
  std::uint64_t CacheEvictions = 0;
  std::size_t ResidentBytes = 0;
};
#endif

class PackReader final
{
public:
  PackReader() noexcept = default;
  explicit PackReader(std::size_t CacheBudget) noexcept;
  PackReader(const PackReader &) = delete;
  PackReader &
  operator=(const PackReader &) = delete;
  PackReader(PackReader &&Other) noexcept;
  PackReader &
  operator=(PackReader &&Other) noexcept;
  ~PackReader() noexcept;

  bool Open() noexcept;
  bool Open(const char *PackPath) noexcept;
  void Close() noexcept;

  bool IsOpen() const noexcept { return PackFd_ >= 0; }

  bool LookupSource(const SourceKey &Key,
                    std::uint32_t *GeometrySlot) noexcept;
  PreloadStatus LookupPreload(const std::string &BundleId,
                                      std::uint32_t Ordinal,
                                      std::uint32_t *GeometrySlot) noexcept;
  bool BundlePreloadCount(const std::string &BundleId,
                          std::uint64_t *Output) const noexcept;
  const struct BundlePreloadDirectory *
  GetBundlePreloadDirectory(std::uint64_t BundleId) const noexcept;
  bool GetGeometryKind(
      std::uint32_t GeometrySlot,
      ::physx_pack::detail::GeometryKind *Output) const noexcept;

  // 外置不可变数据允许跨拍共享；持有者释放块后才允许 LRU 回收其预算。
  bool AcquireBlock(std::uint32_t GeometrySlot,
                    BlockView *Output) noexcept;

#if PHYSX_PACK_DIAGNOSTICS
  PackReaderStats Stats() const noexcept
  {
    PackReaderStats Result = Stats_;
    Result.ResidentBytes = CachedBytes_;
    return Result;
  }
#endif

  static constexpr std::uint32_t KInvalidGeometrySlot = UINT32_MAX;

private:
  struct SourceKeyHash
  {
    std::size_t operator()(const SourceKey &Key) const noexcept;
  };

  struct PreloadKey
  {
    std::string BundleId;
    std::uint32_t Ordinal = 0;

    bool operator==(const PreloadKey &Other) const noexcept
    {
      return Ordinal == Other.Ordinal && BundleId == Other.BundleId;
    }
  };

  struct PreloadKeyHash
  {
    std::size_t operator()(const PreloadKey &Key) const noexcept;
  };

  struct Slot
  {
    ::physx_pack::detail::GeometryKind Kind =
        ::physx_pack::detail::GeometryKind::Unknown;
    std::uint32_t Codec = 0;
    std::uint64_t Offset = 0;
    std::uint64_t CompressedSize = 0;
    std::uint64_t RawSize = 0;
    bool Defined = false;
  };

  struct CacheEntry
  {
    std::shared_ptr<const std::vector<std::uint8_t>> Data;
    std::size_t Bytes = 0;
    std::list<std::uint32_t>::iterator Lru;
  };

  bool OpenPack(const char *Path) noexcept;
  bool ReadPackIndex();
  bool ReadAt(void *Buffer, std::size_t Size,std::uint64_t Offset) const noexcept;
  bool RangeWithinFile(std::uint64_t Offset, std::uint64_t Size) const noexcept;
  bool ReserveCacheSpace(std::size_t Size) noexcept;
  void ResetIndexes() noexcept;

  int PackFd_ = -1;
  std::uint64_t PackSize_ = 0;
  std::vector<Slot> Slots_;
  std::unordered_map<std::uint32_t, std::size_t> SlotIndex_;
  std::unordered_map<SourceKey, std::uint32_t, SourceKeyHash>SourceIndex_;
  std::unordered_map<PreloadKey, std::uint32_t, PreloadKeyHash> PreloadIndex_;
  std::unordered_map<std::string, std::uint64_t> BundlePreloadCounts_;
  std::unordered_map<std::uint64_t,
                     ::physx_pack::detail::BundlePreloadDirectory>
      NumericBundlePreloads_;
  std::unordered_map<std::uint32_t, CacheEntry> Cache_;
  std::list<std::uint32_t> CacheLru_;
  std::size_t CachedBytes_ = 0;
  std::size_t CacheBudget_ = KPackCacheBudget;
#if PHYSX_PACK_DIAGNOSTICS
  PackReaderStats Stats_{};
#endif
};

} // namespace detail
} // namespace physx_pack

#endif // PHYSX_PACK_PACK_READER_H
