#ifndef PHYSX_PACK_GEOMETRY_READER_H
#define PHYSX_PACK_GEOMETRY_READER_H

#include "detail/pack_reader.h"
#include "detail/ray_query.h"
#include "physx_pack/visibility_query.h"

#include <cstdint>
#include <memory>

namespace physx_pack
{
namespace detail
{

struct GeometryDescription
{
  visibility::Aabb LocalBounds{};
  bool BoundsDefined = false;
};

class GeometryStore;

class GeometryReader final
{
public:
  GeometryReader() noexcept;
  ~GeometryReader() noexcept;
  GeometryReader(const GeometryReader &) = delete;
  GeometryReader &
  operator=(const GeometryReader &) = delete;
  GeometryReader(GeometryReader &&) noexcept;
  GeometryReader &operator=(GeometryReader &&) noexcept;

  // A lease belongs to one query thread. Reset or rebinding invalidates its view.
  void Reset() noexcept;
  Error LastError() const noexcept;

private:
  friend class GeometryStore;
  struct State;
  std::unique_ptr<State> State_;
};

class GeometryStore final
{
public:
  explicit GeometryStore(PackReader &Package) noexcept;
  ~GeometryStore() noexcept;
  GeometryStore(const GeometryStore &) = delete;
  GeometryStore &operator=(const GeometryStore &) = delete;

  // Cached metadata is shared, immutable, and does not retain decoded blocks.
  Error Describe(std::uint32_t Slot, visibility::GeometryType Requested,
                 GeometryDescription *Output) noexcept;
  Error Bind(std::uint32_t Slot, visibility::GeometryType Requested,
             GeometryReader &Lease, visibility::GeometryView *Output) noexcept;
  std::uint64_t MetadataBytes() const noexcept;

private:
  struct State;
  std::unique_ptr<State> State_;
};

} // namespace detail
} // namespace physx_pack

#endif // PHYSX_PACK_GEOMETRY_READER_H
