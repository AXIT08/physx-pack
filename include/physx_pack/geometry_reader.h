#ifndef PHYSX_PACK_GEOMETRY_READER_H
#define PHYSX_PACK_GEOMETRY_READER_H

#include "physx_pack/pack_reader.h"
#include "physx_pack/ray_query.h"

#include <cstdint>
#include <memory>

namespace physx_pack
{
namespace detail
{

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

  // 回调依赖本对象和 Package 的生命周期；重新绑定会撤销之前的视图。
  bool Bind(PackReader &Package, std::uint32_t GeometrySlot,visibility::GeometryType Requested,visibility::GeometryView *Output,void *ContinuationContext = nullptr,bool (*Continue)(void *) = nullptr) noexcept;

private:
  struct State;
  std::unique_ptr<State> State_;
};

} // namespace detail
} // namespace physx_pack

#endif // PHYSX_PACK_GEOMETRY_READER_H
