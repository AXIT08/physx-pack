#include "physx_pack/visibility_query.h"
#include <type_traits>

static_assert(physx_pack::ApiVersion == 2);
static_assert(std::is_copy_constructible_v<physx_pack::SceneSnapshot>);
static_assert(!std::is_copy_constructible_v<physx_pack::QueryContext>);
static_assert(!std::is_copy_constructible_v<physx_pack::Scene>);
static_assert(!std::is_copy_constructible_v<physx_pack::Package>);

int main()
{
  physx_pack::QueryContext Context;
  const physx_pack::SceneSnapshot Empty;
  const auto Result = Context.Check(Empty, {});
  physx_pack::RaycastResult Trace{};
  const auto TraceError = Context.TraceBatch(Empty, nullptr, 0, nullptr);
  return Result.State == physx_pack::VisibilityState::Unknown &&
                 Trace.State == physx_pack::RaycastState::Unknown &&
                 TraceError == physx_pack::Error::None
             ? 0
             : 1;
}
