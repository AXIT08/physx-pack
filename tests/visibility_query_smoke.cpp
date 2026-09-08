#include "physx_pack/visibility_query.h"

#include <cassert>

int main()
{
  physx_pack::VisibilityQuery Query;
  assert(!Query.IsOpen());
  assert(!Query.Open("/definitely/missing/physx.pack"));
  assert(!Query.IsOpen());

  physx_pack::TargetInstance Target{};
  physx_pack::SceneSnapshot Scene{};
  physx_pack::Ray Ray{};
  assert(Query.Check(Target, Scene, Ray) ==
         physx_pack::VisibilityState::Unknown);
  return 0;
}
