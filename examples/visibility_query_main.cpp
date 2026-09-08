#include "physx_pack/visibility_query.h"

#include <cstdlib>
#include <iostream>

int main(int argc, char **argv)
{
  physx_pack::VisibilityQuery Query;
  if (!(argc > 1 ? Query.Open(argv[1]) : Query.Open()))
  {
    std::cerr << "physx.pack could not be opened\n";
    return EXIT_FAILURE;
  }

  physx_pack::TargetInstance Target{};
  Target.Id.Origin = "cn";
  Target.Id.BundleId = "example.bundle";
  Target.Id.SerializedFile = "CAB-example";
  Target.Id.PathId = 1U;

  physx_pack::SceneSnapshot Scene{};
  Scene.Instances.push_back(Target);

  physx_pack::Ray Ray{};
  Ray.Origin = {0.0f, 100.0f, 0.0f};
  Ray.Direction = {0.0f, -1.0f, 0.0f};
  Ray.MaxDistance = 200.0f;

  switch (Query.Check(Target, Scene, Ray))
  {
  case physx_pack::VisibilityState::Visible:
    std::cout << "visible\n";
    break;
  case physx_pack::VisibilityState::Blocked:
    std::cout << "blocked\n";
    break;
  case physx_pack::VisibilityState::Unknown:
    std::cout << "unknown\n";
    break;
  }
  return EXIT_SUCCESS;
}
