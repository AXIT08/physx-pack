#include "physx_pack/visibility_query.h"
#include <cerrno>
#include <cstdlib>
#include <iostream>

int main(int argc, char **argv)
{
  if (argc != 6)
  {
    std::cerr << "usage: physx_visibility_example pack origin bundle serialized-file path-id\n";
    return 2;
  }
  char *End=nullptr; errno=0;
  const auto PathId=std::strtoull(argv[5],&End,0);
  if(errno==ERANGE || End==argv[5] || *End!='\0')
  {
    std::cerr << "invalid path-id\n";
    return 2;
  }
  physx_pack::Error Failure{};
  auto Package = physx_pack::Package::Open(argv[1], {}, &Failure);
  if (!Package)
  {
    std::cerr << physx_pack::ToString(Failure) << '\n';
    return 1;
  }
  physx_pack::Scene Scene(Package);
  physx_pack::InstanceDesc Target;
  Target.Id = 1;
  Target.Resource = {argv[2], argv[3], argv[4], PathId};
  Target.DoubleSided = true;
  Failure = Scene.ApplyUpdates(&Target, 1);
  if (Failure != physx_pack::Error::None)
  {
    std::cerr << physx_pack::ToString(Failure) << '\n';
    return 1;
  }
  physx_pack::QueryContext Worker;
  const auto Result = Worker.Check(Scene.GetSnapshot(), {Target.Id, {{0, 100, 0}, {0, -1, 0}, 200}});
  std::cout << physx_pack::ToString(Result.State) << ": " << physx_pack::ToString(Result.Reason) << '\n';
  return Result.State == physx_pack::VisibilityState::Unknown ? 1 : 0;
}
