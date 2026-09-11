#include "xc_read_frame_adapter.h"
#include <cstdlib>
#include <iostream>
#include <string>
int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cerr << "usage: physx_read_frame_example physx.pack [--sequence N --plan-revision N]\n";
    return 2;
  }
  physx_pack::Error e = physx_pack::Error::None;
  auto package = physx_pack::Package::Open(argv[1], {}, &e);
  if (!package)
  {
    std::cerr << physx_pack::ToString(e) << '\n';
    return 1;
  }
  using namespace physx_pack::xc_read;
  ReadFramePhysxAdapter adapter(package);
  ReadFrameInput in;
  in.Sequence = 1;
  in.PlanRevision = 1;
  in.Camera = {0, 0, 10};
  in.Bones[0] = {0, 0, 0};
  in.ActiveMask = 1;
  in.Target = {1, 1, 1};
  for (int i = 2; i + 1 < argc; ++i)
  {
    const std::string option = argv[i];
    if (option == "--sequence")
      in.Sequence = std::strtoull(argv[++i], nullptr, 10);
    else if (option == "--plan-revision")
      in.PlanRevision = std::strtoull(argv[++i], nullptr, 10);
  }
  e = adapter.BeginFrame(in);
  if (e != physx_pack::Error::None)
    return 1;
  e = adapter.EndFrame();
  if (e != physx_pack::Error::None)
    return 1;
  auto out = adapter.Resolve();
  std::cout << "sequence=" << out.Sequence << " plan_revision=" << out.PlanRevision
            << " completed_mask=" << out.CompletedMask << '\n';
  return 0;
}
