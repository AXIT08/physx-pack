#include "test_fixture.h"
#include "xc_read_frame_adapter.h"
#include <array>
using namespace physx_pack;
using namespace physx_pack::xc_read;
using namespace test;
int main()
{
  auto fixture = StandardPack();
  TempPack file(fixture);
  Error failure = Error::None;
  auto package = Package::Open(file.Path(), {}, &failure);
  CHECK(package && failure == Error::None);
  ReadFramePhysxAdapter adapter(package);
  ReadFrameInput input{};
  input.Sequence = 1;
  input.PlanRevision = 7;
  input.Camera = {0, 0, 10};
  input.Bones[0] = {0, 0, 0};
  input.Bones[1] = {0, 0, 5};
  input.ActiveMask = 3;
  input.Target = {1, 42, 9001};
  std::array<ReadFrameShape, 2> shapes{};
  shapes[0].ConsumerKey = input.Target;
  shapes[0].Resource = Key(1);
  shapes[0].DoubleSided = true;
  shapes[1].ConsumerKey = {2, 99, 9002};
  shapes[1].Resource = Key(1);
  shapes[1].WorldPose.Position.Z = 5;
  shapes[1].DoubleSided = true;
  input.Shapes = shapes.data();
  input.ShapeCount = shapes.size();
  CHECK(adapter.BeginFrame(input) == Error::None);
  CHECK(adapter.EndFrame() == Error::None);
  auto result = adapter.Resolve();
  CHECK(result.Sequence == 1 && result.PlanRevision == 7);
  CHECK((result.CompletedMask & 3U) == 3U);
  CHECK(result.Bones[0].State == VisibilityState::Blocked);
  CHECK(result.Bones[0].Hit == shapes[1].ConsumerKey);
  CHECK(adapter.BeginFrame(1, 7, input.Camera, input.Bones, input.ActiveMask, input.Target) ==
        Error::InvalidArgument);
  CHECK(adapter.BeginFrame(2, 7, input.Camera, input.Bones, input.ActiveMask, input.Target) ==
        Error::None);
  CHECK(adapter.UpsertShape(shapes[0]) == Error::None);
  CHECK(adapter.UpsertShape(shapes[0]) == Error::InvalidArgument);
  CHECK(adapter.EndFrame() == Error::InvalidArgument);
  ReadFrameInput bad = input;
  bad.Sequence = 3;
  bad.Shapes = nullptr;
  bad.ShapeCount = 1;
  CHECK(adapter.BeginFrame(bad) == Error::InvalidArgument);
  QueryContext query;
  Scene scene(package);
  InstanceDesc blocker{};
  blocker.Id = 77;
  blocker.Resource = Key(1);
  blocker.DoubleSided = true;
  CHECK(scene.ApplyUpdates(&blocker, 1) == Error::None);
  std::array<RaycastRequest, 2> requests{
      {{{{0, 0, 10}, {0, 0, -1}, 20}, 0}, {{{0, 20, 10}, {0, 0, -1}, 20}, 0}}};
  std::array<RaycastResult, 2> traces{};
  CHECK(query.TraceBatch(scene.GetSnapshot(), requests.data(), requests.size(), traces.data()) ==
        Error::None);
  CHECK(traces[0].State == RaycastState::Hit && traces[0].HitInstance == 77);
  CHECK(traces[1].State == RaycastState::Miss);
  return 0;
}
