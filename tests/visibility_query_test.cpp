#include "test_fixture.h"
#include <atomic>
#include <limits>
#include <thread>

namespace
{
using namespace physx_pack;
using namespace test;
const QueryRequest Forward{10, {{0,0,0}, {0,0,1}, 100}};

std::shared_ptr<Package> Open(const TempPack &File, std::size_t Budget = 1024 * 1024)
{
  Error Failure{}; auto P = Package::Open(File.Path(), {Budget}, &Failure);
  CHECK(P); CHECK(Failure == Error::None); return P;
}
void Expect(const VisibilityResult &R, VisibilityState State, Error Reason = Error::None)
{
  if (R.State != State || R.Reason != Reason)
    std::cerr << "result " << ToString(R.State) << '/' << ToString(R.Reason)
              << " target=" << R.TargetDistance << " blocker=" << R.BlockingInstance
              << " problem=" << R.ProblemInstance << '\n';
  CHECK(R.State == State); CHECK(R.Reason == Reason);
}

void InstanceIdentityAndTransactions()
{
  TempPack File(StandardPack()); auto P = Open(File); Scene S(P); QueryContext Q;
  CHECK(S.GetSnapshot()); CHECK(S.GetSnapshot().Generation() == 0);
  std::array<InstanceDesc,2> Items{Instance(10,10), Instance(20,5)};
  CHECK(S.ApplyUpdates(Items.data(),Items.size()) == Error::None);
  const auto Old = S.GetSnapshot(); const auto Generation = Old.Generation();
  auto R = Q.Check(Old,Forward); Expect(R,VisibilityState::Blocked);
  CHECK(R.BlockingInstance == 20); CHECK(Near(R.TargetDistance,10)); CHECK(Near(R.BlockingDistance,5));
  auto Duplicate = Items; Duplicate[1].Id = 10;
  CHECK(S.ApplyUpdates(Duplicate.data(),2) == Error::InvalidArgument);
  auto Invalid = Items; Invalid[0].WorldPose.Position.Z = 30; Invalid[1].Scale.X = 0;
  CHECK(S.ApplyUpdates(Invalid.data(),2) == Error::InvalidArgument);
  CHECK(S.GetSnapshot().Generation() == Generation);
  Expect(Q.Check(S.GetSnapshot(),Forward),VisibilityState::Blocked);
  InstanceId Missing = 999;
  CHECK(S.ApplyUpdates(nullptr,0,&Missing,1) == Error::TargetNotFound);
  CHECK(S.GetSnapshot().Generation() == Generation);
  InstanceId Blocker = 20;
  CHECK(S.ApplyUpdates(nullptr,0,&Blocker,1) == Error::None);
  CHECK(S.GetSnapshot().InstanceCount() == 1);
  Expect(Q.Check(S.GetSnapshot(),Forward),VisibilityState::Visible);
  Expect(Q.Check(Old,Forward),VisibilityState::Blocked);
  CHECK(S.ApplyUpdates(nullptr,0) == Error::None);
  CHECK(S.ApplyUpdates(nullptr,1) == Error::InvalidArgument);
}

void ArgumentsResourcesAndVariants()
{
  TempPack File(StandardPack()); auto P=Open(File); Scene S(P); QueryContext Q;
  auto Target=Instance(10,10,2); CHECK(S.ApplyUpdates(&Target,1)==Error::None);
  Expect(Q.Check(S.GetSnapshot(),{99,Forward.QueryRay}),VisibilityState::Unknown,Error::TargetNotFound);
  Expect(Q.Check({},Forward),VisibilityState::Unknown,Error::NotOpen);
  auto Request=Forward; Request.QueryRay.Direction={0,0,0};
  Expect(Q.Check(S.GetSnapshot(),Request),VisibilityState::Unknown,Error::InvalidArgument);
  Request=Forward; Request.QueryRay.Direction={0,0,7};
  auto R=Q.Check(S.GetSnapshot(),Request); Expect(R,VisibilityState::Visible); CHECK(Near(R.TargetDistance,10));
  Request.QueryRay.MaxDistance=9;
  Expect(Q.Check(S.GetSnapshot(),Request),VisibilityState::Unknown,Error::TargetNotHit);
  Target.Resource=Key(999); CHECK(S.ApplyUpdates(&Target,1)==Error::None);
  R=Q.Check(S.GetSnapshot(),Forward); Expect(R,VisibilityState::Unknown,Error::MissingResource); CHECK(R.ProblemInstance==10);
  Target.Resource=Key(2); Target.MeshType=MeshVariant::Convex; CHECK(S.ApplyUpdates(&Target,1)==Error::None);
  R=Q.Check(S.GetSnapshot(),Forward); Expect(R,VisibilityState::Unknown,Error::MissingVariant); CHECK(R.ProblemInstance==10);
  Target.Resource=Key(3); Target.MeshType=MeshVariant::Automatic; CHECK(S.ApplyUpdates(&Target,1)==Error::None);
  R=Q.Check(S.GetSnapshot(),Forward); Expect(R,VisibilityState::Visible); CHECK(Near(R.TargetDistance,9));
  std::array<QueryRequest,2> Requests{Forward, {10,{{},{},100}}};
  std::array<VisibilityResult,2> Results;
  CHECK(Q.CheckBatch(S.GetSnapshot(),Requests.data(),2,Results.data())==Error::None);
  Expect(Results[0],VisibilityState::Visible); CHECK(Results[1].Reason==Error::InvalidArgument);
  CHECK(Q.CheckBatch(S.GetSnapshot(),nullptr,0,nullptr)==Error::None);
  CHECK(Q.CheckBatch(S.GetSnapshot(),nullptr,1,Results.data())==Error::InvalidArgument);
  CHECK(Q.CheckBatch(S.GetSnapshot(),Requests.data(),1,nullptr)==Error::InvalidArgument);
}

void KnownBlockerWinsUnknown()
{
  TempPack File(StandardPack()); Scene S(Open(File)); QueryContext Q;
  std::array<InstanceDesc,3> Items{Instance(10,10),Instance(20,5),Instance(30,3,999)};
  CHECK(S.ApplyUpdates(Items.data(),Items.size())==Error::None);
  auto R=Q.Check(S.GetSnapshot(),Forward); Expect(R,VisibilityState::Blocked); CHECK(R.BlockingInstance==20);
  InstanceId Blocker=20; CHECK(S.ApplyUpdates(nullptr,0,&Blocker,1)==Error::None);
  R=Q.Check(S.GetSnapshot(),Forward); Expect(R,VisibilityState::Unknown,Error::MissingResource); CHECK(R.ProblemInstance==30);
  Items[2].WorldPose.Position.X=10000;
  CHECK(S.ApplyUpdates(&Items[2],1)==Error::None);
  // Position alone cannot exclude a resource for which no bounds are known.
  Expect(Q.Check(S.GetSnapshot(),Forward),VisibilityState::Unknown,Error::MissingResource);
}

void HeightFieldTransformsAndHoles()
{
  TempPack File(StandardPack()); Scene S(Open(File)); QueryContext Q;
  auto Target=Instance(10,0,4); Target.MeshType=MeshVariant::Automatic;
  Target.Scale={-2,3,-4}; Target.WorldPose.Position={7,11,13};
  CHECK(S.ApplyUpdates(&Target,1)==Error::None);
  QueryRequest Request{10,{{6.5f,30,12},{0,-9,0},100}};
  auto R=Q.Check(S.GetSnapshot(),Request); Expect(R,VisibilityState::Visible); CHECK(Near(R.TargetDistance,13));
  // Negative vertical scale must not invert the broad-phase min/max ordering.
  Target.Scale.Y=-3; CHECK(S.ApplyUpdates(&Target,1)==Error::None);
  R=Q.Check(S.GetSnapshot(),Request); Expect(R,VisibilityState::Visible); CHECK(Near(R.TargetDistance,25));
  Target.Resource=Key(5); CHECK(S.ApplyUpdates(&Target,1)==Error::None);
  Expect(Q.Check(S.GetSnapshot(),Request),VisibilityState::Unknown,Error::TargetNotHit);
  Target.Resource=Key(4); Target.Scale={.5f,.5f,.5f}; Target.WorldPose={};
  CHECK(S.ApplyUpdates(&Target,1)==Error::None);
  R=Q.Check(S.GetSnapshot(),{10,{{.75f,10,.75f},{0,-1,0},100}});
  Expect(R,VisibilityState::Visible); CHECK(Near(R.TargetDistance,9));
}

void BoundsExcludeUnreadableTiles()
{
  auto F=StandardPack(); F.Blocks.push_back({7,3,HeightField(203)});
  F.Blocks.push_back({203,7,HeightTile(),1,true}); F.Sources.push_back({Key(7),7});
  TempPack File(F); Scene S(Open(File)); QueryContext Q;
  std::array<InstanceDesc,2> Items{Instance(10,0,4),Instance(20,0,7)};
  for (auto &I:Items) I.MeshType=MeshVariant::Automatic;
  Items[0].WorldPose.Position.Y=-10; Items[1].WorldPose.Position.X=100;
  CHECK(S.ApplyUpdates(Items.data(),2)==Error::None);
  const QueryRequest Request{10,{{0.5f,30,0.5f},{0,-1,0},100}};
  Expect(Q.Check(S.GetSnapshot(),Request),VisibilityState::Visible);
  Items[1].WorldPose.Position.X=0; CHECK(S.ApplyUpdates(&Items[1],1)==Error::None);
  auto R=Q.Check(S.GetSnapshot(),Request); Expect(R,VisibilityState::Unknown,Error::CorruptData);
  CHECK(R.ProblemInstance==20);
}

void BudgetCorruptionAndRTree()
{
  TempPack File(StandardPack()); auto Small=Open(File,32); Scene S(Small); QueryContext Q;
  auto Target=Instance(10,10); CHECK(S.ApplyUpdates(&Target,1)==Error::None);
  Expect(Q.Check(S.GetSnapshot(),Forward),VisibilityState::Unknown,Error::BudgetExceeded);
  const auto Stats=Small->GetCacheStats(); CHECK(Stats.ResidentBytes<=Stats.BudgetBytes);
  auto F=StandardPack(); for(auto &B:F.Blocks) if(B.Id==101) B.Corrupt=true;
  TempPack Bad(F); Scene Corrupt(Open(Bad)); CHECK(Corrupt.ApplyUpdates(&Target,1)==Error::None);
  Expect(Q.Check(Corrupt.GetSnapshot(),Forward),VisibilityState::Unknown,Error::CorruptData);
  Scene RTree(Open(File)); Target.Resource=Key(6); CHECK(RTree.ApplyUpdates(&Target,1)==Error::None);
  Expect(Q.Check(RTree.GetSnapshot(),Forward),VisibilityState::Visible);
}

void WarmQueriesAndPoseUpdates()
{
  TempPack File(StandardPack()); auto P=Open(File); Scene S(P); QueryContext Q;
  std::array<InstanceDesc,2> Items{Instance(10,10),Instance(20,20)};
  CHECK(S.ApplyUpdates(Items.data(),2)==Error::None);
  Expect(Q.Check(S.GetSnapshot(),Forward),VisibilityState::Visible);
  const auto Before=P->GetCacheStats(); const auto SceneBefore=S.GetStats();
  for(unsigned I=0;I<32;++I) Expect(Q.Check(S.GetSnapshot(),Forward),VisibilityState::Visible);
  CHECK(P->GetCacheStats().GeometryMetadataBytes==Before.GeometryMetadataBytes);
  CHECK(P->GetCacheStats().BytesDecoded==Before.BytesDecoded);
  Items[1].WorldPose.Position.Z=5; CHECK(S.ApplyUpdates(&Items[1],1)==Error::None);
  CHECK(S.GetStats().ResourceResolutions==SceneBefore.ResourceResolutions);
  CHECK(S.GetStats().BvhBuilds==SceneBefore.BvhBuilds);
  CHECK(S.GetStats().BvhRefits>SceneBefore.BvhRefits);
  Expect(Q.Check(S.GetSnapshot(),Forward),VisibilityState::Blocked);
  CHECK(Q.GetStats().Queries>=34); CHECK(P->GetInfo().IndexBytes>0);
}

void SnapshotThreadsAndLifetime()
{
  SceneSnapshot Survivor;
  {
    TempPack File(StandardPack()); auto P=Open(File); Scene S(P);
    std::array<InstanceDesc,2> Items{Instance(10,10),Instance(20,20)};
    CHECK(S.ApplyUpdates(Items.data(),2)==Error::None); Survivor=S.GetSnapshot();
    std::atomic<bool> Failed{false},Start{false}; std::vector<std::thread> Workers;
    for(unsigned Worker=0;Worker<4;++Worker) Workers.emplace_back([&] {
      QueryContext Q;
      while(!Start.load(std::memory_order_acquire)) std::this_thread::yield();
      for(unsigned I=0;I<250;++I)
      {
        const auto Old=Q.Check(Survivor,Forward), Current=Q.Check(S.GetSnapshot(),Forward);
        if(Old.State!=VisibilityState::Visible || !Near(Old.TargetDistance,10) ||
           (Current.State!=VisibilityState::Visible && Current.State!=VisibilityState::Blocked)) Failed=true;
      }
    });
    Start.store(true,std::memory_order_release);
    for(unsigned I=0;I<120;++I)
    { Items[1].WorldPose.Position.Z=(I&1)?5:20; CHECK(S.ApplyUpdates(&Items[1],1)==Error::None); }
    for(auto &Worker:Workers) Worker.join();
    CHECK(!Failed.load());
  }
  QueryContext Q; Expect(Q.Check(Survivor,Forward),VisibilityState::Visible);
}

void TriangleEpsilonAndInvalidIndices()
{
  auto F=StandardPack(); Bytes Triangle(128);
  Put32(Triangle,0,0x544D4358);Put32(Triangle,4,1);Put32(Triangle,8,3);Put32(Triangle,12,1);
  Put32(Triangle,24,4);PutFloat(Triangle,28,0.01f);Put64(Triangle,32,80);Put64(Triangle,40,116);Put64(Triangle,48,128);
  PutFloat(Triangle,92,1);PutFloat(Triangle,108,1);Put32(Triangle,120,1);Put32(Triangle,124,2);
  for(auto &B:F.Blocks) if(B.Id==101) B.Data=Triangle;
  TempPack File(F);Scene S(Open(File));QueryContext Q;auto Target=Instance(10,10);
  CHECK(S.ApplyUpdates(&Target,1)==Error::None);
  Expect(Q.Check(S.GetSnapshot(),{10,{{-.005f,.2f,0},{0,0,1},100}}),VisibilityState::Visible);
  for(auto &B:F.Blocks) if(B.Id==101) Put32(B.Data,124,999);
  TempPack Bad(F);Scene Malformed(Open(Bad));CHECK(Malformed.ApplyUpdates(&Target,1)==Error::None);
  Expect(Q.Check(Malformed.GetSnapshot(),Forward),VisibilityState::Unknown,Error::CorruptData);
}

void LongNearlyParallelRayStillVisitsBlocker()
{
  TempPack File(StandardPack()); Scene S(Open(File)); QueryContext Q;
  std::array<InstanceDesc,2> Items{Instance(10,10000),Instance(20,0)};
  CHECK(S.ApplyUpdates(Items.data(),2)==Error::None);
  QueryRequest Request{20,{{-2,0,-1e10f},{2e-10f,0,1},2e10f}};
  Expect(Q.Check(S.GetSnapshot(),Request),VisibilityState::Visible);
  Request.Target=10;
  const auto Result=Q.Check(S.GetSnapshot(),Request);
  Expect(Result,VisibilityState::Blocked); CHECK(Result.BlockingInstance==20);
  Request={20,{{-2,0,-10},{2e-10f,0,1},100}};
  Expect(Q.Check(S.GetSnapshot(),Request),VisibilityState::Unknown,Error::TargetNotHit);
}

void LargeTransformCancellationKeepsConservativeBounds()
{
  auto F=StandardPack();
  const std::array<float,12> Vertices{1e8f-16,1e8f-16,1e8f,1e8f+16,1e8f-16,1e8f,
                                      1e8f+16,1e8f+16,1e8f,1e8f-16,1e8f+16,1e8f};
  for(auto &B:F.Blocks) if(B.Id==101)
    for(std::size_t I=0;I<Vertices.size();++I) PutFloat(B.Data,80+4*I,Vertices[I]);
  TempPack File(F); Scene S(Open(File)); QueryContext Q;
  auto Target=Instance(10),Blocker=Instance(20);
  Target.WorldPose={{165756336,19058994,-46492612},.0811589583755f,-.499687880278f,.70876044035f,.491308301687f};
  Blocker.WorldPose={{165756384,19059094,-46492676},.0811589583755f,-.499687880278f,.70876044035f,.491308301687f};
  QueryRequest Request{10,{{10.3142557144f,115.160072327f,-68.4552154541f},
                           {-.375957101583f,-.788066148758f,.487450480461f},512}};
  CHECK(S.ApplyUpdates(&Target,1)==Error::None);
  Expect(Q.Check(S.GetSnapshot(),Request),VisibilityState::Visible);
  InstanceId TargetId=10; CHECK(S.ApplyUpdates(&Blocker,1,&TargetId,1)==Error::None);
  Request.Target=20; Expect(Q.Check(S.GetSnapshot(),Request),VisibilityState::Visible);
  CHECK(S.ApplyUpdates(&Target,1)==Error::None); Request.Target=10;
  const auto Result=Q.Check(S.GetSnapshot(),Request);
  Expect(Result,VisibilityState::Blocked); CHECK(Result.BlockingInstance==20);
}
}

int main()
{
  try
  {
    InstanceIdentityAndTransactions(); ArgumentsResourcesAndVariants(); KnownBlockerWinsUnknown();
    HeightFieldTransformsAndHoles(); BoundsExcludeUnreadableTiles(); BudgetCorruptionAndRTree();
    WarmQueriesAndPoseUpdates(); SnapshotThreadsAndLifetime(); TriangleEpsilonAndInvalidIndices();
    LongNearlyParallelRayStillVisitsBlocker();
    LargeTransformCancellationKeepsConservativeBounds();
    std::cout << "public scene/query tests passed\n"; return 0;
  }
  catch(const std::exception &E) { std::cerr << E.what() << '\n'; return 1; }
}
