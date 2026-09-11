#include "test_fixture.h"
#include <chrono>
#include <iomanip>
#include <random>

namespace
{
using Clock=std::chrono::steady_clock;
double Milliseconds(Clock::time_point Start)
{ return std::chrono::duration<double,std::milli>(Clock::now()-Start).count(); }
double Percentile(std::vector<double> Values,double Fraction)
{ std::sort(Values.begin(),Values.end()); return Values[static_cast<std::size_t>((Values.size()-1)*Fraction)]; }
}
int main()
{
  using namespace physx_pack;
  try
  {
    constexpr unsigned Seed=20260911,InstanceCount=5000,RayCount=512,Rounds=80;
    std::mt19937 Random(Seed); test::TempPack File(test::StandardPack()); Error Failure;
    auto P=Package::Open(File.Path(),{},&Failure); CHECK(P); Scene S(P); QueryContext Q;
    std::vector<InstanceDesc> Items; Items.reserve(InstanceCount);
    for(unsigned I=0;I<InstanceCount;++I)
    {
      auto D=test::Instance(I+1,I<2500?8.0f:20.0f,I<2500?2:1);
      D.WorldPose.Position.X=static_cast<float>((I%2500)%50)*3;
      D.WorldPose.Position.Y=static_cast<float>((I%2500)/50)*3;
      Items.push_back(std::move(D));
    }
    auto Start=Clock::now(); CHECK(S.ApplyUpdates(Items.data(),Items.size())==Error::None);
    const auto PrepareMs=Milliseconds(Start);
    std::array<QueryRequest,RayCount> Requests;
    for(auto &R:Requests)
    {
      const auto Index=2500+(Random()%2500); const auto &D=Items[Index];
      R={D.Id,{{D.WorldPose.Position.X,D.WorldPose.Position.Y,0},{0,0,1},100}};
    }
    std::array<VisibilityResult,RayCount> Results;
    CHECK(Q.CheckBatch(S.GetSnapshot(),Requests.data(),Requests.size(),Results.data())==Error::None);
    std::vector<double> UpdateMs,BatchMs;
    std::array<InstanceDesc,128> Updates;
    std::uint64_t Blocked=0,Unknown=0;
    for(unsigned Frame=0;Frame<Rounds;++Frame)
    {
      for(unsigned I=0;I<Updates.size();++I)
      { Updates[I]=Items[(Frame*128+I)%Items.size()]; Updates[I].WorldPose.Position.X+=(Frame&1)?0.02f:-0.02f; }
      Start=Clock::now(); CHECK(S.ApplyUpdates(Updates.data(),Updates.size())==Error::None);
      UpdateMs.push_back(Milliseconds(Start));
      Start=Clock::now(); CHECK(Q.CheckBatch(S.GetSnapshot(),Requests.data(),Requests.size(),Results.data())==Error::None);
      BatchMs.push_back(Milliseconds(Start));
      for(const auto &R:Results) { Blocked+=R.State==VisibilityState::Blocked; Unknown+=R.State==VisibilityState::Unknown; }
    }
    const auto Cache=P->GetCacheStats(); const auto Info=P->GetInfo(); const auto Scene=S.GetStats(); const auto Query=Q.GetStats();
    CHECK(Unknown==0); CHECK(Blocked==std::uint64_t(Rounds)*RayCount);
    std::cout<<std::fixed<<std::setprecision(3)
      <<"{\"api\":2,\"seed\":"<<Seed<<",\"instances\":"<<InstanceCount<<",\"rays_per_batch\":"<<RayCount
      <<",\"rounds\":"<<Rounds<<",\"prepare_ms\":"<<PrepareMs<<",\"update_p95_ms\":"<<Percentile(UpdateMs,.95)
      <<",\"batch_p50_ms\":"<<Percentile(BatchMs,.5)<<",\"batch_p95_ms\":"<<Percentile(BatchMs,.95)
      <<",\"index_bytes\":"<<Info.IndexBytes<<",\"cache_resident_bytes\":"<<Cache.ResidentBytes
      <<",\"cache_peak_bytes\":"<<Cache.PeakResidentBytes<<",\"cache_temporary_peak_bytes\":"<<Cache.PeakTemporaryBytes
      <<",\"geometry_metadata_bytes\":"<<Cache.GeometryMetadataBytes<<",\"cache_hits\":"<<Cache.Hits
      <<",\"cache_misses\":"<<Cache.Misses<<",\"bytes_decoded\":"<<Cache.BytesDecoded
      <<",\"queries\":"<<Query.Queries<<",\"geometry_tests\":"<<Query.GeometryTests<<",\"bounds_tests\":"<<Query.BoundsTests
      <<",\"bvh_builds\":"<<Scene.BvhBuilds<<",\"bvh_refits\":"<<Scene.BvhRefits
      <<",\"resource_resolutions\":"<<Scene.ResourceResolutions<<",\"blocked\":"<<Blocked<<",\"unknown\":"<<Unknown<<"}\n";
    return 0;
  }
  catch(const std::exception &E) { std::cerr<<E.what()<<'\n'; return 1; }
}
