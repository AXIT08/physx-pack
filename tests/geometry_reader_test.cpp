#include "test_fixture.h"
#include "detail/geometry_reader.h"

namespace
{
using namespace test;
using physx_pack::Error;
namespace d = physx_pack::detail;
namespace v = physx_pack::visibility;

void HeightFieldBoundsAndWarmMetadata()
{
  TempPack File(StandardPack()); d::PackReader Pack(1024*1024); CHECK(Pack.Open(File.Path()));
  d::GeometryStore Store(Pack); d::GeometryDescription Description;
  CHECK(Store.Describe(4,v::GeometryType::HeightField,&Description)==Error::None);
  CHECK(Description.BoundsDefined); CHECK(Near(Description.LocalBounds.Minimum.Y,2));
  CHECK(Near(Description.LocalBounds.Maximum.X,2)); CHECK(Near(Description.LocalBounds.Maximum.Z,2));
  const auto Metadata=Store.MetadataBytes(); CHECK(Metadata>0);
  d::GeometryReader Lease; v::GeometryView View;
  CHECK(Store.Bind(4,v::GeometryType::HeightField,Lease,&View)==Error::None);
  CHECK(Near(View.HeightField.RowScale,1)); CHECK(Near(View.HeightField.ColumnScale,1));
  CHECK(Near(View.HeightField.HeightScale,1));
  std::array<v::HeightFieldSample,4> Samples;
  CHECK(View.HeightField.LoadCell(View.HeightField.ProviderContext,0,0,&Samples));
  CHECK(Samples[0].Height==2); const auto Decoded=Pack.Stats().BytesDecoded;
  for(unsigned I=0;I<50;++I)
  {
    CHECK(Store.Bind(4,v::GeometryType::HeightField,Lease,&View)==Error::None);
    CHECK(Near(View.HeightField.RowScale,1)); CHECK(Store.MetadataBytes()==Metadata);
  }
  CHECK(Pack.Stats().BytesDecoded==Decoded);
  View.WorldPose.Position={7,11,13}; View.HeightField.RowScale=-2;
  View.HeightField.ColumnScale=-4; View.HeightField.HeightScale=3; View.HeightField.DoubleSided=true;
  View.HeightField.LocalBounds={{-4,6,-8},{0,6,0}};
  const auto Hit=v::RaycastGeometry({{6.5f,30,12},{0,-1,0},100},View,true);
  CHECK(Hit.Status==v::QueryStatus::Hit); CHECK(Near(Hit.Distance,13));
  CHECK(Lease.LastError()==Error::None);
}

void TileBudgetErrorSurvivesCallback()
{
  TempPack File(StandardPack()); d::PackReader Pack(70); CHECK(Pack.Open(File.Path()));
  d::GeometryStore Store(Pack); d::GeometryReader Lease; v::GeometryView View;
  CHECK(Store.Bind(4,v::GeometryType::HeightField,Lease,&View)==Error::None);
  std::array<v::HeightFieldSample,4> Samples;
  for(auto &S:Samples) S.Height=42;
  CHECK(!View.HeightField.LoadCell(View.HeightField.ProviderContext,0,0,&Samples));
  CHECK(Lease.LastError()==Error::BudgetExceeded);
  for(const auto &S:Samples) CHECK(S.Height==0 && S.Material0==0 && S.Material1==0);
  CHECK(Pack.Stats().ResidentBytes<=70); Lease.Reset(); CHECK(Lease.LastError()==Error::None);
}

void TileIoErrorSurvivesCallback()
{
  TempPack File(StandardPack()); d::PackReader Pack(1024*1024); CHECK(Pack.Open(File.Path()));
  d::GeometryStore Store(Pack); d::GeometryReader Lease; v::GeometryView View;
  CHECK(Store.Bind(4,v::GeometryType::HeightField,Lease,&View)==Error::None);
  CHECK(truncate(File.Path(),0)==0);
  std::array<v::HeightFieldSample,4> Samples;
  CHECK(!View.HeightField.LoadCell(View.HeightField.ProviderContext,0,0,&Samples));
  CHECK(Lease.LastError()==Error::IoError);
}

void InvalidTriangleIndicesHaveNoTrustedBounds()
{
  auto F=StandardPack(); for(auto &B:F.Blocks) if(B.Id==101) Put32(B.Data,128,1000);
  TempPack File(F); d::PackReader Pack; CHECK(Pack.Open(File.Path()));
  d::GeometryStore Store(Pack); d::GeometryDescription Description;
  CHECK(Store.Describe(1,v::GeometryType::TriangleMesh,&Description)==Error::CorruptData);
  CHECK(!Description.BoundsDefined);
}
}
int main()
{
  try
  {
    HeightFieldBoundsAndWarmMetadata(); TileBudgetErrorSurvivesCallback();
    TileIoErrorSurvivesCallback(); InvalidTriangleIndicesHaveNoTrustedBounds();
    std::cout << "geometry metadata/lease tests passed\n"; return 0;
  }
  catch(const std::exception &E) { std::cerr << E.what() << '\n'; return 1; }
}
