#ifndef PHYSX_PACK_TEST_FIXTURE_H
#define PHYSX_PACK_TEST_FIXTURE_H

#include "physx_pack/visibility_query.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <unistd.h>
#include <zlib.h>

namespace test
{
using Bytes = std::vector<std::uint8_t>;
inline void Require(bool Value, const char *Expression, int Line)
{
  if (!Value) throw std::runtime_error(std::string("line ") + std::to_string(Line) + ": " + Expression);
}
#define CHECK(expression) ::test::Require(static_cast<bool>(expression), #expression, __LINE__)
inline bool Near(float A, float B) { return std::fabs(A - B) < 1.0e-4f; }
inline void Put32(Bytes &Data, std::size_t At, std::uint32_t Value)
{ for (unsigned I = 0; I < 4; ++I) Data.at(At + I) = static_cast<std::uint8_t>(Value >> (8 * I)); }
inline void Put64(Bytes &Data, std::size_t At, std::uint64_t Value)
{ for (unsigned I = 0; I < 8; ++I) Data.at(At + I) = static_cast<std::uint8_t>(Value >> (8 * I)); }
inline void PutFloat(Bytes &Data, std::size_t At, float Value)
{ std::uint32_t Bits; std::memcpy(&Bits, &Value, sizeof(Bits)); Put32(Data, At, Bits); }
inline physx_pack::ResourceKey Key(std::uint64_t PathId = 1)
{ return {"synthetic", "0000000000000001", "CAB-test", PathId}; }

struct Block { std::uint32_t Id, Kind; Bytes Data; std::uint32_t Codec = 0; bool Corrupt = false; };
struct Source { physx_pack::ResourceKey Key; std::uint32_t Slot; };
struct Preload { std::string Bundle; std::uint32_t Ordinal, Slot; std::uint64_t Count; };
struct PackFixture
{
  std::vector<Block> Blocks;
  std::vector<Source> Sources;
  std::vector<Preload> Preloads;
  Bytes Encode() const
  {
    Bytes Strings, SourceBytes(Sources.size() * 40), PreloadBytes(Preloads.size() * 24);
    const auto String = [&](Bytes &Into, std::size_t At, const std::string &Value)
    {
      Put32(Into, At, static_cast<std::uint32_t>(Strings.size()));
      Put32(Into, At + 4, static_cast<std::uint32_t>(Value.size()));
      Strings.insert(Strings.end(), Value.begin(), Value.end());
    };
    for (std::size_t I = 0; I < Sources.size(); ++I)
    {
      const auto &S = Sources[I]; const auto At = I * 40;
      String(SourceBytes, At, S.Key.Origin); String(SourceBytes, At + 8, S.Key.BundleId);
      String(SourceBytes, At + 16, S.Key.SerializedFile); Put64(SourceBytes, At + 24, S.Key.PathId);
      Put32(SourceBytes, At + 32, S.Slot);
    }
    for (std::size_t I = 0; I < Preloads.size(); ++I)
    {
      const auto &P = Preloads[I]; const auto At = I * 24;
      String(PreloadBytes, At, P.Bundle); Put32(PreloadBytes, At + 8, P.Ordinal);
      Put32(PreloadBytes, At + 12, P.Slot); Put64(PreloadBytes, At + 16, P.Count);
    }
    const std::size_t SourceAt = 104 + Blocks.size() * 40;
    const std::size_t PreloadAt = SourceAt + SourceBytes.size();
    const std::size_t StringAt = PreloadAt + PreloadBytes.size(), DataAt = StringAt + Strings.size();
    Bytes Data(DataAt);
    std::memcpy(Data.data(), "XCPHYSX", 7); Put32(Data, 8, 104);
    Put64(Data, 24, 104); Put32(Data, 32, Blocks.size()); Put32(Data, 36, 40);
    Put64(Data, 40, SourceAt); Put32(Data, 48, Sources.size()); Put32(Data, 52, 40);
    Put64(Data, 56, PreloadAt); Put32(Data, 64, Preloads.size()); Put32(Data, 68, 24);
    Put64(Data, 72, DataAt); Put64(Data, 88, StringAt); Put64(Data, 96, Strings.size());
    std::copy(SourceBytes.begin(), SourceBytes.end(), Data.begin() + SourceAt);
    std::copy(PreloadBytes.begin(), PreloadBytes.end(), Data.begin() + PreloadAt);
    std::copy(Strings.begin(), Strings.end(), Data.begin() + StringAt);
    for (std::size_t I = 0; I < Blocks.size(); ++I)
    {
      const auto &B = Blocks[I]; Bytes Encoded = B.Data;
      if (B.Codec == 1)
      {
        uLongf Size = compressBound(B.Data.size()); Encoded.resize(Size);
        CHECK(compress2(Encoded.data(), &Size, B.Data.data(), B.Data.size(), Z_BEST_SPEED) == Z_OK);
        Encoded.resize(Size);
      }
      if (B.Corrupt && !Encoded.empty()) Encoded[0] ^= 0xFF;
      const auto At = 104 + I * 40;
      Put32(Data, At, B.Id); Put32(Data, At + 4, B.Kind); Put32(Data, At + 8, B.Codec);
      Put64(Data, At + 16, Data.size()); Put64(Data, At + 24, Encoded.size());
      Put64(Data, At + 32, B.Data.size());
      Data.insert(Data.end(), Encoded.begin(), Encoded.end());
    }
    Put64(Data, 80, Data.size() - DataAt);
    return Data;
  }
};

class TempPack
{
public:
  explicit TempPack(const PackFixture &Fixture) : TempPack(Fixture.Encode()) {}
  explicit TempPack(const Bytes &Data)
  {
    char Template[] = "/tmp/physx-test-XXXXXX";
    const int Fd = mkstemp(Template); CHECK(Fd >= 0); close(Fd); Path_ = Template;
    std::ofstream File(Path_, std::ios::binary);
    File.write(reinterpret_cast<const char *>(Data.data()), Data.size()); CHECK(File.good());
  }
  ~TempPack() { std::remove(Path_.c_str()); }
  TempPack(const TempPack &) = delete;
  const char *Path() const { return Path_.c_str(); }
private:
  std::string Path_;
};

inline Bytes MeshRoot(std::uint32_t Triangle, std::uint32_t Convex = UINT32_MAX)
{
  Bytes Data(16); Put32(Data, 0, 0x524D4358); Put32(Data, 4, Triangle); Put32(Data, 8, Convex); return Data;
}
inline Bytes Quad(bool RTree = false)
{
  Bytes Data(152 + (RTree ? 112 : 0)); Put32(Data, 0, 0x544D4358);
  Put32(Data, 4, RTree ? 0 : 1); Put32(Data, 8, 4); Put32(Data, 12, 2);
  Put32(Data, 16, RTree ? 1 : 0); Put32(Data, 20, RTree ? 1 : 0); Put32(Data, 24, 4);
  PutFloat(Data, 28, 1e-7f); Put64(Data, 32, 80); Put64(Data, 40, 128); Put64(Data, 48, 152);
  const std::array<float, 12> Vertices{-1,-1,0, 1,-1,0, 1,1,0, -1,1,0};
  for (std::size_t I = 0; I < Vertices.size(); ++I) PutFloat(Data, 80 + I * 4, Vertices[I]);
  const std::array<unsigned, 6> Indices{0,1,2,0,2,3};
  for (std::size_t I = 0; I < Indices.size(); ++I) Put32(Data, 128 + I * 4, Indices[I]);
  if (RTree)
  {
    for (unsigned Lane = 0; Lane < 4; ++Lane)
      for (unsigned Axis = 0; Axis < 3; ++Axis)
      {
        PutFloat(Data, 152 + Axis * 16 + Lane * 4, Lane ? 1 : (Axis == 2 ? 0 : -1));
        PutFloat(Data, 152 + 48 + Axis * 16 + Lane * 4, Lane ? -1 : (Axis == 2 ? 0 : 1));
      }
    Put32(Data, 152 + 96, 3);
  }
  return Data;
}
inline Bytes Cube()
{
  Bytes Data(112); Put32(Data, 0, 0x504D4358); Put32(Data, 4, 6); Put64(Data, 8, 16);
  const std::array<std::array<float, 4>, 6> Planes{{{1,0,0,-1},{-1,0,0,-1},{0,1,0,-1},
                                                {0,-1,0,-1},{0,0,1,-1},{0,0,-1,-1}}};
  for (std::size_t I = 0; I < 6; ++I) for (std::size_t J = 0; J < 4; ++J)
    PutFloat(Data, 16 + I * 16 + J * 4, Planes[I][J]);
  return Data;
}
inline Bytes HeightField(std::uint32_t Tile)
{
  Bytes Data(52); Put32(Data,0,0x46484358); Put32(Data,4,3); Put32(Data,8,3); Put32(Data,12,64);
  Put32(Data,16,2); Put32(Data,20,2); Put32(Data,24,1); Put32(Data,28,1);
  Put64(Data,32,48); Put32(Data,48,Tile); return Data;
}
inline Bytes HeightTile(bool Holes = false)
{
  Bytes Data(36);
  for (std::size_t I = 0; I < 9; ++I)
  { Data[I*4] = 2; Data[I*4+2] = Holes ? 127 : 0; Data[I*4+3] = Holes ? 127 : 0; }
  return Data;
}
inline PackFixture StandardPack()
{
  PackFixture F;
  F.Blocks = {{1,1,MeshRoot(101,102)}, {2,1,MeshRoot(101)}, {3,2,MeshRoot(UINT32_MAX,102)},
              {4,3,HeightField(201)}, {5,3,HeightField(202)}, {6,1,MeshRoot(103)},
              {101,1,Quad(),1}, {102,2,Cube(),1}, {103,1,Quad(true),1},
              {201,7,HeightTile(),1}, {202,7,HeightTile(true),1}};
  for (unsigned I = 1; I <= 6; ++I) F.Sources.push_back({Key(I), I});
  return F;
}
inline physx_pack::InstanceDesc Instance(std::uint64_t Id, float Z = 0, std::uint64_t Resource = 1)
{
  physx_pack::InstanceDesc D; D.Id = Id; D.Resource = Key(Resource); D.WorldPose.Position.Z = Z;
  D.DoubleSided = true; D.MeshType = physx_pack::MeshVariant::Triangle; return D;
}
} // namespace test
#endif
