#include "detail/pack_reader.h"
#include "test_fixture.h"

#include <atomic>
#include <chrono>
#include <sys/resource.h>
#include <thread>

using physx_pack::Error;
using physx_pack::detail::BlockView;
using physx_pack::detail::PackReader;
using physx_pack::detail::PreloadStatus;

namespace
{
physx_pack::detail::SourceKey Source(std::uint64_t Id)
{
  const auto Value = test::Key(Id);
  return {Value.Origin, Value.BundleId, Value.SerializedFile, Value.PathId};
}
std::uint64_t Read64(const test::Bytes &Bytes, std::size_t At)
{
  std::uint64_t Value = 0;
  for (unsigned I = 0; I < 8; ++I) Value |= static_cast<std::uint64_t>(Bytes[At + I]) << (8 * I);
  return Value;
}

void ErrorsAndSourceConflicts()
{
  PackReader Reader;
  std::uint32_t Slot = 123;
  CHECK(Reader.ResolveSource(Source(1), &Slot) == Error::NotOpen);
  CHECK(Slot == UINT32_MAX);
  CHECK(Reader.ResolveSource(Source(1), nullptr) == Error::InvalidArgument);
  CHECK(!Reader.Open(nullptr));
  CHECK(Reader.OpenError() == Error::InvalidArgument);
  CHECK(!Reader.Open("/definitely/not/a/physx/package"));
  CHECK(Reader.OpenError() == Error::IoError);
  test::TempPack Invalid(test::Bytes(20));
  CHECK(!Reader.Open(Invalid.Path()));
  CHECK(Reader.OpenError() == Error::CorruptData);
  test::PackFixture Fixture;
  Fixture.Blocks = {{1, 7, {1,2,3,4}}, {2, 7, {5,6,7,8}}};
  Fixture.Sources = {{test::Key(1), 1}, {test::Key(1), 1}, {test::Key(2), 1},
                     {test::Key(2), 2}, {test::Key(3), 999}};
  test::TempPack Pack(Fixture);
  CHECK(Reader.Open(Pack.Path()));
  CHECK(Reader.OpenError() == Error::None);
  CHECK(Reader.ResolveSource(Source(1), &Slot) == Error::None && Slot == 1);
  CHECK(Reader.ResolveSource(Source(2), &Slot) == Error::CorruptData && Slot == UINT32_MAX);
  CHECK(Reader.ResolveSource(Source(3), &Slot) == Error::MissingResource);
  CHECK(Reader.ResolveSource(Source(999), &Slot) == Error::MissingResource);
  CHECK(Reader.ResolveSource({}, &Slot) == Error::InvalidArgument);
  CHECK(Reader.Info().SourceCount == 5 && Reader.Info().BlockCount == 2);
  CHECK(Reader.Info().IndexBytes > 0);
  CHECK(Reader.Stats().ResidentBytes == 0);
  PackReader Moved(std::move(Reader));
  CHECK(!Reader.IsOpen());
  CHECK(Moved.ResolveSource(Source(1), &Slot) == Error::None);
  CHECK(Moved.OpenError() == Error::None);
  auto Empty = test::PackFixture{}.Encode();
  for (auto Offset : {24U,40U,56U,72U,88U}) test::Put64(Empty, Offset, 0);
  test::TempPack EmptyPack(Empty);
  CHECK(Reader.Open(EmptyPack.Path()));
  CHECK(Reader.Info().SourceCount == 0);
}

void CompactPreloads()
{
  test::PackFixture Fixture;
  Fixture.Blocks = {{1, 7, {1}}, {2, 7, {2}}};
  Fixture.Preloads = {
      {"0x00000000000000ab",2,UINT32_MAX,4}, {"00000000000000AB",0,1,4},
      {"0X00000000000000AB",0,1,4}, {"0x00000000000000ab",1,1,4},
      {"00000000000000ab",1,2,4}, {"00000000000000ab",3,999,4},
      {"named",0,1,2}, {"named",1,2,3}};
  test::TempPack Pack(Fixture);
  PackReader Reader;
  CHECK(Reader.Open(Pack.Path()));
  std::uint32_t Slot = 0;
  std::uint64_t Count = 0;
  CHECK(Reader.BundlePreloadCount("0x00000000000000ab", &Count) && Count == 4);
  CHECK(Reader.LookupPreload("00000000000000AB", 0, &Slot) == PreloadStatus::Bound && Slot == 1);
  CHECK(Reader.LookupPreload("0x00000000000000ab", 1, &Slot) == PreloadStatus::Unresolved);
  CHECK(Reader.LookupPreload("00000000000000ab", 2, &Slot) == PreloadStatus::Unresolved);
  CHECK(Reader.LookupPreload("00000000000000ab", 3, &Slot) == PreloadStatus::Unresolved);
  CHECK(Reader.LookupPreload("00000000000000ab", 4, &Slot) == PreloadStatus::Missing);
  CHECK(!Reader.BundlePreloadCount("named", &Count));
  CHECK(Reader.LookupPreload("named", 0, &Slot) == PreloadStatus::Unresolved);
  const auto *Directory = Reader.GetBundlePreloadDirectory(0xab);
  CHECK(Directory && Directory->CountDefined && Directory->Entries.size() == 4);
  CHECK(Directory->Entries[0].Ordinal == 0 && Directory->Entries[0].GeometrySlot == 1);
  CHECK(Directory->Entries[1].GeometrySlot == UINT32_MAX);
  PackReader Moved(std::move(Reader));
  CHECK(Moved.GetBundlePreloadDirectory(0xab)->Entries[0].GeometrySlot == 1);

  Fixture.Preloads.clear();
  constexpr unsigned Ordinals = 100000;
  for (unsigned I = 0; I < Ordinals; ++I) Fixture.Preloads.push_back({"0000000000000001",I,1,Ordinals});
  test::TempPack Large(Fixture);
  CHECK(Moved.Open(Large.Path()));
  CHECK(Moved.Info().PreloadCount == Ordinals);
  CHECK(Moved.Info().IndexBytes < Ordinals * 32ULL);
  CHECK(Moved.GetBundlePreloadDirectory(1)->Entries.size() == Ordinals);
  CHECK(Moved.LookupPreload("0000000000000001", Ordinals - 1, &Slot) == PreloadStatus::Bound);
}

void BudgetAndPins()
{
  test::PackFixture Fixture;
  Fixture.Blocks = {{1,7,{1,1,1,1}}, {2,7,{2,2,2,2}}, {3,7,{3,3,3,3}}, {4,7,{}}};
  test::TempPack Pack(Fixture);
  PackReader Reader(8);
  CHECK(Reader.Open(Pack.Path()));
  BlockView A, B, C;
  Error Failure = Error::NotOpen;
  CHECK(Reader.AcquireBlock(1, &A, &Failure) && Failure == Error::None);
  CHECK(Reader.AcquireBlock(2, &B, &Failure));
  CHECK(!Reader.AcquireBlock(3, &C, &Failure) && Failure == Error::BudgetExceeded);
  CHECK(Reader.OpenError() == Error::None);
  CHECK(Reader.Stats().ResidentBytes == 8 && Reader.Stats().PeakResidentBytes == 8);
  A.Storage.reset();
  CHECK(Reader.AcquireBlock(3, &C, &Failure));
  CHECK(Reader.Stats().Evictions == 1);
  CHECK(B.Data()[0] == 2 && C.Data()[0] == 3);
  CHECK(Reader.AcquireBlock(2, &A, &Failure));
  CHECK(Reader.Stats().Hits == 1);
  A.Storage.reset(); B.Storage.reset(); C.Storage.reset();
  Reader.Close();
  CHECK(Reader.Stats().ResidentBytes == 0);
  CHECK(!Reader.AcquireBlock(1, &A, &Failure) && Failure == Error::NotOpen);

  PackReader Tiny(4);
  CHECK(Tiny.Open(Pack.Path()) && Tiny.AcquireBlock(1, &A));
  Tiny.Close();
  CHECK(A.Data()[0] == 1 && Tiny.Stats().ResidentBytes == 4);
  CHECK(Tiny.Open(Pack.Path()));
  CHECK(!Tiny.AcquireBlock(2, &B, &Failure) && Failure == Error::BudgetExceeded);
  A.Storage.reset();
  CHECK(Tiny.AcquireBlock(2, &B, &Failure));
  PackReader Zero(0);
  CHECK(Zero.Open(Pack.Path()));
  CHECK(!Zero.AcquireBlock(1, &A, &Failure) && Failure == Error::BudgetExceeded);
  CHECK(Zero.AcquireBlock(4, &A, &Failure) && A.Size() == 0);
  PackReader Large(256U * 1024U * 1024U);
  CHECK(Large.Stats().BudgetBytes == 256U * 1024U * 1024U);
}

void MalformedBlocksAndZlib()
{
  test::PackFixture Fixture;
  test::Bytes Random(150000);
  std::uint32_t Seed = 1327;
  for (auto &Byte : Random) { Seed = Seed * 1664525U + 1013904223U; Byte = Seed >> 24; }
  Fixture.Blocks = {{1,7,Random,1}, {2,7,{1,2,3},1,true}, {3,7,{1,2,3},2}, {4,7,{},1}};
  test::TempPack Pack(Fixture);
  PackReader Reader(200000);
  CHECK(Reader.Open(Pack.Path()));
  BlockView View;
  Error Failure;
  CHECK(Reader.AcquireBlock(1, &View, &Failure));
  CHECK(*View.Storage == Random);
  CHECK(Reader.Stats().TemporaryBytes == 0);
  CHECK(Reader.Stats().PeakTemporaryBytes > 65536 && Reader.Stats().PeakTemporaryBytes < 262144);
  CHECK(!Reader.AcquireBlock(2, &View, &Failure) && Failure == Error::CorruptData);
  CHECK(!Reader.AcquireBlock(3, &View, &Failure) && Failure == Error::UnsupportedGeometry);
  CHECK(!Reader.AcquireBlock(999, &View, &Failure) && Failure == Error::MissingResource);
  CHECK(!Reader.AcquireBlock(1, nullptr, &Failure) && Failure == Error::InvalidArgument);
  CHECK(Reader.AcquireBlock(4, &View, &Failure) && View.Size() == 0);

  Fixture.Blocks = {{1,7,{1,2,3},1}};
  auto Trailing = Fixture.Encode();
  Trailing.push_back(99);
  test::Put64(Trailing, 104 + 24, Read64(Trailing, 104 + 24) + 1);
  test::Put64(Trailing, 80, Read64(Trailing, 80) + 1);
  test::TempPack TrailingPack(Trailing);
  CHECK(Reader.Open(TrailingPack.Path()));
  CHECK(!Reader.AcquireBlock(1, &View, &Failure) && Failure == Error::CorruptData);
  CHECK(Reader.Stats().ResidentBytes == 0 && Reader.Stats().TemporaryBytes == 0);
  auto TooLarge = Fixture.Encode();
  test::Put64(TooLarge, 104 + 32, 128U * 1024U * 1024U + 1ULL);
  test::TempPack Oversize(TooLarge);
  CHECK(Reader.Open(Oversize.Path()));
  CHECK(!Reader.AcquireBlock(1, &View, &Failure) && Failure == Error::CorruptData);

  Fixture.Blocks = {{1,7,{1}}, {1,7,{2}}};
  Fixture.Sources = {{test::Key(1),1}};
  test::TempPack Duplicate(Fixture);
  CHECK(Reader.Open(Duplicate.Path()));
  std::uint32_t Slot;
  CHECK(Reader.ResolveSource(Source(1), &Slot) == Error::CorruptData);
  CHECK(!Reader.AcquireBlock(1, &View, &Failure) && Failure == Error::CorruptData);
}

void ConcurrentCache()
{
  test::PackFixture Fixture;
  Fixture.Blocks = {{1,7,test::Bytes(4096,42),1}};
  Fixture.Sources = {{test::Key(1),1}};
  test::TempPack Pack(Fixture);
  PackReader Reader(4096);
  CHECK(Reader.Open(Pack.Path()));
  std::atomic<bool> Failed{false};
  std::vector<std::thread> Threads;
  for (unsigned Worker = 0; Worker < 8; ++Worker) Threads.emplace_back([&]
  {
    for (unsigned Iteration = 0; Iteration < 1000; ++Iteration)
    {
      BlockView View;
      Error Failure = Error::CorruptData;
      std::uint32_t Slot;
      if (Reader.ResolveSource(Source(1), &Slot) != Error::None || Slot != 1 ||
          !Reader.AcquireBlock(1, &View, &Failure) || Failure != Error::None ||
          View.Size() != 4096 || View.Data()[4095] != 42) Failed.store(true);
      BlockView Missing;
      if (Reader.AcquireBlock(999, &Missing, &Failure) || Failure != Error::MissingResource)
        Failed.store(true);
    }
  });
  for (auto &Thread : Threads) Thread.join();
  CHECK(!Failed.load());
  const auto Stats = Reader.Stats();
  CHECK(Stats.Misses == 1 && Stats.Hits == 7999 && Stats.BytesDecoded == 4096);
  CHECK(Stats.ResidentBytes == 4096 && Stats.PeakResidentBytes == 4096);
  CHECK(Reader.OpenError() == Error::None);
}
} // namespace

int main(int ArgumentCount, char **Arguments)
{
  try
  {
    if (ArgumentCount == 3 && std::string(Arguments[1]) == "--inspect")
    {
      const auto Start = std::chrono::steady_clock::now();
      PackReader Reader;
      CHECK(Reader.Open(Arguments[2]));
      const auto Milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - Start).count();
      const auto Info = Reader.Info();
      struct rusage Usage{};
      CHECK(getrusage(RUSAGE_SELF, &Usage) == 0);
      std::cout << "{\"file_bytes\":" << Info.FileBytes << ",\"source_count\":" << Info.SourceCount
                << ",\"block_count\":" << Info.BlockCount << ",\"preload_count\":" << Info.PreloadCount
                << ",\"index_bytes\":" << Info.IndexBytes << ",\"open_ms\":" << Milliseconds
                << ",\"peak_rss_kib\":" << Usage.ru_maxrss
                << ",\"decoded_resident_bytes\":" << Reader.Stats().ResidentBytes << "}\n";
      return 0;
    }
    ErrorsAndSourceConflicts(); CompactPreloads(); BudgetAndPins();
    MalformedBlocksAndZlib(); ConcurrentCache();
    std::cout << "pack_reader tests passed\n";
    return 0;
  }
  catch (const std::exception &Failure) { std::cerr << Failure.what() << '\n'; return 1; }
}
