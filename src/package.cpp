#include "detail/runtime.h"
#include <new>
#include <utility>

namespace physx_pack
{
Package::Package(std::shared_ptr<detail::PackageData> Data) noexcept : Data_(std::move(Data)) {}
Package::~Package() noexcept = default;

std::shared_ptr<Package> Package::Open(const char *Path, const PackageOptions &Options,
                                       Error *Failure) noexcept
{
  if (Failure)
    *Failure = Error::None;
  try
  {
    if (!Path || !*Path)
    {
      if (Failure)
        *Failure = Error::InvalidArgument;
      return {};
    }
    auto Data = std::make_shared<detail::PackageData>(Options.CacheBudgetBytes);
    if (!Data->Reader.Open(Path))
    {
      if (Failure)
        *Failure = Data->Reader.OpenError();
      return {};
    }
    return std::shared_ptr<Package>(new Package(std::move(Data)));
  }
  catch (const std::bad_alloc &)
  {
    if (Failure)
      *Failure = Error::OutOfMemory;
  }
  catch (...)
  {
    if (Failure)
      *Failure = Error::IoError;
  }
  return {};
}

PackageInfo Package::GetInfo() const noexcept
{
  return Data_->Reader.Info();
}
CacheStats Package::GetCacheStats() const noexcept
{
  auto Stats = Data_->Reader.Stats();
  Stats.GeometryMetadataBytes = Data_->Geometry.MetadataBytes();
  return Stats;
}

const char *ToString(VisibilityState Value) noexcept
{
  switch (Value)
  {
  case VisibilityState::Visible:
    return "visible";
  case VisibilityState::Blocked:
    return "blocked";
  case VisibilityState::Unknown:
    return "unknown";
  }
  return "unknown";
}
const char *ToString(Error Value) noexcept
{
  switch (Value)
  {
  case Error::None:
    return "none";
  case Error::InvalidArgument:
    return "invalid_argument";
  case Error::NotOpen:
    return "not_open";
  case Error::IoError:
    return "io_error";
  case Error::CorruptData:
    return "corrupt_data";
  case Error::UnsupportedGeometry:
    return "unsupported_geometry";
  case Error::MissingResource:
    return "missing_resource";
  case Error::MissingVariant:
    return "missing_variant";
  case Error::BudgetExceeded:
    return "budget_exceeded";
  case Error::OutOfMemory:
    return "out_of_memory";
  case Error::TargetNotFound:
    return "target_not_found";
  case Error::TargetNotHit:
    return "target_not_hit";
  }
  return "invalid_error";
}
const char *ToString(RaycastState Value) noexcept
{
  switch (Value)
  {
  case RaycastState::Miss: return "miss";
  case RaycastState::Hit: return "hit";
  case RaycastState::Unknown: return "unknown";
  }
  return "unknown";
}
} // namespace physx_pack
