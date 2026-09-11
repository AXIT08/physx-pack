#ifndef PHYSX_PACK_XC_READ_FRAME_ADAPTER_H
#define PHYSX_PACK_XC_READ_FRAME_ADAPTER_H
#include "physx_pack/visibility_query.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
namespace physx_pack::xc_read
{
struct ReadFrameEntityKey
{
  std::uint8_t Kind = 0;
  std::int64_t EntityId = 0;
  std::uint64_t ObjectToken = 0;
  bool operator==(const ReadFrameEntityKey &o) const noexcept
  {
    return Kind == o.Kind && EntityId == o.EntityId && ObjectToken == o.ObjectToken;
  }
};
struct ReadFrameShape
{
  ReadFrameEntityKey ConsumerKey{};
  ResourceKey Resource{};
  Pose WorldPose{};
  Vec3 Scale{1, 1, 1};
  bool DoubleSided = false;
  MeshVariant MeshType = MeshVariant::Automatic;
  std::uint8_t SceneKind = 0;
};
struct ReadFrameInput
{
  std::uint64_t Sequence = 0, PlanRevision = 0;
  Vec3 Camera{};
  std::array<Vec3, 16> Bones{};
  std::uint32_t ActiveMask = 0;
  ReadFrameEntityKey Target{};
  const ReadFrameShape *Shapes = nullptr;
  std::size_t ShapeCount = 0;
};
struct ReadFrameBoneResult
{
  VisibilityState State = VisibilityState::Unknown;
  Error Reason = Error::NotOpen;
  float Distance = std::numeric_limits<float>::infinity();
  ReadFrameEntityKey Hit{};
};
struct ReadFrameVisibility
{
  std::uint64_t Sequence = 0, PlanRevision = 0;
  std::uint32_t CompletedMask = 0;
  std::array<ReadFrameBoneResult, 16> Bones{};
  Error FrameError = Error::None;
};
class ReadFramePhysxAdapter final
{
public:
  explicit ReadFramePhysxAdapter(std::shared_ptr<Package>);
  ~ReadFramePhysxAdapter() noexcept;
  ReadFramePhysxAdapter(const ReadFramePhysxAdapter &) = delete;
  ReadFramePhysxAdapter &operator=(const ReadFramePhysxAdapter &) = delete;
  Error BeginFrame(std::uint64_t, std::uint64_t, Vec3, const std::array<Vec3, 16> &, std::uint32_t,
                   const ReadFrameEntityKey &) noexcept;
  Error BeginFrame(const ReadFrameInput &) noexcept;
  Error UpsertShape(const ReadFrameShape &) noexcept;
  Error EndFrame() noexcept;
  ReadFrameVisibility Resolve() noexcept;
  ReadFrameVisibility LastFrameResult(Error) const noexcept;
  InstanceId InstanceIdFor(const ReadFrameEntityKey &) const noexcept;
  SceneSnapshot Snapshot() const noexcept;

private:
  struct Impl;
  std::unique_ptr<Impl> Impl_;
};
} // namespace physx_pack::xc_read
#endif
