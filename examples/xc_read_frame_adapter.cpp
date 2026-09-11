#include "xc_read_frame_adapter.h"
#include <cmath>
#include <functional>
#include <new>
#include <unordered_map>
#include <unordered_set>
#include <vector>
namespace physx_pack::xc_read
{
namespace
{
struct KeyHash
{
  std::size_t operator()(const ReadFrameEntityKey &k) const noexcept
  {
    std::size_t h = k.Kind;
    h ^= std::hash<std::int64_t>{}(k.EntityId) + (h << 6) + (h >> 2);
    h ^= std::hash<std::uint64_t>{}(k.ObjectToken) + (h << 6) + (h >> 2);
    return h;
  }
};
bool Finite(Vec3 v) noexcept
{
  return std::isfinite(v.X) && std::isfinite(v.Y) && std::isfinite(v.Z);
}
bool ValidKey(const ReadFrameEntityKey &k) noexcept
{
  return k.Kind && k.EntityId && k.ObjectToken;
}
bool ValidResource(const ResourceKey &r) noexcept
{
  return !r.Origin.empty() && !r.BundleId.empty() && !r.SerializedFile.empty() && r.PathId;
}
bool ValidShape(const ReadFrameShape &s) noexcept
{
  return ValidKey(s.ConsumerKey) && ValidResource(s.Resource) && Finite(s.WorldPose.Position) &&
         Finite(s.Scale) && s.Scale.X != 0 && s.Scale.Y != 0 && s.Scale.Z != 0 &&
         std::isfinite(s.WorldPose.RotationX) && std::isfinite(s.WorldPose.RotationY) &&
         std::isfinite(s.WorldPose.RotationZ) && std::isfinite(s.WorldPose.RotationW);
}
InstanceId StableId(const ReadFrameEntityKey &k) noexcept
{
  std::uint64_t h = 1469598103934665603ULL;
  h ^= k.Kind;
  h *= 1099511628211ULL;
  for (std::uint64_t v : {static_cast<std::uint64_t>(k.EntityId), k.ObjectToken})
    for (unsigned s = 0; s < 64; s += 8)
    {
      h ^= (v >> s) & 0xffU;
      h *= 1099511628211ULL;
    }
  h |= std::uint64_t{1} << 63;
  return h;
}
} // namespace
struct ReadFramePhysxAdapter::Impl
{
  std::shared_ptr<Package> PackageHandle;
  std::unique_ptr<Scene> SceneHandle;
  SceneSnapshot Published;
  QueryContext Query;
  std::unordered_map<ReadFrameEntityKey, InstanceId, KeyHash> KeyIds;
  std::unordered_map<InstanceId, ReadFrameEntityKey> IdKeys;
  std::unordered_set<InstanceId> PublishedIds, PendingIds;
  std::vector<InstanceDesc> Pending;
  std::array<Vec3, 16> Bones{};
  Vec3 Camera{};
  std::uint32_t ActiveMask = 0;
  std::uint64_t Sequence = 0, PlanRevision = 0, LastSequence = 0, LastPlanRevision = 0;
  InstanceId Target = 0;
  bool InFrame = false;
  Error FrameError = Error::None;
  InstanceId GetId(const ReadFrameEntityKey &k)
  {
    auto i = KeyIds.find(k);
    if (i != KeyIds.end())
      return i->second;
    InstanceId id = StableId(k);
    while (!id || (IdKeys.count(id) && !(IdKeys.at(id) == k)))
      ++id;
    KeyIds.emplace(k, id);
    IdKeys.emplace(id, k);
    return id;
  }
};
ReadFramePhysxAdapter::ReadFramePhysxAdapter(std::shared_ptr<Package> p)
    : Impl_(std::make_unique<Impl>())
{
  Impl_->PackageHandle = std::move(p);
  if (Impl_->PackageHandle)
    Impl_->SceneHandle = std::make_unique<Scene>(Impl_->PackageHandle);
  else
    Impl_->FrameError = Error::NotOpen;
}
ReadFramePhysxAdapter::~ReadFramePhysxAdapter() noexcept = default;
Error ReadFramePhysxAdapter::BeginFrame(std::uint64_t seq, std::uint64_t rev, Vec3 cam,
                                        const std::array<Vec3, 16> &bones, std::uint32_t mask,
                                        const ReadFrameEntityKey &target) noexcept
{
  try
  {
    if (!Impl_->SceneHandle)
      return Impl_->FrameError = Error::NotOpen;
    if (!seq || seq <= Impl_->LastSequence || rev < Impl_->LastPlanRevision || !ValidKey(target) ||
        (mask & 0xffff0000U) || !Finite(cam))
      return Impl_->FrameError = Error::InvalidArgument;
    for (auto b : bones)
      if (!Finite(b))
        return Impl_->FrameError = Error::InvalidArgument;
    Impl_->Sequence = seq;
    Impl_->PlanRevision = rev;
    Impl_->Camera = cam;
    Impl_->Bones = bones;
    Impl_->ActiveMask = mask;
    Impl_->Target = Impl_->GetId(target);
    Impl_->Pending.clear();
    Impl_->PendingIds.clear();
    Impl_->FrameError = Error::None;
    Impl_->InFrame = true;
    return Error::None;
  }
  catch (const std::bad_alloc &)
  {
    return Impl_->FrameError = Error::OutOfMemory;
  }
}
Error ReadFramePhysxAdapter::BeginFrame(const ReadFrameInput &i) noexcept
{
  Error e = BeginFrame(i.Sequence, i.PlanRevision, i.Camera, i.Bones, i.ActiveMask, i.Target);
  if (e != Error::None)
    return e;
  if (i.ShapeCount && !i.Shapes)
    return Impl_->FrameError = Error::InvalidArgument;
  for (std::size_t n = 0; n < i.ShapeCount; ++n)
    if ((e = UpsertShape(i.Shapes[n])) != Error::None)
      return e;
  return Error::None;
}
Error ReadFramePhysxAdapter::UpsertShape(const ReadFrameShape &s) noexcept
{
  try
  {
    if (!Impl_->InFrame || Impl_->FrameError != Error::None || !ValidShape(s))
      return Impl_->FrameError = Error::InvalidArgument;
    InstanceId id = Impl_->GetId(s.ConsumerKey);
    if (!Impl_->PendingIds.insert(id).second)
      return Impl_->FrameError = Error::InvalidArgument;
    InstanceDesc d;
    d.Id = id;
    d.Resource = s.Resource;
    d.WorldPose = s.WorldPose;
    d.Scale = s.Scale;
    d.DoubleSided = s.DoubleSided;
    d.MeshType = s.MeshType;
    Impl_->Pending.push_back(std::move(d));
    return Error::None;
  }
  catch (const std::bad_alloc &)
  {
    return Impl_->FrameError = Error::OutOfMemory;
  }
}
Error ReadFramePhysxAdapter::EndFrame() noexcept
{
  try
  {
    if (!Impl_->InFrame)
      return Error::InvalidArgument;
    if (Impl_->FrameError != Error::None)
    {
      Impl_->InFrame = false;
      return Impl_->FrameError;
    }
    std::vector<InstanceId> rem;
    for (auto id : Impl_->PublishedIds)
      if (!Impl_->PendingIds.count(id))
        rem.push_back(id);
    Error e = Impl_->SceneHandle->ApplyUpdates(Impl_->Pending.data(), Impl_->Pending.size(),
                                               rem.data(), rem.size());
    if (e == Error::None)
    {
      Impl_->Published = Impl_->SceneHandle->GetSnapshot();
      Impl_->PublishedIds = Impl_->PendingIds;
      Impl_->LastSequence = Impl_->Sequence;
      Impl_->LastPlanRevision = Impl_->PlanRevision;
    }
    Impl_->InFrame = false;
    Impl_->Pending.clear();
    Impl_->PendingIds.clear();
    Impl_->FrameError = e;
    return e;
  }
  catch (const std::bad_alloc &)
  {
    Impl_->InFrame = false;
    Impl_->Pending.clear();
    Impl_->PendingIds.clear();
    return Impl_->FrameError = Error::OutOfMemory;
  }
}
ReadFrameVisibility ReadFramePhysxAdapter::Resolve() noexcept
{
  ReadFrameVisibility o;
  o.Sequence = Impl_->LastSequence;
  o.PlanRevision = Impl_->LastPlanRevision;
  if (!Impl_->Published)
  {
    o.FrameError = Error::NotOpen;
    return o;
  }
  std::array<RaycastRequest, 16> rq{};
  std::array<std::size_t, 16> slots{};
  std::size_t n = 0;
  for (std::size_t i = 0; i < 16; ++i)
    if (Impl_->ActiveMask & (1U << i))
    {
      Vec3 d{Impl_->Bones[i].X - Impl_->Camera.X, Impl_->Bones[i].Y - Impl_->Camera.Y,
             Impl_->Bones[i].Z - Impl_->Camera.Z};
      float dist =
          static_cast<float>(std::sqrt(double(d.X) * d.X + double(d.Y) * d.Y + double(d.Z) * d.Z));
      if (!(dist > 0) || !std::isfinite(dist))
        continue;
      rq[n].QueryRay = {Impl_->Camera, d, dist};
      rq[n].IgnoreInstance = Impl_->Target;
      slots[n++] = i;
    }
  std::array<RaycastResult, 16> rs{};
  o.FrameError = Impl_->Query.TraceBatch(Impl_->Published, rq.data(), n, rs.data());
  for (std::size_t i = 0; i < n; ++i)
  {
    auto s = slots[i];
    o.CompletedMask |= 1U << s;
    o.Bones[s].Reason = rs[i].Reason;
    o.Bones[s].Distance = rs[i].Distance;
    if (rs[i].HitInstance)
    {
      auto it = Impl_->IdKeys.find(rs[i].HitInstance);
      if (it != Impl_->IdKeys.end())
        o.Bones[s].Hit = it->second;
    }
    o.Bones[s].State = rs[i].State == RaycastState::Miss  ? VisibilityState::Visible
                       : rs[i].State == RaycastState::Hit ? VisibilityState::Blocked
                                                          : VisibilityState::Unknown;
  }
  return o;
}
ReadFrameVisibility ReadFramePhysxAdapter::LastFrameResult(Error e) const noexcept
{
  ReadFrameVisibility o;
  o.Sequence = Impl_->Sequence;
  o.PlanRevision = Impl_->PlanRevision;
  o.FrameError = e;
  return o;
}
InstanceId ReadFramePhysxAdapter::InstanceIdFor(const ReadFrameEntityKey &k) const noexcept
{
  auto i = Impl_->KeyIds.find(k);
  return i == Impl_->KeyIds.end() ? 0 : i->second;
}
SceneSnapshot ReadFramePhysxAdapter::Snapshot() const noexcept
{
  return Impl_->Published;
}
} // namespace physx_pack::xc_read
