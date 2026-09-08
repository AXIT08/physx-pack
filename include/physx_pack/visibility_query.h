#ifndef PHYSX_PACK_PUBLIC_VISIBILITY_QUERY_H
#define PHYSX_PACK_PUBLIC_VISIBILITY_QUERY_H

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace physx_pack
{

enum class VisibilityState
{
  Visible,
  Blocked,
  Unknown,
};

struct Vec3
{
  float X = 0.0f;
  float Y = 0.0f;
  float Z = 0.0f;
};

struct Pose
{
  Vec3 Position{};
  float RotationX = 0.0f;
  float RotationY = 0.0f;
  float RotationZ = 0.0f;
  float RotationW = 1.0f;
};

struct Ray
{
  Vec3 Origin{};
  Vec3 Direction{};
  float MaxDistance = 0.0f;
};

struct TargetId
{
  std::string Origin;
  std::string BundleId;
  std::string SerializedFile;
  std::uint64_t PathId = 0;
};

struct TargetInstance
{
  TargetId Id{};
  Pose WorldPose{};
  Vec3 Scale{1.0f, 1.0f, 1.0f};
  bool DoubleSided = false;
};

struct SceneSnapshot
{
  std::vector<TargetInstance> Instances;
};

class VisibilityQuery final
{
public:
  VisibilityQuery() noexcept;
  explicit VisibilityQuery(const char *PackPath) noexcept;
  ~VisibilityQuery() noexcept;

  VisibilityQuery(const VisibilityQuery &) = delete;
  VisibilityQuery &operator=(const VisibilityQuery &) = delete;
  VisibilityQuery(VisibilityQuery &&) noexcept;
  VisibilityQuery &operator=(VisibilityQuery &&) noexcept;

  bool Open() noexcept;
  bool Open(const char *PackPath) noexcept;
  void Close() noexcept;
  bool IsOpen() const noexcept;

  VisibilityState Check(const TargetInstance &Target,
                        const SceneSnapshot &Scene,
                        const Ray &QueryRay) noexcept;

private:
  class Impl;
  std::unique_ptr<Impl> Impl_;
};

} // namespace physx_pack

#endif // PHYSX_PACK_PUBLIC_VISIBILITY_QUERY_H
