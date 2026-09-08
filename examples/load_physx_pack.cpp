#include "Read/ExternalCollisionPackage.h"
#include "Read/ExternalGeometryProvider.h"
#include "Read/ReadVisibilityQuery.h"

#include <cstdlib>
#include <iostream>

int main(int argc, char **argv)
{
  const char *PackPath = argc > 1 ? argv[1] : nullptr;

  xc::read_detail::ExternalCollisionPackage Package;
  if (!(PackPath != nullptr ? Package.Open(PackPath) : Package.Open()))
  {
    std::cerr << "cannot open physx.pack\n";
    return EXIT_FAILURE;
  }

  // Slot 必须来自完整来源索引或运行时映射，不能把文件下标当成 slot。
  constexpr std::uint32_t GeometrySlot = 1234U;
  xc::read_detail::ExternalGeometryProvider Provider;
  xc::visibility::GeometryView Geometry{};
  Geometry.WorldPose.Position = {0.0f, 0.0f, 0.0f};
  Geometry.HeightField.HeightScale = 1.0f;
  Geometry.HeightField.RowScale = 1.0f;
  Geometry.HeightField.ColumnScale = 1.0f;

  if (!Provider.Bind(Package, GeometrySlot,
                     xc::visibility::GeometryType::HeightField,
                     &Geometry))
  {
    std::cerr << "geometry slot is unavailable\n";
    return EXIT_FAILURE;
  }

  xc::visibility::Ray Ray{};
  Ray.Origin = {0.0f, 100.0f, 0.0f};
  Ray.Direction = {0.0f, -1.0f, 0.0f};
  Ray.MaxDistance = 200.0f;

  const xc::visibility::RayHit Hit =
      xc::visibility::RaycastGeometry(Ray, Geometry);
  switch (Hit.Status)
  {
  case xc::visibility::QueryStatus::Hit:
    std::cout << "hit distance: " << Hit.Distance << "\n";
    break;
  case xc::visibility::QueryStatus::Miss:
    std::cout << "miss\n";
    break;
  case xc::visibility::QueryStatus::Undefined:
    std::cout << "undefined\n";
    break;
  }

  // Provider 必须在所有 RaycastGeometry 调用结束后才销毁。
  return EXIT_SUCCESS;
}
