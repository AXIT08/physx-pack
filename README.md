# physx.pack 碰撞数据包

[下载 physx.pack](https://github.com/AXIT08/physx-pack/releases/download/cn/physx.pack)

`physx.pack` 是用于碰撞与遮挡查询的外置二进制数据包，面向国服（CN/RMCN）场景。它包含模型碰撞几何、地形高度场、角色命中区域和资源索引。

## 数据内容

| 内容 | 用途 |
| --- | --- |
| 模型碰撞几何 | 三角形表面、凸体及其查询索引 |
| 地形高度场 | 高度、洞口和表面材料 |
| 角色命中区域 | 盒形、胶囊形等局部碰撞区域 |
| 资源索引 | 将完整资源来源对应到几何条目 |

包内数据只描述碰撞形状，不包含贴图、模型显示网格或其他渲染资源。

## 文件位置

支持该格式的应用默认读取：

```text
/data/adb/physx/physx.pack
```

文件名必须保持为 `physx.pack`，无需解压。应用启动时加载一次，运行中替换文件不会自动生效，更新后需要重新启动应用。

## 源码接入

仓库只提供数据包，不提供通用 PhysX SDK。其他 C++ 项目需要在自己的工程中接入三部分：

1. **Pack parser**：打开文件，解析包头、字符串池、source/preload 索引、geometry slot 和数据块目录。
2. **Geometry provider**：按 slot 读取 Mesh、Convex 或 HeightField 数据，并提供查询算法需要的索引、顶点、树节点或 Terrain cell。
3. **Ray query**：使用当前实例的世界姿态、缩放和射线参数，对 provider 提供的几何执行相交查询。

不要把文件直接传给 `PxPhysics` 或当作 ZIP；这是项目自定义的文件布局。资源身份应使用：

```text
origin + bundle_id + serialized_file + path_id
```

不能用名称、单独 path_id、native 指针或数组下标猜测 geometry slot。

### C++ 最小调用示例

以下示例使用 XC 当前的 parser/provider/query 接口，展示正确的生命周期和调用顺序：

```cpp
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

  // 示例值；生产代码必须先由来源索引解析真实 slot。
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

  // Provider 和 Package 必须活到所有查询完成。
  return EXIT_SUCCESS;
}
```

完整文件见 [examples/load_physx_pack.cpp](https://github.com/AXIT08/physx-pack/blob/main/examples/load_physx_pack.cpp)。

### 接入要点

- `Open()` 失败时关闭文件并清空索引；失败不能当作没有碰撞。
- `Bind()` 失败或查询返回 `Undefined` 时，表示几何无法判定，不应直接改写成“遮挡”。
- 同一几何可被多条射线共享 descriptor，但每个实例必须单独应用世界姿态、缩放和双面标志。
- HeightField 使用包内的 signed 16 位 sample；不要再次转置、量化或除以 `32766`。按 cell 读取四个角点，边缘 cell 使用实际尺寸。
- 复杂几何按需读取和解压，不要在每条三角形访问时重新打开文件。
- 数据块读取失败只影响依赖该块的查询；应用应保留“无法判定”状态。
- Provider 回调应捕获异常、清零输出并返回失败，不能让异常穿过查询接口。

### 编译依赖

示例所用实现需要：

- C++17；
- 与接口匹配的 pack parser、geometry provider 和 ray query 实现；
- zlib（用于压缩数据块）。

不同项目可以替换自己的场景过滤和结果结构，但必须保留 slot 身份映射、实例姿态独立性和 `Undefined` 失败语义。
