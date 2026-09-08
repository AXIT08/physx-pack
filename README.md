# physx.pack 可见性查询示例

[下载 physx.pack](https://github.com/AXIT08/physx-pack/releases/download/cn/physx.pack)

本仓库提供一个可直接编译的 C++17 示例工程，用于读取 `physx.pack` 并进行可见性判断。工程包含三部分：包解析、几何读取和可见性门面。

`physx.pack` 是项目自定义的碰撞数据包，不是 ZIP，也不能直接传给通用 PhysX SDK。它包含模型碰撞几何、地形高度场、角色命中区域和资源索引。

## 快速编译

需要 C++17、CMake、Ninja 或其他 CMake 生成器，以及 zlib。

```sh
cmake -S . -B build -G Ninja
cmake --build build
```

生成的 `physx_visibility_example` 只依赖公开的可见性头文件。运行时可以传入 pack 路径；不传参数时使用：

```text
/data/adb/physx/physx.pack
```

```sh
./build/physx_visibility_example /path/to/physx.pack
```

## 可见性查询

应用只需要包含 `include/physx_pack/visibility_query.h`：

```cpp
#include "physx_pack/visibility_query.h"

#include <iostream>

int main()
{
  physx_pack::VisibilityQuery Query;
  if (!Query.Open("/path/to/physx.pack"))
  {
    return 1;
  }

  physx_pack::TargetInstance Target{};
  Target.Id = {"cn", "example.bundle", "CAB-example", 1U};

  physx_pack::SceneSnapshot Scene{};
  Scene.Instances.push_back(Target);

  physx_pack::Ray Ray{};
  Ray.Origin = {0.0f, 100.0f, 0.0f};
  Ray.Direction = {0.0f, -1.0f, 0.0f};
  Ray.MaxDistance = 200.0f;

  const auto Result = Query.Check(Target, Scene, Ray);
  std::cout << static_cast<int>(Result) << '\\n';
}
```

`SceneSnapshot` 必须包含目标以及可能挡住目标的其他实例。每个实例需要完整资源身份、当前世界姿态、缩放和双面标志。目标自身的命中不会被算作阻挡；只有其他实例在目标之前命中时才返回 `Blocked`。

结果含义：

- `Visible`：目标可达，未确认其他实例在前方阻挡。
- `Blocked`：确认其他实例在目标之前命中。
- `Unknown`：包、来源、几何块、姿态、射线或查询无法可靠确定。

`Unknown` 不应改写为 `Blocked`。查询对象和场景数据必须在调用期间保持有效；同一个查询对象可以连续检查多条射线。

## 读取链路

公开源码的调用关系为：

```text
physx.pack
  → PackReader：读取包头、来源索引和数据块
  → GeometryReader：按来源读取 Mesh、Convex 或 HeightField
  → RayQuery：执行几何相交
  → VisibilityQuery：比较目标与阻挡实例的最近命中
```

包解析和几何读取接口位于 `include/physx_pack/`，实现位于 `src/`。它们只处理本地包和本地几何，不读取设备进程内存，不依赖 SQLite 或其他外部资源。

复杂数据按需读取和解压。HeightField 使用包内 signed 16 位 sample 和按 cell 读取的四角数据；实例缩放和世界姿态由调用方在每次查询时提供。

## 文件与生命周期

包在打开时解析索引，运行中替换文件不会自动生效。更新文件后重新创建 `VisibilityQuery` 或重新启动使用它的应用。

解析失败、范围越界、解压失败和几何数据损坏只会导致相关查询返回 `Unknown`。程序不执行 SHA-256、包版本或跨区域 fallback。

## 源码结构

- `include/physx_pack/visibility_query.h`：普通应用使用的最小接口。
- `include/physx_pack/pack_reader.h`：包索引和数据块读取接口。
- `include/physx_pack/geometry_reader.h`：Mesh、Convex、HeightField 读取接口。
- `include/physx_pack/ray_query.h`：几何相交接口。
- `src/`：对应实现。
- `examples/visibility_query_main.cpp`：完整调用示例。

这些接口只覆盖 `physx.pack` 可见性判断，不包含 ESP、Aim、驱动或其他业务逻辑。
