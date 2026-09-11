# physx-pack

C++17 只读碰撞包与批量可见性查询库。API v2 面向 Linux/WSL 和 Android arm64（API 29+）。

[下载 CN physx.pack](https://github.com/AXIT08/physx-pack/releases/download/cn/physx.pack) · [SHA-256](https://github.com/AXIT08/physx-pack/releases/download/cn/physx.pack.sha256) · [发布清单](https://github.com/AXIT08/physx-pack/releases/download/cn/release-manifest.json)

**API v2 与旧调用方式不兼容，库和调用方必须一起重新编译。** 当前 CN 包有 2,566,538 条 preload，旧读取库的 1,048,576 条上限无法打开。迁移说明见 [MIGRATION.md](MIGRATION.md)。包仍使用 XCPHYSX v1，不是 ZIP 或通用 PhysX SDK 序列化文件。

## 构建

依赖 CMake 3.16+、C++17、zlib 和系统线程库。原生 Windows/MSVC 不属于支持平台，在 Windows 上使用 WSL。

```sh
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build
ctest --test-dir build --output-on-failure
cmake --install build --prefix /your/prefix
```

安装接口只有 `physx_pack/visibility_query.h`；CMake 消费方使用：

```cmake
find_package(physx_pack 2 CONFIG REQUIRED)
target_link_libraries(your_target PRIVATE physx_pack::physx_pack)
```

Android 使用 NDK 的 CMake toolchain，设置 `ANDROID_ABI=arm64-v8a` 和 `ANDROID_PLATFORM=android-29`。合成数据测试不需要模型包；真实资源与生产工具不进入源码仓库。

## 接入

```cpp
#include "physx_pack/visibility_query.h"

physx_pack::Error error{};
auto package = physx_pack::Package::Open("/path/to/physx.pack", {}, &error);
if (!package) { /* log physx_pack::ToString(error) */ return; }

physx_pack::Scene scene(package);
physx_pack::InstanceDesc target{};
target.Id = 1; // Caller-assigned scene instance identity, never a mesh path ID.
target.Resource = {"AlwaysNeed", "0x0123456789abcdef", "CAB-example", 42};
target.MeshType = physx_pack::MeshVariant::Triangle;
target.DoubleSided = true;
if (scene.ApplyUpdates(&target, 1) != physx_pack::Error::None) return;

auto snapshot = scene.GetSnapshot();
physx_pack::QueryContext worker;
physx_pack::QueryRequest request{target.Id, {{0, 100, 0}, {0, -1, 0}, 200}};
physx_pack::VisibilityResult result = worker.Check(snapshot, request);
```

示例四元组是占位符，必须替换为实际包内身份。`ResourceKey` 的 Origin、BundleId、SerializedFile、PathId 必须全部匹配；Origin 不是区域名，例如 CN 包使用 `AlwaysNeed`、`PrimaryPack`，恢复的内置资源使用 `apk`。有符号 Unity PathID 使用其 uint64 位表示。

`InstanceId` 是非零且场景内唯一的实例标识。同一资源可以被多个实例复用；查询只排除目标自身的 InstanceId，其余副本仍可阻挡。目标姿态与几何变体只取自当前快照，不再另传一份目标对象。

对接 MeshCollider 时按 `m_Convex` 选择 `MeshVariant::Convex` 或 `Triangle`。`Automatic` 使用包中默认变体；指定变体缺失时返回 `Unknown/MissingVariant`。HeightField 使用 Automatic，缩放映射为 X 行、Y 高度、Z 列；非零负缩放有效。

## 场景与批量查询

`ApplyUpdates(upserts, count, removals, removal_count)` 原子应用一批变更。新增和已有实例都使用 upserts；不存在的删除ID返回 TargetNotFound。ID重复、增删重叠、空资源字段、零缩放或无效姿态会拒绝整批操作。零操作不改变 generation。资源不存在属于合法的未解析实例，随后查询返回明确原因。

快照不可变，持有包与实例数据。场景更新和包包装对象释放不影响既有快照。增删实例或可界定状态变化会重建场景 BVH；仅姿态变化复用几何描述并 refit。三角/地形使用可靠包围盒；仅有凸包平面而缺少可靠边界的资源保守查询。

```cpp
physx_pack::QueryRequest requests[2] = {/* target IDs and rays */};
physx_pack::VisibilityResult results[2];
worker.CheckBatch(snapshot, requests, 2, results);
```

一个 QueryContext 只能由一个工作线程使用；多个上下文可并发查询同一个快照。场景更新内部串行化，读取快照可以并发。库不创建工作线程，不做远程读取或设备访问。批量接口的缓冲区由调用方提供，单项失败记录在相应结果中；Count 为零时允许空指针。

## 结果语义

- `Visible`：射线命中指定目标，且目标之前没有阻挡或无法排除的未知实例。
- `Blocked`：已确认其他实例不晚于目标命中。即使另有未解析实例，也不会丢弃已证实的阻挡。
- `Unknown`：目标未命中、目标缺失、几何缺失、数据错误或资源不足。Reason 和 ProblemInstance 指出原因；不要把 Unknown 改写成 Blocked。

结果还包含 TargetDistance、BlockingInstance、BlockingDistance。没有相应命中时距离为无穷大、实例ID为零。遮挡距离相等时按较小实例ID稳定选择。

射线方向只需非零且有限，库内部归一化；MaxDistance 始终是世界坐标长度且必须有限、严格大于零。四元数会规范化；全零或非有限四元数被拒绝。不可确定的实例只有在可靠包围盒证明不影响目标前射线段时才被排除。

## 缓存与生命周期

默认128 MiB只限制解码块缓存，不是整个进程的内存上限。`PackageOptions::CacheBudgetBytes` 可调整，零预算可用于诊断；单块格式读取上限仍为128 MiB。块被查询固定期间不会被回收。预算不足返回 BudgetExceeded，不冒充 CorruptData。

`GetInfo()` 报告包大小、索引数量和估算索引内存；`GetCacheStats()` 报告共享缓存、解压临时内存和几何元数据；`GetStats()` 提供场景构建/refit及上下文查询计数。热查询复用几何元数据和线程工作区。

包文件必须保持不变。更新文件时使用原子替换，再打开新的 Package、构建新场景；旧快照继续持有旧资源。库不在查询期间校验整包 SHA-256，下载完整性由发布清单和调用方验证。

## 当前 CN 数据边界

当前包 290,507,249 bytes，SHA-256 为 `04f86b264c2c63f14859e8a002668b7301f218ba55dbf8b6cec54cbbf6520f50`。包含10,050份三角载荷、600份凸包载荷和259份地形，共10,876个可查询资源身份。

127份载荷由原始 Mesh 使用官方 PhysX 4.1.2 重新烘焙；它们在20个基线上解码结果一致，但不代表所有运行时行为已经证明一致。仍有1个凸包触发面数限制、2个失效目标PathID、1个缺失CAB；另外有未被当前静态MeshCollider引用的Mesh没有baked数据。数据覆盖状态为 **partial**，运行时世界未验证。

89,486条静态MeshCollider记录中89,406条所需几何可用、74条原本没有Mesh，另外6条受上述几何或引用缺口影响。验证通过表示编码、索引或指定测试通过，不等于资源覆盖完整。模型包和脱敏验证摘要通过Release附件分发。
