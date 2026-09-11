# XC-SKJH Read 帧接入示例

本文说明如何把 XC-SKJH 的 Read 帧数据转换为公开的 PhysX v2 接口。XC-SKJH 仅作为调用方语义参考；示例不包含、也不能直接 `include` XC-SKJH 私有头文件。公开库只负责包内几何查询，实体 owner、detail、ScreenBox、HitBox 和最终 Aim 选点仍由 XC-SKJH 业务层处理。

## 数据映射

| XC-SKJH 来源 | 公开适配层字段 | 说明 |
| --- | --- | --- |
| `ReadFrameAssembler::Snapshot.CameraPosition` | `ReadFrameInput.Camera` | 相机来自 assembler 的当前帧快照。 |
| `VisibilityTargetSample.SourceSequence` | `ReadFrameInput.Sequence` | 保留帧序列，用于拒绝过期结果。 |
| `VisibilityTargetSample.PlanRevision` | `ReadFrameInput.PlanRevision` | 保留计划版本；版本不一致时提交失败，旧快照继续有效。 |
| `VisibilityTargetSample.Bones` | `ReadFrameInput.Bones[16]` | 16 个骨点位置；由调用方提供活动掩码。 |
| `VisibilityTargetSample.Key.Kind/EntityId/Obj` | `ReadFrameEntityKey` / `ReadFrameShape.ConsumerKey` / `ReadFrameInput.Target` | 必须包含实体类型、实体 ID 和对象令牌，不能用 Mesh `PathId` 代替实例身份。适配器内部将其映射为稳定 `InstanceId`。 |
| `SampleVisibilitySceneOnce` 过滤后的 static/dynamic/compound | `ReadFrameShape` | 先沿用 XC-SKJH 的 layer、trigger 和有效性过滤，再逐个提交。 |
| `CollisionRuntimeBinding::Lookup` | `ResourceKey` | 解析结果必须包含 Origin、BundleId、SerializedFile、PathId 四元组；适配层不接受裸 geometry slot。 |

同一资源的多个场景实例必须使用不同的非零 `InstanceId`。查询会排除目标自身的实例 ID，但同资源的其他实例仍然可以阻挡射线。

## 真实调用顺序

适配层的生命周期对应 XC-SKJH 的一帧：启动时打开一次包；每帧开始时提交帧元数据和骨点；把已经过滤的形状加入本次事务；成功发布快照后，再对目标实例执行查询。

```cpp
#include "physx_pack/visibility_query.h"
#include "xc_read_frame_adapter.h"  // 公开示例头，不是 XC-SKJH 私有头
#include <memory>
#include <utility>
#include <vector>

using namespace physx_pack;
using namespace physx_pack::xc_read;

std::shared_ptr<Package> OpenPhysxPackage(const char* path) {
  Error error{};
  auto package = Package::Open(path, {}, &error);
  if (!package) {
    // 记录 ToString(error)，不要把打开失败当作没有遮挡。
    return {};
  }
  return package;
}

void ResolveFrameVisibilityForRead(
    ReadFramePhysxAdapter& adapter,
    const ReadFrameAssembler::Snapshot& snapshot,
    const VisibilityTargetSample& target,
    const std::vector<ReadFrameShape>& filteredShapes) {
  if (adapter.BeginFrame(target.SourceSequence,
                         target.PlanRevision,
                         snapshot.Camera,
                         target.Bones,
                         target.ActiveMask) != Error::None) {
    return;
  }

  for (const auto& shape : filteredShapes) {
    // shape 中的 ConsumerKey、完整 ResourceKey、WorldPose、Scale、
    // DoubleSided 和 MeshVariant 均来自当前 Read 帧的已解析结果。
    if (adapter.UpsertShape(shape) != Error::None) {
      // 当前事务失败；不能继续使用半提交场景。
      return;
    }
  }

  if (adapter.EndFrame() != Error::None) {
    // 重复 ID、帧序列倒退、计划版本冲突或无效姿态会保留旧快照。
    return;
  }

  const ReadFrameVisibility result = adapter.Resolve();
  // result.Sequence/PlanRevision 与输入一致，逐骨点结果保留
  // CompletedMask、Visible/Blocked/Unproven、距离和阻挡实例。
  // 映射回 XC-SKJH 的 EntityVisibilityResult 后，再由业务层处理 owner、
  // ColliderConfig、ScreenBox、HitBox 和 Aim 选点。
}

// 应用启动时打开一次包，并在每个 Read 帧调用上面的函数。
std::unique_ptr<ReadFramePhysxAdapter> StartReadPhysx() {
  auto package = OpenPhysxPackage("/data/physx.pack");
  if (!package)
    return {};
  return std::make_unique<ReadFramePhysxAdapter>(std::move(package));
}
```

调用方应把 `ResolveFrameVisibilityForRead` 放在 `ReadFrameAssembler::ResolveFrameVisibility` 的同一帧阶段。目标键在 `ReadFrameInput.Target` 中随帧提交，由 `Resolve()` 查询对应实例；适配器不是对任意骨点线段宣称完整可见的通用遮挡器。每个骨点射线都会排除目标自身实例，目标未命中、资源缺失、变体缺失或包错误会返回 `Unknown` 及明确原因。

## 几何与变体规则

- `m_Convex` 为真时使用 `MeshVariant::Convex`，否则使用 `MeshVariant::Triangle`；明确指定的变体不存在时保持 `Unknown`，不得静默切换到另一种几何。
- HeightField 使用 `MeshVariant::Automatic`，世界缩放由库按 X 行、Y 高度、Z 列处理。
- `ResourceKey` 四元组必须与包索引完全匹配。Origin 是资源来源标识，不是区域名；CN 包常见值为 `AlwaysNeed`、`PrimaryPack`，恢复的内置资源为 `apk`。
- `InstanceId` 在场景内非零且唯一；同一资源的不同姿态或不同实体不能共享 ID。

## 结果边界

`Visible` 表示射线命中指定目标，且目标之前没有已确认的阻挡；`Blocked` 表示已确认其他实例不晚于目标命中；`Unknown` 表示目标未命中或无法可靠判断。若适配器对三态做业务层命名映射，可将 `Unknown` 显示为 `Unproven`，但不得伪造零命中 `Visible` 或宣称覆盖完整。

未解析实例只有在可靠包围盒证明不影响目标前射线段时才会被排除。适配器不得访问远程设备、进程地址或 XC-SKJH 的认证/读取实现。

## 验证

公开仓库中的 `xc_read_frame_adapter` 使用合成 `ReadFrameInput`，因此可以独立编译和测试；它不携带 XC-SKJH 原始帧、设备数据或私有类型。接入 XC-SKJH 时，应在业务层完成类型转换后再调用适配器，并验证同资源多实例遮挡、部分活动骨点、过期序列和资源缺口等场景。
