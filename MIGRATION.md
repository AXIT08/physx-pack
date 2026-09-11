# API v1 到 v2

API v2需要重新编译库与调用方。包格式仍为XCPHYSX v1，新读取器支持旧包；新CN包包含超过250万条preload，旧读取器无法打开。

| v1 | v2 |
| --- | --- |
| `VisibilityQuery::Open(path)` | `Package::Open(path, options, &error)` |
| `TargetId` | `ResourceKey`，仍是资源四元组 |
| `TargetInstance` | `InstanceDesc`，增加独立的非零 `InstanceId` |
| `SceneSnapshot.Instances` | `Scene::ApplyUpdates()` 后调用 `GetSnapshot()` |
| `Check(target, scene, ray)` | `QueryContext::Check(snapshot, {instance_id, ray})` |
| 仅 `VisibilityState` | `VisibilityResult`，含三态、Reason、距离和相关实例ID |
| 隐藏的单位射线方向约束 | 方向内部归一化，MaxDistance为世界单位 |
| 共用一个查询对象 | 每工作线程独立QueryContext，共享不可变快照 |

更新实例时保持其InstanceId稳定，资源四元组与场景实例ID用途不同。同资源的多个副本必须使用不同ID。删除实例在ApplyUpdates的removals数组中指定ID。

按实际MeshCollider的m_Convex设置MeshType；Automatic只沿用包默认类型。HeightField使用Automatic。实例的资源缺失与指定变体缺失都保留为Unknown，不使用渲染网格或其他变体补位。

底层PackReader、GeometryReader、RayQuery及诊断头现为内部实现，不再从include/physx_pack发布或安装。旧代码应迁移到公共头visibility_query.h。没有旧API转发层，不依赖旧头文件的内存布局或枚举值。

新的Package通过共享所有权保持资源生命周期；旧快照独立于Scene后续更新。Scene构造和QueryContext构造可能抛出std::bad_alloc；Open、ApplyUpdates和查询接口返回Error或Unknown，并捕获内部资源分配失败。

新版修正同资源多实例识别、HeightField重复缩放及已确认阻挡被未知实例覆盖的问题，因此这些场景的结果会与v1不同。源资源缺口继续存在，调用方必须保留Unknown状态。

即便输入方向近似单位向量，规范化也可能改变末位浮点数。恰好贴着三角边缘的射线可能因此改变命中结果。5,120个基线中观察到两例这种变化；对v1施加相同方向规范化后，全部结果与v2一致。
