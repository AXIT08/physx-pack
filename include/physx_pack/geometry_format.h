#ifndef PHYSX_PACK_GEOMETRY_FORMAT_H
#define PHYSX_PACK_GEOMETRY_FORMAT_H

#include <cstddef>
#include <cstdint>

namespace physx_pack
{
namespace detail
{

// 文件内偏移只用于本地数据解析，不构造远程地址。
inline constexpr std::uint32_t KPackMeshRootMagic = 0x524D4358U;
inline constexpr std::uint32_t KPackTriangleMagic = 0x544D4358U;
inline constexpr std::uint32_t KPackConvexMagic = 0x504D4358U;
inline constexpr std::uint32_t KPackHeightFieldMagic = 0x46484358U;
inline constexpr std::uint32_t KPackHitboxSlot = 0xFFFFFFF0U;
inline constexpr std::uint32_t KPackFirstDataBlock = 0x01000000U;
inline constexpr std::size_t KPackMeshRootBytes = 16U;
inline constexpr std::size_t KPackTriangleHeaderBytes = 80U;
inline constexpr std::size_t KPackConvexHeaderBytes = 16U;
inline constexpr std::size_t KPackHeightFieldHeaderBytes = 48U;
inline constexpr std::uint32_t KPackTerrainTileSide = 64U;

// Mesh 根：magic、三角形块号、凸包块号、保留字（4 个 u32）。
// 三角形头：magic/树型/顶点数/三角形数/节点数/根/索引宽度（u32），
// epsilon（f32），顶点/索引/节点偏移（u64），量化系数（6 个 f32）。
// RTree 保留 112 字节的六组轴数据和四个 child；BV4 保留 64 字节量化块。
// 凸包头：magic/数量（u32）、平面偏移（u64），平面为 nx/ny/nz/d（f32）。
// 高度场头：magic/行/列/块边长（u32），最小/最大高度（i32），
// 块行/列（u32），块号数组偏移（u64），保留字（u64）。
// 地形块按行存放 PxHeightFieldSample（i16/u8/u8）；边缘块不补齐。

} // namespace detail
} // namespace physx_pack

#endif
