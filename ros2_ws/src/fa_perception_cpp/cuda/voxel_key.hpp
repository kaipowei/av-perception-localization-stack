#pragma once

#include <cmath>
#include <cstdint>

#ifdef __CUDACC__
#define VOXEL_KEY_HD __host__ __device__
#else
#define VOXEL_KEY_HD
#endif

// Shared by the CPU and CUDA downsamplers so both bucket points into voxels
// the exact same way -- otherwise a mismatch between the two wouldn't tell
// you anything about which one is "right."
struct VoxelKey
{
  float voxel_size;

  VOXEL_KEY_HD int64_t operator()(float x, float y, float z) const
  {
    constexpr int64_t kOffset = 1 << 19;
    const int64_t ix = static_cast<int64_t>(floorf(x / voxel_size)) + kOffset;
    const int64_t iy = static_cast<int64_t>(floorf(y / voxel_size)) + kOffset;
    const int64_t iz = static_cast<int64_t>(floorf(z / voxel_size)) + kOffset;
    return (ix << 40) | (iy << 20) | iz;
  }
};
