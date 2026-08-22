#include <thrust/device_vector.h>
#include <thrust/host_vector.h>
#include <thrust/sequence.h>
#include <thrust/sort.h>
#include <thrust/unique.h>

#include <algorithm>
#include <cstdint>

#include "voxel_key.hpp"

namespace
{
__global__ void computeKeysKernel(
  const float * xyz, int count, float voxel_size, int64_t * keys)
{
  int i = blockIdx.x * blockDim.x + threadIdx.x;
  if (i >= count) {return;}
  VoxelKey key_of{voxel_size};
  keys[i] = key_of(xyz[i * 3 + 0], xyz[i * 3 + 1], xyz[i * 3 + 2]);
}
}  // namespace

// Downsamples `count` XYZ points (packed x0,y0,z0,x1,y1,z1,...) to at most
// one point per voxel_size cube. Writes up to max_out points into out_xyz
// and returns how many it actually wrote. Which point survives per voxel is
// whichever one the GPU's sort happens to place first, not a centroid —
// good enough for a first pass; averaging per voxel is a natural follow-up.
extern "C" int voxel_downsample(
  const float * in_xyz, int count, float voxel_size, float * out_xyz, int max_out)
{
  if (count <= 0) {return 0;}

  thrust::device_vector<float> d_in(in_xyz, in_xyz + count * 3);
  thrust::device_vector<int64_t> d_keys(count);

  const int block = 256;
  const int grid = (count + block - 1) / block;
  computeKeysKernel<<<grid, block>>>(
    thrust::raw_pointer_cast(d_in.data()), count, voxel_size,
    thrust::raw_pointer_cast(d_keys.data()));

  thrust::device_vector<int> d_idx(count);
  thrust::sequence(d_idx.begin(), d_idx.end());
  thrust::sort_by_key(d_keys.begin(), d_keys.end(), d_idx.begin());

  thrust::device_vector<int64_t> d_unique_keys(count);
  thrust::device_vector<int> d_unique_idx(count);
  auto end = thrust::unique_by_key_copy(
    d_keys.begin(), d_keys.end(), d_idx.begin(),
    d_unique_keys.begin(), d_unique_idx.begin());
  const int unique_count = static_cast<int>(end.first - d_unique_keys.begin());

  const int out_count = std::min(unique_count, max_out);
  thrust::host_vector<int> h_idx(d_unique_idx.begin(), d_unique_idx.begin() + out_count);
  thrust::host_vector<float> h_in(d_in.begin(), d_in.end());
  for (int i = 0; i < out_count; ++i) {
    out_xyz[i * 3 + 0] = h_in[h_idx[i] * 3 + 0];
    out_xyz[i * 3 + 1] = h_in[h_idx[i] * 3 + 1];
    out_xyz[i * 3 + 2] = h_in[h_idx[i] * 3 + 2];
  }
  return out_count;
}
