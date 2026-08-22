#include "voxel_key.hpp"

#include <cstdint>
#include <unordered_map>

// CPU reference implementation, same bucketing rule as the GPU version
// (voxel_key.hpp), used to (a) sanity-check the GPU output and (b) give the
// benchmark something to compare wall-clock time against.
extern "C" int voxel_downsample_cpu(
  const float * in_xyz, int count, float voxel_size, float * out_xyz, int max_out)
{
  if (count <= 0) {return 0;}

  VoxelKey key_of{voxel_size};
  std::unordered_map<int64_t, int> first_index_by_voxel;
  first_index_by_voxel.reserve(count);

  for (int i = 0; i < count; ++i) {
    const int64_t key = key_of(in_xyz[i * 3 + 0], in_xyz[i * 3 + 1], in_xyz[i * 3 + 2]);
    first_index_by_voxel.emplace(key, i);
  }

  int out_count = 0;
  for (const auto & [key, index] : first_index_by_voxel) {
    if (out_count >= max_out) {break;}
    out_xyz[out_count * 3 + 0] = in_xyz[index * 3 + 0];
    out_xyz[out_count * 3 + 1] = in_xyz[index * 3 + 1];
    out_xyz[out_count * 3 + 2] = in_xyz[index * 3 + 2];
    ++out_count;
  }
  return out_count;
}
