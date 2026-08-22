#include <chrono>
#include <cstdio>
#include <random>
#include <vector>

extern "C" int voxel_downsample(const float *, int, float, float *, int);
extern "C" int voxel_downsample_cpu(const float *, int, float, float *, int);

namespace
{
std::vector<float> makeRandomCloud(int n, unsigned seed)
{
  std::mt19937 rng(seed);
  std::uniform_real_distribution<float> xy(-20.0f, 20.0f);
  std::uniform_real_distribution<float> z(-1.0f, 1.0f);
  std::vector<float> xyz(n * 3);
  for (int i = 0; i < n; ++i) {
    xyz[i * 3 + 0] = xy(rng);
    xyz[i * 3 + 1] = xy(rng);
    xyz[i * 3 + 2] = z(rng);
  }
  return xyz;
}

template<typename Fn>
double timedMs(Fn && fn)
{
  const auto t0 = std::chrono::steady_clock::now();
  fn();
  const auto t1 = std::chrono::steady_clock::now();
  return std::chrono::duration<double, std::milli>(t1 - t0).count();
}
}  // namespace

int main()
{
  const float voxel_size = 0.2f;

  // The first CUDA call in a process pays for lazy context init (can be tens
  // of ms); absorb that here so it doesn't pollute the first real
  // measurement below.
  {
    std::vector<float> warm(3, 0.0f), warm_out(3);
    voxel_downsample(warm.data(), 1, voxel_size, warm_out.data(), 1);
  }

  printf(
    "%10s %12s %12s %10s %10s %9s\n",
    "points", "cpu_ms", "gpu_ms", "cpu_out", "gpu_out", "speedup");
  for (int n : {10000, 100000, 500000, 1000000}) {
    auto cloud = makeRandomCloud(n, 42);
    std::vector<float> cpu_out(n * 3), gpu_out(n * 3);
    int cpu_count = 0, gpu_count = 0;

    const double cpu_ms = timedMs(
      [&] {cpu_count = voxel_downsample_cpu(cloud.data(), n, voxel_size, cpu_out.data(), n);});
    const double gpu_ms = timedMs(
      [&] {gpu_count = voxel_downsample(cloud.data(), n, voxel_size, gpu_out.data(), n);});

    printf(
      "%10d %12.3f %12.3f %10d %10d %8.2fx",
      n, cpu_ms, gpu_ms, cpu_count, gpu_count, cpu_ms / gpu_ms);
    if (cpu_count != gpu_count) {printf("  MISMATCH");}
    printf("\n");
  }
  return 0;
}
