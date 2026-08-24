#include "voxel_map.h"

#include <iostream>
#include <stdexcept>

int main()
{
  try
  {
    VoxelMapConfig config{};
    std::unordered_map<VOXEL_LOCATION, VoxelOctoTree *> external_map;
    VoxelMapManager manager(config, external_map);
    manager.voxel_map_[VOXEL_LOCATION(0, 0, 0)] =
        new VoxelOctoTree(1, 0, 1, 10, 0.01F);
    manager.voxel_map_[VOXEL_LOCATION(1, 0, 0)] =
        new VoxelOctoTree(1, 0, 1, 10, 0.01F);

    StatesGroup seed;
    seed.pos_end = V3D(4.0, -2.0, 1.0);
    seed.vel_end = V3D(3.0, 0.5, -0.1);
    seed.gravity = V3D(0.0, 0.0, -9.81);
    const std::size_t deleted = manager.ResetLocalMap(seed);
    if (deleted != 2 || !manager.voxel_map_.empty() ||
        (manager.state_.pos_end - seed.pos_end).norm() > 1.0e-12 ||
        (manager.state_.vel_end - seed.vel_end).norm() > 1.0e-12 ||
        (manager.last_slide_position - seed.pos_end).norm() > 1.0e-12)
      throw std::runtime_error(
          "local voxel reset did not preserve its seed state");
  }
  catch (const std::exception &error)
  {
    std::cerr << "voxel_map_reset_test failed: " << error.what() << '\n';
    return 1;
  }
  std::cout << "voxel_map_reset_test passed\n";
  return 0;
}
