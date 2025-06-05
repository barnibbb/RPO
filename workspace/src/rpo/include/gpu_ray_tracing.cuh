#pragma once

#include <vector>

void runKernel(
    const std::vector<int>& occupancy_grid, 
    const std::vector<float>& origins, 
    const std::vector<float>& targets,
    std::vector<int>& visibility_map, 
    int dim_x, int dim_y, int dim_z,
    float grid_x, float grid_y, float grid_z,
    float resolution, float max_range,
    int num_origins, int num_targets
);

