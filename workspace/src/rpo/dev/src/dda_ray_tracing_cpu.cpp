#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>



bool checkOccupancy(const int* occupancy_grid, int x, int y, int z, int dim_x, int dim_y, int dim_z);

bool castRayDDA(const int* occupancy_grid, int dim_x, int dim_y, int dim_z, float resolution, 
    double grid_origin_x, double grid_origin_y, double grid_origin_z, float ox, float oy, float oz, 
    float dx, float dy, float dz, float max_range, int target_x, int target_y, int target_z);

void runRayTracing(const int* occupancy_grid, int dim_x, int dim_y, int dim_z, float resolution,
    double grid_origin_x, double grid_origin_y, double grid_origin_z, float ox, float oy, float oz,
    float max_range, float* targets, int num_rays, int* hit_mask);



int main (int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: rosrun rpo CPU_DDA <octomap.ot>" << std::endl;
        return 1;
    }

    // Read Octomap
    const std::string filename = argv[1];
    std::unique_ptr<octomap::ColorOcTree> tree = nullptr;
    tree.reset(dynamic_cast<octomap::ColorOcTree*>(octomap::AbstractOcTree::read(filename)));

    // Basic parameters
    octomap::point3d origin(0.025f, 0.025f, 0.325f);
    float resolution = tree->getResolution();
    float max_range = 50.0;


    // Target generation
    auto target_start = std::chrono::high_resolution_clock::now();

    std::vector<float> targets;

    for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it)
    {
        octomap::point3d target = it.getCoordinate();

        targets.push_back(target.x());
        targets.push_back(target.y());
        targets.push_back(target.z());
    }

    int n_targets = targets.size() / 3;

    auto target_end = std::chrono::high_resolution_clock::now();
    auto target_duration = std::chrono::duration_cast<std::chrono::microseconds>(target_end - target_start);


    // Creating 1D occupancy grid
    auto grid_start = std::chrono::high_resolution_clock::now();

    int total_voxels = 0;

    double min_x, min_y, min_z;
    double max_x, max_y, max_z;
    tree->getMetricMin(min_x, min_y, min_z);
    tree->getMetricMax(max_x, max_y, max_z);

    int dim_x = std::ceil((max_x - min_x) / resolution);
    int dim_y = std::ceil((max_y - min_y) / resolution);
    int dim_z = std::ceil((max_z - min_z) / resolution);

    std::vector<int> occupancy_grid(dim_x * dim_y * dim_z, 0);

    for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it)
    {
        if (tree->isNodeOccupied(*it))
        {
            octomap::point3d coord = it.getCoordinate();
            int x = static_cast<int>((coord.x() - min_x) / resolution);
            int y = static_cast<int>((coord.y() - min_y) / resolution);
            int z = static_cast<int>((coord.z() - min_z) / resolution);
            occupancy_grid[x + y * dim_x + z * dim_x * dim_y] = 1;

            ++total_voxels;
        }
    }

    auto grid_end = std::chrono::high_resolution_clock::now();
    auto grid_duration = std::chrono::duration_cast<std::chrono::microseconds>(grid_end - grid_start);



    // GT ray tracing
    auto gt_start = std::chrono::high_resolution_clock::now();

    int gt_hit_count = 0;

    #pragma omp parallel for
    for (size_t idx = 0; idx < n_targets; ++idx)
    {
        octomap::point3d target = octomap::point3d(targets[3*idx], targets[3*idx+1], targets[3*idx+2]);
        octomap::point3d end_point;

        bool hit = tree->castRay(origin, (target - origin), end_point, true, 1.1 * (target - origin).norm());

        if (hit)
        {
            if ((end_point - target).norm() < resolution * 0.5)
            {
                ++gt_hit_count;
            }
        }
    }

    auto gt_end = std::chrono::high_resolution_clock::now();
    auto gt_duration = std::chrono::duration_cast<std::chrono::microseconds>(gt_end - gt_start);

    std::cout << gt_hit_count << " visible voxels out of " << total_voxels << " are computed in " << (target_duration.count() + gt_duration.count()) << " microseconds." << std::endl;



    // Basic DDA ray tracing on CPU
    auto dda_start = std::chrono::high_resolution_clock::now();
    int dda_hit_count = 0;

    #pragma omp parallel for
    for (size_t idx = 0; idx < n_targets; ++idx)
    {
        float tx = targets[3 * idx + 0];
        float ty = targets[3 * idx + 1];
        float tz = targets[3 * idx + 2];

        float dx = tx - origin.x();
        float dy = ty - origin.y();
        float dz = tz - origin.z();

        int target_x = (tx - min_x) / resolution;
        int target_y = (ty - min_y) / resolution;
        int target_z = (tz - min_z) / resolution;

        if (castRayDDA(occupancy_grid.data(), dim_x, dim_y, dim_z, resolution, min_x, min_y, min_z, 
            origin.x(), origin.y(), origin.z(), dx, dy, dz, max_range, target_x, target_y, target_z)) ++dda_hit_count;
    }

    auto dda_end = std::chrono::high_resolution_clock::now();
    auto dda_duration = std::chrono::duration_cast<std::chrono::microseconds>(dda_end - dda_start);

    std::cout << dda_hit_count << " visible voxels out of " << total_voxels << " are computed in " << target_duration.count() + grid_duration.count() + dda_duration.count() << " microseconds." << std::endl;



    // DDA ray tracing on CPU with "kernel"
    auto cpu_start = std::chrono::high_resolution_clock::now();
    
    int* cpu_hit_mask = new int[n_targets];


    auto kernel_start = std::chrono::high_resolution_clock::now();
    
    runRayTracing(occupancy_grid.data(), dim_x, dim_y, dim_z, resolution, min_x, min_y, min_z,
        origin.x(), origin.y(), origin.z(), max_range, targets.data(), n_targets, cpu_hit_mask);

    auto kernel_end = std::chrono::high_resolution_clock::now();
    auto kernel_duration = std::chrono::duration_cast<std::chrono::microseconds>(kernel_end - kernel_start);

    std::cout << "Kernel time: " << kernel_duration.count() << std::endl;



    int cpu_hit_count = 0;

    for (size_t i = 0; i < n_targets; ++i)
    {
        if (cpu_hit_mask[i] == 1) ++cpu_hit_count;
    }

    auto cpu_end = std::chrono::high_resolution_clock::now();
    auto cpu_duration = std::chrono::duration_cast<std::chrono::microseconds>(cpu_end - cpu_start);

    std::cout << cpu_hit_count << " visible voxels out of " << total_voxels << " are computed in " << (target_duration.count() + grid_duration.count() + cpu_duration.count()) << " microseconds." << std::endl;
}




bool checkOccupancy(const int* occupancy_grid, int x, int y, int z, int dim_x, int dim_y, int dim_z)
{
    if (x < 0 || y < 0 || z < 0 || x >= dim_x || y >= dim_y || z >= dim_z)
    {
        return false;
    }

    int idx = x + y * dim_x + z * dim_x * dim_y;
    return (occupancy_grid[idx] != 0);
}



bool castRayDDA(const int* occupancy_grid, int dim_x, int dim_y, int dim_z, float resolution, 
    double grid_origin_x, double grid_origin_y, double grid_origin_z, float ox, float oy, float oz, 
    float dx, float dy, float dz, float max_range, int target_x, int target_y, int target_z)
{
    int x = static_cast<int>((ox - grid_origin_x) / resolution);
    int y = static_cast<int>((oy - grid_origin_y) / resolution);
    int z = static_cast<int>((oz - grid_origin_z) / resolution);


    float length = std::sqrt(dx * dx + dy * dy + dz * dz);
    dx /= length; dy /= length; dz /= length;

    float voxel_size = resolution;
    
    float t_delta_x = voxel_size / std::abs(dx);
    float t_delta_y = voxel_size / std::abs(dy);
    float t_delta_z = voxel_size / std::abs(dz);

    int step_x = (dx > 0) ? 1 : -1;
    int step_y = (dy > 0) ? 1 : -1;
    int step_z = (dz > 0) ? 1 : -1;

    float voxel_border_x = (x + (step_x > 0 ? 1 : 0)) * voxel_size + grid_origin_x;
    float voxel_border_y = (y + (step_y > 0 ? 1 : 0)) * voxel_size + grid_origin_y;
    float voxel_border_z = (z + (step_z > 0 ? 1 : 0)) * voxel_size + grid_origin_z;

    float t_max_x = (voxel_border_x - ox) / dx;
    float t_max_y = (voxel_border_y - oy) / dy;
    float t_max_z = (voxel_border_z - oz) / dz;
    
    float t = 0.0f;

    while (t < max_range)
    {
        if (checkOccupancy(occupancy_grid, x, y, z, dim_x, dim_y, dim_z))
        {
            return (x == target_x && y == target_y && z == target_z);
        }

        if (t_max_x < t_max_y)
        {
            if (t_max_x < t_max_z)
            {
                x += step_x;
                t = t_max_x;
                t_max_x += t_delta_x;
            }
            else
            {
                z += step_z;
                t = t_max_z;
                t_max_z += t_delta_z;
            }
        }
        else
        {
            if (t_max_y < t_max_z)
            {
                y += step_y;
                t = t_max_y;
                t_max_y += t_delta_y;
            }
            else
            {
                z += step_z;
                t = t_max_z;
                t_max_z += t_delta_z;
            }
        }


        if (x < 0 || y < 0 || z < 0 || x >= dim_x || y >= dim_y || z >= dim_z)
        {
            return false;
        }
    }

    return false;
}



void runRayTracing(const int* occupancy_grid, int dim_x, int dim_y, int dim_z, float resolution,
    double grid_origin_x, double grid_origin_y, double grid_origin_z, float ox, float oy, float oz,
    float max_range, float* targets, int num_rays, int* hit_mask)
{
    #pragma omp parallel for
    for (size_t idx = 0; idx < num_rays; ++idx)
    {
        float tx = targets[3 * idx + 0];
        float ty = targets[3 * idx + 1];
        float tz = targets[3 * idx + 2];

        float dx = tx - ox;
        float dy = ty - oy;
        float dz = tz - oz;

        int target_x = (tx - grid_origin_x) / resolution;
        int target_y = (ty - grid_origin_y) / resolution;
        int target_z = (tz - grid_origin_z) / resolution;

        bool hit = castRayDDA(
            occupancy_grid, dim_x, dim_y, dim_z,
            resolution,
            grid_origin_x, grid_origin_y, grid_origin_z,
            ox, oy, oz,
            dx, dy, dz,
            max_range,
            target_x, target_y, target_z);

        hit_mask[idx] = hit ? 1 : 0;
    }    
}




