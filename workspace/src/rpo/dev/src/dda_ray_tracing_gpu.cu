#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>

#include <cuda_runtime.h>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>


__device__ bool isOccupied(const int* occupancy_grid, int x, int y, int z, int dim_x, int dim_y, int dim_z);

__device__ bool castRayDDAGPU(const int* occupancy_grid, int dim_x, int dim_y, int dim_z, float resolution,
    double grid_origin_x, double grid_origin_y, double grid_origin_z, float ox, float oy, float oz,
    float dx, float dy, float dz, float max_range, int target_x, int target_y, int target_z);

__global__ void rayTraceKernel(const int* occupancy_grid, int dim_x, int dim_y, int dim_z,
    float resolution, double grid_origin_x, double grid_origin_y, double grid_origin_z,
    float* origins, int num_origins, float max_range, float* targets, int num_targets, int* hit_mask);


int main (int argc, char** argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: rosrun rpo GPU_DDA <octomap.ot>" << std::endl;
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

    // Multiple origins
    int n_origins = 24;

    std::vector<float> origins(3 * n_origins);

    for (int i = 0; i < n_origins; ++i)
    {
        origins[3 * i + 0] = origin.x();
        origins[3 * i + 1] = origin.y();
        origins[3 * i + 2] = origin.z() + i * resolution;
    }


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
            ++total_voxels;

            octomap::point3d coord = it.getCoordinate();
            int x = static_cast<int>((coord.x() - min_x) / resolution);
            int y = static_cast<int>((coord.y() - min_y) / resolution);
            int z = static_cast<int>((coord.z() - min_z) / resolution);
            occupancy_grid[x + y * dim_x + z * dim_x * dim_y] = 1;
        }
    }

    auto grid_end = std::chrono::high_resolution_clock::now();
    auto grid_duration = std::chrono::duration_cast<std::chrono::microseconds>(grid_end - grid_start);



    // DDA ray tracing on GPU
    auto gpu_start = std::chrono::high_resolution_clock::now();

    std::vector<float> targets;

    for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it)
    {
        octomap::point3d target = it.getCoordinate();

        targets.push_back(target.x());
        targets.push_back(target.y());
        targets.push_back(target.z());
    }

    int n_targets = targets.size() / 3;
    int total_rays = n_origins * n_targets;

    std::vector<int> gpu_hit_mask(total_rays);


    int* d_grid;
    cudaMalloc(&d_grid, occupancy_grid.size() * sizeof(int));
    cudaMemcpy(d_grid, occupancy_grid.data(), occupancy_grid.size() * sizeof(int), cudaMemcpyHostToDevice);
    

    float* d_origins;
    cudaMalloc(&d_origins, 3 * n_origins * sizeof(float));
    cudaMemcpy(d_origins, origins.data(), 3 * n_origins * sizeof(float), cudaMemcpyHostToDevice);


    float* d_targets;
    cudaMalloc(&d_targets, 3 * n_targets * sizeof(float));
    cudaMemcpy(d_targets, targets.data(), 3 * n_targets * sizeof(float), cudaMemcpyHostToDevice);


    int* d_results;
    cudaMalloc(&d_results, total_rays * sizeof(int));

    int threadsPerBlock = 256;
    int blocks = (total_rays + threadsPerBlock - 1) / threadsPerBlock;

    rayTraceKernel<<<blocks, threadsPerBlock>>>(
        d_grid, dim_x, dim_y, dim_z, resolution,
        min_x, min_y, min_z, d_origins, n_origins,
        max_range, d_targets, n_targets, d_results
    );

    cudaDeviceSynchronize();

    cudaMemcpy(gpu_hit_mask.data(), d_results, total_rays * sizeof(int), cudaMemcpyDeviceToHost);

    cudaFree(d_grid);
    cudaFree(d_origins);
    cudaFree(d_targets);
    cudaFree(d_results);

    int gpu_hit_count = 0;

    for (size_t i = 0; i < total_rays; ++i)
    {
        if (gpu_hit_mask[i] == 1) ++gpu_hit_count;
    }

    std::vector<int> target_reached(n_targets, 0);

    for (int origin_idx = 0; origin_idx < n_origins; ++origin_idx)
    {
        for (int target_idx = 0; target_idx < n_targets; ++target_idx)
        {
            int idx = origin_idx * n_targets + target_idx;
            if (gpu_hit_mask[idx] == 1)
            {
                target_reached[target_idx] = 1;
            }
        }
    }

    int reached_count = 0;
    for (int i = 0; i < n_targets; ++i)
    {
        if (target_reached[i] == 1) ++reached_count;
    }
    std::cout << "Targets reached: " << reached_count << "\n";


    auto gpu_end = std::chrono::high_resolution_clock::now();
    auto gpu_duration = std::chrono::duration_cast<std::chrono::microseconds>(gpu_end - gpu_start);

    std::cout << gpu_hit_count << " visible voxels out of " << total_voxels << " are computed in " << (grid_duration.count() + gpu_duration.count()) << " microseconds." << std::endl;
}



__device__ bool isOccupied(const int* occupancy_grid, int x, int y, int z, int dim_x, int dim_y, int dim_z)
{
    if (x < 0 || y < 0 || z < 0 || x >= dim_x || y >= dim_y || z >= dim_z)
    {
        return false;
    }

    int idx = x + y * dim_x + z * dim_x * dim_y;
    return (occupancy_grid[idx] != 0);
}




__device__ bool castRayDDAGPU(
    const int* occupancy_grid,
    int dim_x, int dim_y, int dim_z,
    float resolution,
    double grid_origin_x, double grid_origin_y, double grid_origin_z,
    float ox, float oy, float oz,
    float dx, float dy, float dz,
    float max_range, 
    int target_x, int target_y, int target_z)
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
        if (isOccupied(occupancy_grid, x, y, z, dim_x, dim_y, dim_z))
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



__global__ void rayTraceKernel(const int* occupancy_grid, int dim_x, int dim_y, int dim_z,
    float resolution, double grid_origin_x, double grid_origin_y, double grid_origin_z,
    float* origins, int num_origins, float max_range, float* targets, int num_targets, int* hit_mask)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int total_rays = num_origins * num_targets;
    if (idx >= total_rays) return;

    int origin_idx = idx / num_targets;
    int target_idx = idx % num_targets;

    float ox = origins[3 * origin_idx + 0];
    float oy = origins[3 * origin_idx + 1];
    float oz = origins[3 * origin_idx + 2];


    float tx = targets[3 * target_idx + 0];
    float ty = targets[3 * target_idx + 1];
    float tz = targets[3 * target_idx + 2];


    float dx = tx - ox;
    float dy = ty - oy;
    float dz = tz - oz;


    int target_x = (tx - grid_origin_x) / resolution;
    int target_y = (ty - grid_origin_y) / resolution;
    int target_z = (tz - grid_origin_z) / resolution;


    bool hit = castRayDDAGPU(
        occupancy_grid, dim_x, dim_y, dim_z,
        resolution,
        grid_origin_x, grid_origin_y, grid_origin_z,
        ox, oy, oz,
        dx, dy, dz,
        max_range,
        target_x, target_y, target_z);

    hit_mask[idx] = hit ? 1 : 0;
}



