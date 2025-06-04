#include <iostream>
#include <fstream>
#include <chrono>
#include <filesystem>

#include <cuda_runtime.h>

#include "ros_visualizer.h"


const double PRECISION = 1e-4;

__device__ bool isOccupied(const int* occupancy_grid, int x, int y, int z, int dim_x, int dim_y, int dim_z);

__device__ bool castRayDDAGPU(const int* occupancy_grid, int dim_x, int dim_y, int dim_z, float resolution,
    double grid_origin_x, double grid_origin_y, double grid_origin_z, float ox, float oy, float oz,
    float dx, float dy, float dz, float max_range, int target_x, int target_y, int target_z);

__global__ void visibilityKernel(
    const float* origins,
    const float* targets,
    int* visibility_map,
    const int* occupancy_grid,
    int dim_x, int dim_y, int dim_z,
    float grid_x, float grid_y, float grid_z,
    float resolution, float max_range,
    int num_origins, int num_targets
);




int main(int argc, char** argv)
{
    auto start = std::chrono::high_resolution_clock::now();

    // --- Read input parameters ---
    if (argc != 2)
    {
        std::cerr << "Usage: rosrun rpo RPO <parameter_file>" << std::endl;
        return -1;
    }

    const std::string parameters_file = argv[1];

    rpo::Parameters parameters = rpo::Parameters::loadParameters(parameters_file);



    // --- Read 3D models ---                                                                                  
    std::shared_ptr<ColorOcTree> color_model = nullptr;
    std::shared_ptr<rpo::AugmentedOcTree> augmented_model = nullptr;

    std::ifstream file(parameters.paths.color_model);

    if (file.is_open())
    {
        color_model.reset(dynamic_cast<ColorOcTree*>(AbstractOcTree::read(file)));

        std::cout << "Color octree num leaf nodes: " << color_model->getNumLeafNodes() << std::endl;

        file.close();
    }
    else
    {
        std::cerr << "Could not open color octree file!" << std::endl;
        return -1;
    }

    file.open(parameters.paths.augmented_model);

    if (file.is_open())
    {
        augmented_model.reset(dynamic_cast<rpo::AugmentedOcTree*>(AbstractOcTree::read(file)));

        std::cout << "Augmented octree num leaf nodes: " << augmented_model->getNumLeafNodes() << std::endl;

        file.close();
    }        
    else
    {
        std::cerr << "Could not open augmented octree file!" << std::endl;
        return -1;
    }


    assert(color_model->getNumLeafNodes() < 1'000);
    assert(autmented_model->getNumLeafNodes() < 1'000);



    // --- Visualizer initialization ---
    ros::init(argc, argv, "rpo");

    rpo::ROSVisualizer visualizer(augmented_model, color_model, parameters);



    // --- Preprocessing ---
    visualizer.cutUnderGround();
    visualizer.computeGroundZone();
    visualizer.computeGridElements();
    visualizer.computeRayTargets();
    visualizer.computeGeneralVisibility();



    // --- Generating occupancy grid ---
    float resolution = augmented_model->getResolution();

    std::array<double, 6> boundaries = augmented_model->getBoundaries();

    int dim_x = std::ceil((boundaries[1] - boundaries[0]) / resolution);
    int dim_y = std::ceil((boundaries[3] - boundaries[2]) / resolution);
    int dim_z = std::ceil((boundaries[5] - boundaries[4]) / resolution);

    std::vector<int> occupancy_grid(dim_x * dim_y * dim_z, 0);

    for (rpo::AugmentedOcTree::leaf_iterator it = augmented_model->begin_leafs(), end = augmented_model->end_leafs(); it != end; ++it)
    {
        if (augmented_model->isNodeOccupied(*it))
        {
            octomap::point3d coord = it.getCoordinate();
            int x = static_cast<int>((coord.x() - boundaries[0]) / resolution);
            int y = static_cast<int>((coord.y() - boundaries[2]) / resolution);
            int z = static_cast<int>((coord.z() - boundaries[4]) / resolution);
            occupancy_grid[x + y * dim_x + z * dim_x * dim_y] = 1;
        }
    }

    int counter = 0;

    for (size_t i = 0; i < occupancy_grid.size(); ++i)
    {
        if (occupancy_grid[i] == 1) ++counter;
    }

    std::cout << "Occupancy grid elements: " << counter << "\n";



    // --- Setup origins ---
    KeySet origin_keys = visualizer.getBaseReachableElements();
    std::vector<float> origins;

    for (const auto key : origin_keys)
    {
        octomap::point3d pt = augmented_model->keyToCoord(key, augmented_model->getTreeDepth());

        origins.push_back(pt.x());
        origins.push_back(pt.y());
        origins.push_back(pt.z());
    }

    std::cout << "Origins elements: " << origins.size() << "\n";


    // --- Setup targets ---
    std::vector<octomap::OcTreeKey> grid_elements = visualizer.getGridElements();
    std::vector<double> ray_targets = visualizer.getRayTargets();
    std::vector<float> targets;

    float lamp_x, lamp_y;


    for (size_t i = 0; i < grid_elements.size(); ++i)
    {
        octomap::point3d pt = augmented_model->keyToCoord(grid_elements[i], augmented_model->getTreeDepth());

        lamp_x = pt.x();
        lamp_y = pt.y();

        for (size_t j = 1; j < ray_targets.size() - 1; ++j)
        {
            targets.push_back(pt.x());
            targets.push_back(pt.y());
            targets.push_back(ray_targets[j]);
        }
    }

    std::cout << "Target elements: " << targets.size() << "\n";


    // --- Memory allocation ---
    int* d_grid;
    cudaMalloc(&d_grid, occupancy_grid.size() * sizeof(int));
    cudaMemcpy(d_grid, occupancy_grid.data(), occupancy_grid.size() * sizeof(int), cudaMemcpyHostToDevice);

    float* d_origins;
    cudaMalloc(&d_origins, origins.size() * sizeof(float));
    cudaMemcpy(d_origins, origins.data(), origins.size() * sizeof(float), cudaMemcpyHostToDevice);

    float* d_targets;
    cudaMalloc(&d_targets, targets.size() * sizeof(float));
    cudaMemcpy(d_targets, targets.data(), targets.size() * sizeof(float), cudaMemcpyHostToDevice);

    int num_origins = origins.size() / 3;
    int num_targets = targets.size() / 3;
    std::vector<int> visibility_map(num_origins * num_targets);
    int* d_visibility_map;
    cudaMalloc(&d_visibility_map, num_origins * num_targets * sizeof(int));


    // --- Ray tracing kernel
    float max_range = 50.0;
    int threadsPerBlock = 256;
    int blocks = (num_origins * num_targets + threadsPerBlock - 1) / threadsPerBlock;

    visibilityKernel<<<blocks, threadsPerBlock>>>(
        d_origins, d_targets, d_visibility_map, d_grid,
        dim_x, dim_y, dim_z, boundaries[0], boundaries[2], boundaries[4],
        resolution, max_range, num_origins, num_targets   
    );

    cudaDeviceSynchronize();

    cudaMemcpy(visibility_map.data(), d_visibility_map, num_origins * num_targets * sizeof(int), cudaMemcpyDeviceToHost);

    int visible_voxels = 0;

    for (size_t i = 0; i < num_origins; ++i)
    {
        bool visible = false;
        for (size_t j = 0; j < num_targets; ++j)
        {
            if (visibility_map[i * num_targets + j] == 1)
            {
                visible = true;
                break;
            }
        }
        if (visible)
        {
            ++visible_voxels;

            octomap::point3d pt(
                origins[3 * i + 0],
                origins[3 * i + 1],
                origins[3 * i + 2]
            );

            octomap::ColorOcTreeNode* node = color_model->search(pt, color_model->getTreeDepth());

            if (node != nullptr) node->setColor(255, 0, 0);
        } 

        
    }

    std::cout << "Number of visible voxels: " << visible_voxels << std::endl;



    // --- Free memory ---
    cudaFree(d_grid);
    cudaFree(d_origins);
    cudaFree(d_targets);
    cudaFree(d_visibility_map);



    // Visualization
    const std::string out_model = "/home/appuser/data/visible.ot";

    // visualizer.placeLamp(lamp_x, lamp_y);

    color_model->write(out_model);

    auto stop = std::chrono::high_resolution_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::seconds>(stop - start);

    std::cout << "Total duration: " << dur.count() << std::endl;
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


__device__ bool castRayDDAGPU(const int* occupancy_grid, int dim_x, int dim_y, int dim_z, float resolution,
    double grid_origin_x, double grid_origin_y, double grid_origin_z, float ox, float oy, float oz,
    float dx, float dy, float dz, float max_range, int target_x, int target_y, int target_z)
{
    int x = static_cast<int>((ox - grid_origin_x) / resolution);
    int y = static_cast<int>((oy - grid_origin_y) / resolution);
    int z = static_cast<int>((oz - grid_origin_z) / resolution);


    float length = std::sqrt(dx * dx + dy * dy + dz * dz);
    dx /= length; dy /= length; dz /= length;

    float epsilon = 1e-6f;

    if (std::abs(dx) < epsilon) dx = (dx >= 0.0f ? epsilon : -epsilon);
    if (std::abs(dy) < epsilon) dy = (dy >= 0.0f ? epsilon : -epsilon);
    if (std::abs(dz) < epsilon) dz = (dz >= 0.0f ? epsilon : -epsilon);


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
        // Lamps are just virtually placed thus any collision means unreachability
        if (x == target_x && y == target_y && z == target_z)
        {
            return true;
        }
        else if (isOccupied(occupancy_grid, x, y, z, dim_x, dim_y, dim_z))
        {
            return false;
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


__global__ void visibilityKernel(
    const float* origins,
    const float* targets,
    int* visibility_map,
    const int* occupancy_grid,
    int dim_x, int dim_y, int dim_z,
    float grid_x, float grid_y, float grid_z,
    float resolution, float max_range,
    int num_origins, int num_targets
)
{
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    int total_rays = num_origins * num_targets;
    if (idx >= total_rays) return;

    int origin_idx = idx / num_targets;
    int target_idx = idx % num_targets;

    // Origins
    float ox = origins[3 * origin_idx + 0];
    float oy = origins[3 * origin_idx + 1];
    float oz = origins[3 * origin_idx + 2];

    // Targets
    float tx = targets[3 * target_idx + 0];
    float ty = targets[3 * target_idx + 1];
    float tz = targets[3 * target_idx + 2];

    // Directions
    float dx = tx - ox;
    float dy = ty - oy;
    float dz = tz - oz;

    // Target int
    int target_x = (tx - grid_x) / resolution;
    int target_y = (ty - grid_y) / resolution;
    int target_z = (tz - grid_z) / resolution;

    bool hit = false;

    for (int d = 0; d < 3; ++d)
    {
        float sx = ox + (d == 0 ? (dx > 0 ? resolution : -resolution) : 0);
        float sy = oy + (d == 1 ? (dy > 0 ? resolution : -resolution) : 0);
        float sz = oz + (d == 2 ? (dz > 0 ? resolution : -resolution) : 0);

        float dir_x = tx - sx;
        float dir_y = ty - sy;
        float dir_z = tz - sz;

        if (castRayDDAGPU(occupancy_grid, dim_x, dim_y, dim_z, resolution, grid_x, grid_y, grid_z,
            sx, sy, sz, dir_x, dir_y, dir_z, max_range, target_x, target_y, target_z))
        {
            hit = true;
            break;
        } 
    } 

    visibility_map[origin_idx * num_targets + target_idx] = hit ? 1 : 0;
}




