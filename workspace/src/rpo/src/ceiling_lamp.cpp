#include <iostream>
#include <fstream>
#include <chrono>
#include <filesystem>

#include "gpu_ray_tracing.cuh"
#include "ros_visualizer.h"

int main(int argc, char** argv)
{
    // --- Read input parameters ---
    if (argc < 2)
    {
        std::cerr << "Usage: rosrun rpo GPU_Irradiance <parameter_file>" << std::endl;
        return -1;
    }

    const std::string parameters_file = argv[1];

    rpo::Parameters parameters = rpo::Parameters::loadParameters(parameters_file);


    // --- Read 3D models ---                                                                                  
    std::shared_ptr<ColorOcTree> color_model = nullptr;
    std::shared_ptr<rpo::AugmentedOcTree> augmented_model = nullptr;

    std::ifstream file(parameters.paths.color_model);

    std::cout << parameters.paths.color_model << std::endl;
    std::cout << parameters.paths.augmented_model << std::endl;

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
    ros::init(argc, argv, "ceiling_lamp");

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
    std::vector<float> targets(3);

    targets[0] = 0.025;
    targets[1] = 0.025;
    targets[2] = boundaries[5] - 0.075;    

    std::cout << "Target elements: " << targets.size() << "\n";


    // --- Visibility check ---
    int num_origins = origins.size() / 3;
    int num_targets = targets.size() / 3;
    float max_range = 50.0;

    std::vector<int> visibility_map(num_origins * num_targets);

    auto start_rt = std::chrono::high_resolution_clock::now();

    runKernel(occupancy_grid, origins, targets, visibility_map, dim_x, dim_y, dim_z,
        boundaries[0], boundaries[2], boundaries[4], resolution, max_range, num_origins, num_targets);

    auto stop_rt = std::chrono::high_resolution_clock::now();
    auto dur_rt = std::chrono::duration_cast<std::chrono::milliseconds>(stop_rt - start_rt);
    std::cout << "Ray tracing time: " << dur_rt.count() << std::endl;



    // Irradiance computation
    const float ground_level = visualizer.getGroundLevel();
    const int num_lamps = num_targets;
    const double base_coverage = (((parameters.lamp.height + parameters.lamp.offset) / parameters.lamp.height)) * parameters.preprocessing.safety_radius;


    // The irradiance maps are stored by lamp positions
    #pragma omp parallel for
    for (size_t i = 0; i < num_lamps; ++i)
    {
        rpo::ExposureMap irradiance_map;

        octomap::point3d target(targets[3 * i + 0], targets[3 * i + 1], targets[3 * i + 2]);

        octomap::OcTreeKey t_key = augmented_model->coordToKey(target, augmented_model->getTreeDepth());

        octomap::KeySet visible_vox;

        for (size_t j = 0; j < num_origins; ++j)
        {
            float irradiance = 0;

            octomap::point3d origin(origins[3 * j + 0], origins[3 * j + 1], origins[3 * j + 2]);

            octomap::OcTreeKey key = augmented_model->coordToKey(origin, augmented_model->getTreeDepth());

            const int target_idx = i;

            if (visibility_map[j * num_targets + target_idx] == 1)
            {   
                visible_vox.insert(key);

                rpo::AugmentedOcTreeNode* node = augmented_model->search(origin, augmented_model->getTreeDepth());

                const octomap::point3d normal = node->getNormal();

                if (!std::isnan(normal.norm()))
                {
                    const octomap::point3d difference = target - origin;

                    irradiance = parameters.lamp.power * std::abs(std::cos(normal.angleTo(difference))) / (4 * M_PI * std::pow(difference.norm(), 2));     
                }
            }

            if (irradiance > 0)
            {
                irradiance_map[key] = irradiance;
            }
        }

        visualizer.saveBinaryMap2(t_key, irradiance_map);
    }





    // Visible voxels 
    int visible_voxels = 0;

    for (int lamp_idx = 0; lamp_idx < num_lamps; ++lamp_idx)
    {
        for (size_t i = 0; i < num_origins; ++i)
        {
            if (visibility_map[i * num_targets + lamp_idx] == 1)
            {
                ++visible_voxels;

                octomap::point3d pt(origins[3 * i + 0], origins[3 * i + 1], origins[3 * i + 2]);

                octomap::ColorOcTreeNode* node = color_model->search(pt, color_model->getTreeDepth());

                if (node != nullptr) node->setColor(255, 0, 0);
            } 
        }
    }

    std::cout << visible_voxels << std::endl;

    octomap::ColorOcTreeNode* l_node = color_model->updateNode(octomap::point3d(0.025, 0.025, boundaries[5] - 0.075), true);
    

    
    if (l_node != nullptr) l_node->setColor(0, 0, 255);


    const std::string out_model = "/home/appuser/data/visible.ot";
    color_model->write(out_model);


    return 0;
}
