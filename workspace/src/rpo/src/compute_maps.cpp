#include <iostream>
#include <fstream>
#include <chrono>
#include <filesystem>

#include "gpu_ray_tracing.cuh"
#include "ros_visualizer.h"

int main(int argc, char** argv)
{
    auto start = std::chrono::high_resolution_clock::now();

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
    std::shared_ptr<rpo::ExtendedOcTree> extended_model = nullptr;

    std::ifstream file(parameters.paths.color_model);

    std::cout << parameters.paths.color_model << std::endl;
    std::cout << parameters.paths.extended_model << std::endl;

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

    file.open(parameters.paths.extended_model);

    if (file.is_open())
    {
        extended_model.reset(dynamic_cast<rpo::ExtendedOcTree*>(AbstractOcTree::read(file)));

        std::cout << "Extended octree num leaf nodes: " << extended_model->getNumLeafNodes() << std::endl;

        file.close();
    }        
    else
    {
        std::cerr << "Could not open extended octree file!" << std::endl;
        return -1;
    }


    assert(color_model->getNumLeafNodes() < 1'000);
    assert(autmented_model->getNumLeafNodes() < 1'000);





    // --- Visualizer initialization ---
    ros::init(argc, argv, "gpu_irradiance");

    rpo::ROSVisualizer visualizer(extended_model, color_model, parameters);



    // --- Preprocessing ---
    visualizer.cutUnderGround();
    visualizer.computeGroundZone();
    visualizer.computeGridElements();
    visualizer.computeRayTargets();
    visualizer.computeGeneralVisibility();



    // --- Generating occupancy grid ---
    float resolution = extended_model->getResolution();

    std::array<double, 6> boundaries = extended_model->getBoundaries();

    int dim_x = std::ceil((boundaries[1] - boundaries[0]) / resolution);
    int dim_y = std::ceil((boundaries[3] - boundaries[2]) / resolution);
    int dim_z = std::ceil((boundaries[5] - boundaries[4]) / resolution);

    std::vector<int> occupancy_grid(dim_x * dim_y * dim_z, 0);

    for (rpo::ExtendedOcTree::leaf_iterator it = extended_model->begin_leafs(), end = extended_model->end_leafs(); it != end; ++it)
    {
        if (extended_model->isNodeOccupied(*it))
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
        octomap::point3d pt = extended_model->keyToCoord(key, extended_model->getTreeDepth());

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
        octomap::point3d pt = extended_model->keyToCoord(grid_elements[i], extended_model->getTreeDepth());

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
    const float L = resolution;
    const float coefficient = parameters.lamp.power / (4 * M_PI * parameters.lamp.height);
    const float ground_level = visualizer.getGroundLevel();
    const int num_lamps = grid_elements.size();
    const int lamp_elements = ray_targets.size() - 2;
    const double base_coverage = (((parameters.lamp.height + parameters.lamp.offset) / parameters.lamp.height)) * parameters.preprocessing.safety_radius;



    // The irradiance maps are stored by lamp positions
    #pragma omp parallel for
    for (size_t i = 0; i < num_lamps; ++i)
    {
        rpo::ExposureMap irradiance_map;

        octomap::point3d target;

        octomap::KeySet visible_vox;

        target.x() = targets[3 * lamp_elements * i + 0];
        target.y() = targets[3 * lamp_elements * i + 1];

        for (size_t j = 0; j < num_origins; ++j)
        {
            float irradiance = 0;

            octomap::point3d origin(origins[3 * j + 0], origins[3 * j + 1], origins[3 * j + 2]);

            octomap::OcTreeKey key = extended_model->coordToKey(origin, extended_model->getTreeDepth());

            for (size_t k = 1; k < ray_targets.size() - 1; ++k)
            {
                target.z() = ray_targets[k];

                const int target_idx = lamp_elements * i + k - 1;

                // std::cout << j << " " << num_targets << " " << target_idx << " " << j * num_targets + target_idx << " " << (visibility_map[j * num_targets + target_idx] == 1) << "\n";

                if (visibility_map[j * num_targets + target_idx] == 1)
                {   
                    visible_vox.insert(key);

                    rpo::ExtendedOcTreeNode* node = extended_model->search(origin, extended_model->getTreeDepth());

                    const octomap::point3d normal = node->getNormal();

                    double integral = 0;

                    if (!std::isnan(normal.norm()))
                    {
                        const octomap::point3d difference = target - origin;

                        integral = visualizer.computeIrradianceIntegral(difference, normal, L);
                    }

                    if (node->getType() == 2)
                    {
                        const octomap::point3d base_point = point3d(target.x(), target.y(), ground_level);

                        if (std::abs(base_point.x() - origin.x()) < base_coverage && std::abs(base_point.y() - origin.y()) < base_coverage)
                        {
                            integral = 0;
                        }
                    }

                    irradiance += coefficient * integral;
                }
            }

            if (irradiance > 0)
            {
                irradiance_map[key] = irradiance;
            }
        }

        visualizer.saveBinaryMap(grid_elements[i], irradiance_map);

        // std::cout << grid_elements[i][0] << " " << grid_elements[i][1] << "\t" << visible_vox.size() << "\t" << irradiance_map.size() << "\n";
    }
    


    // Visible voxels 
    // for (int lamp_idx = 0; lamp_idx < grid_elements.size(); ++lamp_idx)
    // {
    //     int visible_voxels = 0;

    //     for (size_t i = 0; i < num_origins; ++i)
    //     {
    //         bool visible = false;
    //         for (size_t j = lamp_idx * lamp_elements; j < (lamp_idx + 1) * lamp_elements; ++j)
    //         {
    //             // std::cout << i << " " << num_targets << " " << j << " " << i * num_targets + j << " " << (visibility_map[i * num_targets + j] == 1) << "\n";

    //             if (visibility_map[i * num_targets + j] == 1)
    //             {
    //                 visible = true;
    //                 break;
    //             }
    //         }
    //         if (visible)
    //         {
    //             ++visible_voxels;

    //             octomap::point3d pt(
    //                 origins[3 * i + 0],
    //                 origins[3 * i + 1],
    //                 origins[3 * i + 2]
    //             );

    //             octomap::ColorOcTreeNode* node = color_model->search(pt, color_model->getTreeDepth());

    //             if (node != nullptr) node->setColor(255, 0, 0);
    //         } 
    //     }

    //     std::cout << "Number of visible voxels: " << grid_elements[lamp_idx][0] << " " << grid_elements[lamp_idx][1] << " " << visible_voxels << std::endl;
    // }


    // Visualization
    const std::string out_model = "/home/appuser/data/visible.ot";

    // visualizer.placeLamp(lamp_x, lamp_y);
    // color_model->write(out_model);

    // Runtime check:
    // 1  lamp: ~ 450 ms
    // 90 lamp: ~1300 ms

    auto stop = std::chrono::high_resolution_clock::now();
    auto dur = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    std::cout << "Total duration: " << dur.count() << std::endl;

    return 0;
}

