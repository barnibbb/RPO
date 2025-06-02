#include <iostream>
#include <fstream>
#include <chrono>
#include <filesystem>

#include "ros_visualizer.h"

int main(int argc, char** argv)
{
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

    std::vector<int> occupancy_grid = augmented_model->convertToOccupancyGrid();

    int counter = 0;

    for (size_t i = 0; i < occupancy_grid.size(); ++i)
    {
        if (occupancy_grid[i] == 1) ++counter;
    }

    std::cout << counter << "\n";



}