#include <unistd.h>
#include <chrono>

#include "ros_visualizer.h"


int main (int argc, char** argv)
{
    auto start = std::chrono::high_resolution_clock::now();

    // Read input parameters -------------------------------------------------------
    rpo::Parameters parameters;
    
    if (argc > 1)
    {
        const std::string parameters_file = argv[1];

        parameters.setValues(parameters_file);
    }
    else
    {
        std::cerr << "Parameter file must set!" << std::endl;

        return -1;
    }



    // Read 3D models --------------------------------------------------------------                                                                                  
    std::shared_ptr<ColorOcTree> color_model = nullptr;
    std::shared_ptr<rpo::AugmentedOcTree> augmented_model = nullptr;

    std::ifstream file(parameters.color_model_file);

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

    file.open(parameters.augmented_model_file);

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

    // Visualizer initialization ---------------------------------------------------
    ros::init(argc, argv, "rpo");

    rpo::ROSVisualizer visualizer(augmented_model, color_model, parameters);

    // Preprocessing ---------------------------------------------------------------
    visualizer.cutUnderGround();
    visualizer.computeGroundZone();
    visualizer.computeGridElements();
    visualizer.computeRayTargets();

    visualizer.loadIrradianceMaps();

    visualizer.showElementTypes();


    
    double disinfection_time = 5 * 3600;

    auto grid_elements = visualizer.getGridElements();

    std::vector<double> plan(3 * grid_elements.size());

    for (int i = 0; i < plan.size(); i += 3)
    {
        point3d p = color_model->keyToCoord(grid_elements[i / 3], 16);

        plan[i] = p.x();
        plan[i + 1] = p.y();
        plan[i + 2] = disinfection_time / double(grid_elements.size());
    }
    

    rpo::RadiationPlan best_plan;

    best_plan.first = plan;
    best_plan.second = 0;

    auto grid_plan = visualizer.getGrid(best_plan);
 

    rpo::Score score = visualizer.showResult(grid_plan, true);

    std::cout << std::endl << "General coverage: " << score.general_coverage << std::endl;

    auto stop = std::chrono::high_resolution_clock::now();

    auto dur = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);

    std::cout << "Duration: " << dur.count() << std::endl;


    return 0;
}
