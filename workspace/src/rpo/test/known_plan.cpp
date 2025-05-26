#include <unistd.h>
#include <chrono>

#include "ros_visualizer.h"


int main (int argc, char** argv)
{
    // Read input parameters -------------------------------------------------------
    if (argc != 2)
    {
        std::cerr << "Usage: rosrun rpo RPO <parameter_file>" << std::endl;
        return -1;
    }

    const std::string parameters_file = argv[1];

    rpo::Parameters parameters = rpo::Parameters::loadParameters(parameters_file);



    // Read 3D models --------------------------------------------------------------                                                                                  
    std::shared_ptr<ColorOcTree> color_model = nullptr;
    std::shared_ptr<rpo::AugmentedOcTree> augmented_model = nullptr;

    std::cout << parameters.paths.color_model << "\n";

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

    // Visualizer initialization ---------------------------------------------------
    ros::init(argc, argv, "rpo");

    rpo::ROSVisualizer visualizer(augmented_model, color_model, parameters);

    // Preprocessing ---------------------------------------------------------------
    visualizer.cutUnderGround();
    visualizer.computeGroundZone();
    visualizer.computeGridElements();
    visualizer.computeRayTargets();


    // visualizer.computeIrradianceMaps();
    visualizer.loadIrradianceMaps();

    visualizer.showElementTypes();


    
    double disinfection_time = 5 * 3600;

    auto grid_elements = visualizer.getGridElements();

    // Lambda expression:
    std::sort(grid_elements.begin(), grid_elements.end(), [](const auto& a, const auto& b) {
        return (a.k[0] < b.k[0]) || (a.k[0] == b.k[0] && a.k[1] < b.k[1]);
    });


    std::vector<double> plan(3 * grid_elements.size());

    std::vector<double> times = {
        43.655057, 21.340844, 0.0, 0.0, 0.0, 0.0, 0.0, 26.7287, 97.677524, 0.0, 4.9411318, 0.0, 18.915776, 0.0, 5.8534553, 0.0, 43.504421, 18.215653, 19.640749, 0.0, 0.0, 20.64376, 100.80917, 25.442151, 0.0, 0.0, 9.0709, 12.611764, 0.0, 2.5461836, 2.2377387, 0.89573164, 10.569505, 0.0, 69.132612, 41.549812, 0.0, 0.0, 0.0, 91.754872, 85.180008, 31.036201, 32.178217, 64.467996, 0.0, 5.0033195, 24.664336, 14.71822, 12.213427, 7.5729645, 0.0, 0.0, 0.0, 0.0, 22.077479, 24.078456, 67.650413, 0.0, 48.014903, 22.166759, 16.790682, 0.0, 0.0, 45.098909, 0.0, 8.1636581, 33.164402, 80.146633, 30.446752, 17.804926, 0.0, 9.4642427, 4.2362377, 20.951803, 10.909266, 8.6991444, 0.0, 0.47240209, 1.8636407, 15.215603, 54.18653, 38.327524, 42.488028, 50.568613, 36.290032, 38.228098, 0.0, 12.661753, 0.0, 75.060911
    };


    for (int i = 0; i < plan.size(); i += 3)
    {
        point3d p = color_model->keyToCoord(grid_elements[i / 3], 16);

        std::cout << grid_elements[i / 3][0] << " " << grid_elements[i / 3][1] << "\n";

        plan[i    ] = p.x();
        plan[i + 1] = p.y();
        plan[i + 2] = times[i / 3];
        // plan[i + 2] = disinfection_time / double(grid_elements.size());
    }
    

    rpo::RadiationPlan best_plan;

    best_plan.first = plan;
    best_plan.second = 0;

    auto grid_plan = visualizer.getGrid(best_plan);
 

    rpo::Score score = visualizer.showResult(grid_plan, true);

    std::cout << std::endl << "General coverage: " << score.general_coverage << std::endl;


    return 0;
}
