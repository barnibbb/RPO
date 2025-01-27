#include <unistd.h>
#include <chrono>

#include "ros_visualizer.h"


// PROBLEMS:
// Points at the side of walls on ground level are counted as visible
// Consider single obstacle nodes --> odd vector size should count 2x

// IMPORTANTE:
// Check 3D and simplified ray tracing for certain cases (differences)



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

    // visualizer.filter();

    
    // double x1, y1, z1, x2, y2, z2;

    // augmented_model->getMetricMin(x1, y1, z1);
    // augmented_model->getMetricMax(x2, y2, z2);

    // std::cout << x1 << " " << y1 << " " << x2 << " " << y2 << " " << z1 << " " << z2 << "\n";

    // std::cin.get();
    
    // visualizer.create2DModel();

    // visualizer.computeBreakPoints();

    // visualizer.cutUnderGround2();    

    // std::cout << "Break points computed!" << std::endl; // std::cin.get();
    

    // visualizer.computeIrradianceMaps();

    // std::cout << "Maps computed" << std::endl;

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
    

    // K408
    // std::vector<double> plan = { 6.375, 0.525, 703.24, 5.275, -0.825, 678.454, 1.075, 0.775, 632.07, 3.675, 2.425, 735.713, 0.625, 3.075, 850.523 };


    // std::vector<double> plan = { -1.375,1.825,189.972,1.225,1.975,188.422,0.925,-1.975,193.637,1.125,-0.525,237.942,-3.225,1.775,235.416,3.025,1.875,183.784,3.225,-1.875,176.221,-3.225,-1.925,201.738,-1.075,-1.175,192.867};


    // std::vector<double> plan = { 0.122631, -0.0223292, 1800 };

    // std::vector<double> plan = { 1.26871,1.83461,233.685,3.10692,0.599587,207.599,-1.27179,0.599587,306.861,1.47638,-2.21884,201.696,3.39681,-2.21884,199.462,-3.60694,1.61535,228.748,-0.871637,-0.883297,244.433,-3.32817,-2.60911,177.516 };

    /*
    std::vector<double> plan = { 
        // -5.36675, -2.91054, 2407.87, -3.95822, 1.8304, 2534.17, 3.18136, 2.89501, 3431.68, 3.5893, -2.63424, 2842.64, -10.8811, 0.149716, 3183.65
        // -10.1828, 1.35932, 1613.15, -3.1206, -2.29872, 2143.18, 4.89274, -1.3541, 1567.41, 4.89274, -3.38399, 1377.38, 2.60088, 2.10381, 1132.33, -11.8498, -1.63424, 1328.49, -5.76353, 2.291, 1638.06 }; 
        // -9.73269, -0.00251749, 3618.44, 2.01022, 3.78146, 3250.13, -5.10794, 2.72515, 2340.88, 3.35711, -2.12161, 2809.45, -5.11453, -2.2825, 2381.1
        // 3.13083, 3.30326, 3226.44, 5.30059, -3.27486, 2477.48, -2.19757, -2.53533, 2855.67, -8.23053, 2.01384, 2849.75, -10.4613, -0.376203, 2990.66
        // -10.9077, 0.424158, 2959.3, 2.6367, 3.12577, 2833.42, -5.37416, 3.24226, 2824.04, -4.10749, -2.6971, 3098.68, 4.77561, -2.6971, 2684.56
        // -10.8934, 0.15817, 3160.64, -5.07649, -1.85679, 3299.51, 3.12536, 3.08161, 2504.1, 4.39372, -3.05956, 2476.1, -3.34565, 2.08924, 2959.64
        // -1.5018, 0.252721, 3763.31, 3.4017, 2.65261, 3031.35, 4.91407, -2.7206, 1856.54, -8.11815, -2.59778, 2307.32, -11.038, 3.25703, 3441.48
        -1.625, 0.475, 3763.31, 3.575, 2.825, 3031.35, 4.775, -2.775, 1856.54, -8.025, -2.625, 2307.32, -10.825, 3.175, 3441.48
    };
    */

    // std::vector<double> plan = { 3.39559, 2.62554, 954.541, 8.3596, 5.86627, 949.058, 8.10075, 3.37607, 878.882, 6.11857, 4.60131, 885.376, 0.612783, 5.61203, 951.29, 3.9469, 7.146, 903.321, 5.65865, 0.850044, 978.756, 0.894982, 1.60567, 698.775 };

    // std::vector<double> plan = { 0, 0, 1000 };

    rpo::RadiationPlan best_plan;

    best_plan.first = plan;
    best_plan.second = 0;

    auto grid_plan = visualizer.getGrid(best_plan);
 

    rpo::Score score = visualizer.showResult(grid_plan, true);

    // for (const auto& g : grid_plan.first) std::cout << g << ", ";


    std::cout << std::endl << "General coverage: " << score.general_coverage << std::endl;

    auto stop = std::chrono::high_resolution_clock::now();

    auto dur = std::chrono::duration_cast<std::chrono::nanoseconds>(stop - start);

    std::cout << "Duration: " << dur.count() << std::endl;


    return 0;
}
