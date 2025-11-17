#include <fstream>
#include <memory>
#include <string>

#include "ros_visualizer.h"

int main (int argc, char** argv)
{
    const std::string parameters_file = "/home/appuser/data/params.yaml";

    rpo::Parameters parameters = rpo::Parameters::loadParameters(parameters_file);

    // Read 3D models --------------------------------------------------------------                                                                                  
    std::shared_ptr<ColorOcTree> color_model = nullptr;
    std::shared_ptr<rpo::ExtendedOcTree> extended_model = nullptr;

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



    // Visualizer initialization ---------------------------------------------------
    ros::init(argc, argv, "rpo");
    rpo::ROSVisualizer visualizer(extended_model, color_model, parameters);


    // Preprocessing ---------------------------------------------------------------
    visualizer.cutUnderGround();
    visualizer.computeGroundZone();
    visualizer.computeGridElements();
    

    // Load active indices
    const std::string sol_file = "/home/appuser/data/infirmary2.sol";
    const std::string tsp_file = "/home/appuser/data/infirmary.tsp";
    const std::string order_file = "/home/appuser/data/infirmary.order";
    const std::string path_file = "/home/appuser/data/path.txt";
    const std::string pos_file = "/home/appuser/data/pos.txt";

    visualizer.loadActiveIndices(sol_file);

    visualizer.computeGraph();

    visualizer.exportGraph(tsp_file);

    visualizer.readOrder(order_file);

    visualizer.buildPath(path_file, pos_file);

}
