#include <fstream>
#include <memory>
#include <string>

#include "ros_visualizer.h"

int main (int argc, char** argv)
{
    if (argc < 4) return -1;

    const std::string scene = argv[1];
    const std::string type = argv[2];
    const std::string closed = argv[3];

    bool cls = closed == "closed" ? true : false;

    const std::string work_folder = "/home/appuser/data/" + scene + "/" + scene + "_" + type;
    const std::string sol_file    = work_folder + "_indices.txt";
    const std::string tsp_file    = work_folder + ".tsp";
    const std::string order_file  = work_folder + ".order";
    const std::string path_file   = work_folder + ".path";
    const std::string pos_file    = work_folder + ".pos";

    const std::string parameters_file = "/home/appuser/data/" + scene + "/params_" + scene + "_xy.yaml";

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
    if (type == "active") visualizer.loadActiveIndices(sol_file);
    else visualizer.loadActiveIndices2();

    visualizer.computeGraph();

    visualizer.exportGraph(tsp_file);

    std::string command = "python3 /home/appuser/workspace/src/rpo/scripts/run_concorde.py ";
    command += tsp_file;
    command += " ";
    command += order_file;

    system(command.c_str());

    visualizer.readOrder(order_file, cls);

    visualizer.buildPath(path_file, pos_file);

}
