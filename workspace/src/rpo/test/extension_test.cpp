#include "extended_octree.h"

#include <filesystem>
#include <regex>

#include <ros/ros.h>

int main (int argc, char** argv)
{
    if (argc != 4)
    {
        std::cerr << "Usage: rosrun rpo  Extension <color_model> <surface> <visualize>" << std::endl;
        return -1;
    }

    // Processing input parameters
    const std::string color_model_file = argv[1];

    std::filesystem::path color_path(color_model_file);
    std::filesystem::path dir = color_path.parent_path();
    std::string stem = color_path.stem().string();

    std::string new_stem = std::regex_replace(stem, std::regex("_color$"), "_extended");
    std::string new_filename = new_stem + color_path.extension().string();
    std::filesystem::path extended_model_path = dir / new_filename;

    const std::string extended_model_file = extended_model_path.string();

    bool surface = std::string(argv[2]) == "1" ? true : false;
    bool visualize = std::string(argv[3]) == "1" ? true : false;

    std::ifstream file_1(color_model_file);

    if (file_1.is_open())
    {   
        std::unique_ptr<ColorOcTree> color_octree = nullptr;

        color_octree.reset(dynamic_cast<ColorOcTree*>(AbstractOcTree::read(file_1)));

        color_octree->expand();

        std::cout << "Color octree number of leaf nodes: " << color_octree->getNumLeafNodes() << std::endl;

        std::shared_ptr<rpo::ExtendedOcTree> extended_octree = rpo::ExtendedOcTree::convertToExtendedOcTree(*color_octree);

        std::cout << "Converted extended octree number of leaf nodes: " << extended_octree->getNumLeafNodes() << std::endl;

        extended_octree->compute3DNormalVectors();

        extended_octree->computeGroundLevel();

        extended_octree->findObjects(surface);

        if (visualize)
        {
            ros::init(argc, argv, "extension");

            extended_octree->visualize();
        }

        std::fstream file_2(extended_model_file, std::ios::out);

        if (file_2.is_open())
        {
            extended_octree->write(file_2);

            file_2.close();

            std::shared_ptr<rpo::ExtendedOcTree> read_extended_octree = nullptr;

            file_2.open(extended_model_file, std::ios::in);

            if (file_2.is_open())
            {
                read_extended_octree.reset(dynamic_cast<rpo::ExtendedOcTree*>(AbstractOcTree::read(file_2)));

                std::cout << "Loaded extended octree number of leaf nodes: " << read_extended_octree->getNumLeafNodes() << std::endl;

                file_2.close();
            }
        } 

        file_1.close();
    }
    else
    {
        std::cerr << "Could not open color octree file" << std::endl;
    }


    return 0;
}
