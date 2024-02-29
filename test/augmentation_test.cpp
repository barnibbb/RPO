#include "augmented_octree.h"

int main (int argc, char** argv)
{
    const std::string color_model_file = "/home/barni/rpo_ws/src/rpo/models/base/office_color.ot";
    const std::string augmented_model_file = "/home/barni/rpo_ws/src/rpo/models/augmented/office_augmented_full.ot";

    bool surface = false;
    bool visualize = false;

    std::ifstream file_1(color_model_file);

    if (file_1.is_open())
    {   
        std::unique_ptr<ColorOcTree> color_octree = nullptr;

        color_octree.reset(dynamic_cast<ColorOcTree*>(AbstractOcTree::read(file_1)));

        color_octree->expand();

        std::cout << "Color octree number of leaf nodes: " << color_octree->getNumLeafNodes() << std::endl;

        std::shared_ptr<rpo::AugmentedOcTree> augmented_octree = rpo::AugmentedOcTree::convertToAugmentedOcTree(*color_octree);

        std::cout << "Converted augmented octree number of leaf nodes: " << augmented_octree->getNumLeafNodes() << std::endl;

        augmented_octree->compute3DNormalVectors();

        augmented_octree->computeGroundLevel();

        augmented_octree->findObjects(surface);

        if (visualize)
        {
            ros::init(argc, argv, "augmentation");

            augmented_octree->visualize();
        }

        std::fstream file_2(augmented_model_file, std::ios::out);

        if (file_2.is_open())
        {
            augmented_octree->write(file_2);

            file_2.close();

            std::shared_ptr<rpo::AugmentedOcTree> read_augmented_octree = nullptr;

            file_2.open(augmented_model_file, std::ios::in);

            if (file_2.is_open())
            {
                read_augmented_octree.reset(dynamic_cast<rpo::AugmentedOcTree*>(AbstractOcTree::read(file_2)));

                std::cout << "Loaded augmented octree number of leaf nodes: " << read_augmented_octree->getNumLeafNodes() << std::endl;

                file_2.close();
            }
        } 

        file_1.close();

    }

    return 0;
}
