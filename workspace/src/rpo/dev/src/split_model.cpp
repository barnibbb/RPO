#include <iostream>
#include <memory>
#include <string>

#include "augmented_octree.h"
#include "parameters.h"

int main(int argc, char** argv)
{
    // --- Read input parameters ---
    if (argc < 2)
    {
        std::cerr << "Usage: rosrun rpo SplitModel <parameter_file>" << std::endl;
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


    const point3d vertical(0, 0, 1);

    for (rpo::AugmentedOcTree::leaf_iterator it = augmented_model->begin_leafs(), end = augmented_model->end_leafs(); it != end; ++it)
    {
        octomap::ColorOcTreeNode* c_node = color_model->search(it.getKey(), color_model->getTreeDepth());

        rpo::AugmentedOcTreeNode* a_node = augmented_model->search(it.getKey(), augmented_model->getTreeDepth());

        if (c_node != nullptr && a_node != nullptr)
        {
            const point3d normal = a_node->getNormal();

            const double angle = std::abs(180.0 * normal.angleTo(vertical) / M_PI);

            if (angle <= 20.0 || angle >= 160.0)
            {
                c_node->setColor(255, 0, 0);
            }
            else if (angle >= 70.0 && angle <= 110.0)
            {
                c_node->setColor(0, 0, 255);
            }
        }
    }


    const std::string out_model = "/home/appuser/data/split.ot";
    color_model->write(out_model);


    return 0;
}


