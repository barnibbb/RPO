#include <string>
#include <memory>
#include <unordered_set>

#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <boost/algorithm/string.hpp>

#include "augmented_octree.h"

int main (int argc, char** argv)
{
    std::string model_1 = "/home/barni/rpo_ws/src/rpo/experiments2/1/infirmary/test/model_opt_7.ot";
    std::string model_2 = "/home/barni/rpo_ws/src/rpo/experiments2/1/infirmary/test/model_opt_8.ot";

    // Read 3D models --------------------------------------------------------------                                                                                  
    std::shared_ptr<octomap::ColorOcTree> color_model_1 = nullptr;
    std::shared_ptr<octomap::ColorOcTree> color_model_2 = nullptr;

    octomap::ColorOcTree diff_model(0.05);

    rpo::AugmentedOcTree diff_augmented(0.05);

    std::ifstream file(model_1);

    if (file.is_open())
    {
        color_model_1.reset(dynamic_cast<octomap::ColorOcTree*>(octomap::ColorOcTree::read(file)));

        color_model_1->expand();

        std::cout << "Color octree num leaf nodes: " << color_model_1->getNumLeafNodes() << std::endl;

        file.close();
    }
    else
    {
        std::cerr << "Could not open color octree file!" << std::endl;
        return -1;
    }

    file.open(model_2);

    if (file.is_open())
    {
        color_model_2.reset(dynamic_cast<octomap::ColorOcTree*>(octomap::ColorOcTree::read(file)));

        color_model_2->expand();

        std::cout << "Color octree num leaf nodes: " << color_model_2->getNumLeafNodes() << std::endl;

        file.close();
    }        
    else
    {
        std::cerr << "Could not open augmented octree file!" << std::endl;
        return -1;
    }

    octomap::ColorOcTree ctree2(0.05);

    int differences_1 = 0;
    int differences_2 = 0;

    std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash> diff_missing;
    std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash> diff_extra;

    for (octomap::ColorOcTree::leaf_iterator it = color_model_1->begin_leafs(), end = color_model_1->end_leafs(); it != end; ++it)
    {
        octomap::ColorOcTreeNode* node_4 = ctree2.updateNode(it.getKey(), true);

        node_4->setColor(255, 255, 255);
    }

    ctree2.expand();

    for (octomap::ColorOcTree::leaf_iterator it = color_model_1->begin_leafs(), end = color_model_1->end_leafs(); it != end; ++it)
    {
        octomap::ColorOcTreeNode* node_1 = color_model_1->search(it.getKey(), 16);

        octomap::ColorOcTreeNode* node_2 = color_model_2->search(it.getKey(), 16);

        octomap::ColorOcTreeNode* node_4 = ctree2.search(it.getKey(), 16);

        int r1 = node_1->getColor().r;
        int b1 = node_1->getColor().b;
        int g1 = node_1->getColor().g;

        int r2 = node_2->getColor().r;
        int b2 = node_2->getColor().b;
        int g2 = node_2->getColor().g;

        // if (node_1->getColor().r == 0 && node_2->getColor().r != 0)
        if ((r1 != r2) || (b1 != b2) || (g1 != g2))
        {
            octomap::ColorOcTreeNode* n = diff_model.updateNode(it.getKey(), true);

            n->setColor(255, 0, 0);

            rpo::NodePtr na = diff_augmented.updateNode(it.getKey(), true);

            ++differences_1;

            std::cout << it.getCoordinate() << std::endl;

            diff_extra.insert(it.getKey());

            node_4->setColor(255, 0, 0);
        }
        /*else if (node_1->getColor().r != 0 && node_2->getColor().r == 0)
        {
            octomap::ColorOcTreeNode* n = diff_model.updateNode(it.getKey(), true);

            n->setColor(0, 0, 255);

            ++differences_2;

            std::cout << it.getCoordinate() << std::endl;

            diff_missing.insert(it.getKey());

            node_4->setColor(255, 0, 0);
        }*/

        if (it.getCoordinate().z() < -0.5)
        {
            octomap::ColorOcTreeNode* node_3 = diff_model.updateNode(it.getKey(), true);

            node_3->setColor(0, 0, 0);
        }
    }

    std::cout << "Missing: " << diff_missing.size() << std::endl;
    std::cout << "Extra: " << diff_extra.size() << std::endl;

    std::string model_3 = "/home/barni/rpo_ws/src/rpo/experiments/test/extra.ot";
    std::string model_4 = "/home/barni/rpo_ws/src/rpo/experiments/test/extra_augmented.ot";

    diff_model.write(model_3);
    diff_augmented.write(model_4);

    ctree2.write("/home/barni/rpo_ws/src/rpo/experiments/test/c2.ot");

    std::fstream f("/home/barni/rpo_ws/src/rpo/experiments/test/extra_keys.txt", std::ios::out);

    for (const auto& key : diff_extra)
    {
        f << key[0] << " " << key[1] << " " << key[2] << "\n";
    }

    f.close();

    f.open("/home/barni/rpo_ws/src/rpo/experiments/test/missing_keys.txt", std::ios::out);

    for (const auto& key : diff_missing)
    {
        f << key[0] << " " << key[1] << " " << key[2] << "\n";
    }

    f.close();


    return 0;
}
