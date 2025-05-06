#include "ros_visualizer.h"

int main (int argc, char** argv)
{
    ros::init(argc, argv, "floor_plan_generator");

    const std::string colored_octree = "/home/barni/rpo_ws/src/rpo/experiments/figures/color_model.ot";

    std::shared_ptr<octomap::ColorOcTree> color_model = nullptr;

    color_model.reset(dynamic_cast<ColorOcTree*>(AbstractOcTree::read(colored_octree)));

    color_model->expand();

    const double ground_level = 0.08;

    std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash> keys;

    for (octomap::ColorOcTree::leaf_iterator it = color_model->begin_leafs(), end = color_model->end_leafs(); it != end; ++it)
    {
        if (it.getCoordinate().z() > ground_level)
        {
            keys.insert(it.getKey());
        }
    }

    for (const auto& key : keys)
    {
        color_model->deleteNode(key, 16);
    }


    std::vector<double> plan { 
         -1.625,  0.475, 3763.31, 
          3.575,  2.825, 3031.35, 
          4.775, -2.775, 1856.54, 
         -8.025, -2.625, 2307.32,
        -10.825,  3.175, 3441.48
    };





    const int rad = 4;


    for (int i = 0; i < plan.size(); i += 3)
    {
        point3d center(plan[i], plan[i + 1], 0.075);

        for (int x = -rad; x <= rad; ++x)
        {
            for (int y = -rad; y <= rad; ++y)
            {   
                point3d point = center + point3d(x * color_model->getResolution(), y * color_model->getResolution(), 0);

                ColorOcTreeNode* color_node = color_model->search(point);

                // Grid points - purple
                // color_node->setColor(int(255 * rpo::R[255]), int(255 * rpo::G[255]), int(255 * rpo::B[255]));
            }
        }  
    }


    ros::NodeHandle node_handle;
    ros::Publisher model_publisher = node_handle.advertise<octomap_msgs::Octomap>("/floor_plan", 10);;

    octomap_msgs::Octomap message;

    if (color_model != nullptr)
    {
        std::stringstream data_stream;
        color_model->writeData(data_stream);
        std::string datastring = data_stream.str();

        message.data = std::vector<int8_t>(datastring.begin(), datastring.end());
        message.resolution = color_model->getResolution();
        message.id = color_model->getTreeType();
        message.binary = false;
        message.header.frame_id = "map";
        message.header.stamp = ros::Time();
    }

    while (ros::ok())
    {
        model_publisher.publish(message);
    }

    return 0;
}
