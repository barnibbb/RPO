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
        // -5.225, -2.775, 2407.87, -3.825, 1.875, 2534.17, 3.175, 3.125, 3431.68, 3.775, -2.875, 2842.64, -10.875, 0.325, 3183.65
        // 2.65984, 3.54321, 2902.96, 2.8404, -1.94794, 2791.76, -10.1326, 0.345447, 2735.61, -6.46628, -2.63822, 2721.91, -5.22498, 1.31136, 3247.77   // Current image
        // -9.73269, -0.00251749, 3618.44, 2.01022, 3.78146, 3250.13, -5.10794, 2.72515, 2340.88, 3.35711, -2.12161, 2809.45, -5.11453, -2.2825, 2381.1
        // 3.13083, 3.30326, 3226.44, 5.30059, -3.27486, 2477.48, -2.19757, -2.53533, 2855.67, -8.23053, 2.01384, 2849.75, -10.4613, -0.376203, 2990.66
        // -10.9077, 0.424158, 2959.3, 2.6367, 3.12577, 2833.42, -5.37416, 3.24226, 2824.04, -4.10749, -2.6971, 3098.68, 4.77561, -2.6971, 2684.56
        // -10.8934, 0.15817, 3160.64, -5.07649, -1.85679, 3299.51, 3.12536, 3.08161, 2504.1, 4.39372, -3.05956, 2476.1, -3.34565, 2.08924, 2959.64
        // -1.5018, 0.252721, 3763.31, 3.4017, 2.65261, 3031.35, 4.91407, -2.7206, 1856.54, -8.11815, -2.59778, 2307.32, -11.038, 3.25703, 3441.48
        -1.625, 0.475, 3763.31, 3.575, 2.825, 3031.35, 4.775, -2.775, 1856.54, -8.025, -2.625, 2307.32, -10.825, 3.175, 3441.48
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
