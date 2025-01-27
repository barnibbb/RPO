#include "ros_visualizer.h"


int main (int argc, char** argv)
{
    ros::init(argc, argv, "floor_plan_generator");

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



    // Read 3D model --------------------------------------------------------------- 
    std::shared_ptr<rpo::AugmentedOcTree> augmented_model = nullptr;
    std::shared_ptr<octomap::ColorOcTree> color_model = std::make_shared<octomap::ColorOcTree>(0.05);

    std::ifstream file(parameters.augmented_model_file);

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



    KeySet obstacle_2d, free_2d;

    // Step 1: Map obstacles to the x-y plane
    for (rpo::AugmentedOcTree::leaf_iterator it = augmented_model->begin_leafs(), end = augmented_model->end_leafs(); it != end; ++it)
    {
        point3d point_3d = it.getCoordinate();
    
        point3d point_2d(point_3d.x(), point_3d.y(), 0);

        if (OcTreeKey key; augmented_model->coordToKeyChecked(point_2d, 16, key))
        {
            if (point_3d.z() > 0.08)
            {
                obstacle_2d.insert(key);
            }
            else
            {
                free_2d.insert(key);
            }
        }
    }

    // Step 2: Insert 2D obstacles to the augmented 3D model
    for (const auto& key : obstacle_2d)
    {
        octomap::ColorOcTreeNode* node = color_model->search(key, 16);

        if (node == nullptr)
        {
            node = color_model->updateNode(key, true);
            node->setColor(rpo::BLACK);
        }
    }

    for (const auto& key : free_2d)
    {
        octomap::ColorOcTreeNode* node = color_model->search(key, 16);

        if (node == nullptr)
        {
            // node = color_model->updateNode(key, true);
            // node->setColor(rpo::ORANGE);
        }
    }



    std::vector<double> plan { 
    -5.36675,	-2.91054,	2407.87,	-3.95822,	1.8304,	2534.17,	3.18136,	2.89501,	3431.68,	3.5893,	-2.63424,	2842.64,	-10.8811,	0.149716,	3183.65 };


    for (int i = 0; i < plan.size(); i += 3)
    {
        point3d center(plan[i], plan[i + 1], 0);

        for (int x = -2; x < 3; ++x)
        {
            for (int y = -2; y < 3; ++y)
            {   
                point3d point = center + point3d(x * color_model->getResolution(), y * color_model->getResolution(), 0);

                ColorOcTreeNode* color_node = color_model->search(point);

                // Grid points - purple
                //color_node->setColor(rpo::PURPLE);
            }
        }  
    }

    std::string outfile = "/home/barni/rpo_ws/src/rpo/experiments/test/models/infirmary_floor.ot";

    color_model->write(outfile);

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