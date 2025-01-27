#include <memory>
#include <string>
#include <fstream>
#include <chrono>

#include <octomap/ColorOcTree.h>
#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

using namespace octomap;

using NodePtr = ColorOcTreeNode*;

/// Prototype for advanced ray tracing that returns break points
bool castRay(const std::shared_ptr<ColorOcTree>& octree, const point3d& origin, const point3d& direction, const point3d& target, point3d& end, 
    bool ignore_unknown, double max_range, int depth, double resolution, std::vector<OcTreeKey>& break_keys);


int main (int argc, char** argv)
{
    /// Read octomap models                                                                             
    std::string color_model_file = "/home/barni/rpo_ws/src/rpo/experiments/test/models/infirmary_color.ot";
    std::string floor_model_file = "/home/barni/rpo_ws/src/rpo/experiments/test/models/infirmary_floor.ot";
    
    std::shared_ptr<ColorOcTree> color_model = nullptr;
    std::shared_ptr<ColorOcTree> floor_model = nullptr;

    std::ifstream file(color_model_file);

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

    file.open(floor_model_file);

    if (file.is_open())
    {
        floor_model.reset(dynamic_cast<ColorOcTree*>(AbstractOcTree::read(file)));

        std::cout << "Floor octree num leaf nodes: " << floor_model->getNumLeafNodes() << std::endl;

        file.close();
    }
    else
    {
        std::cerr << "Could not open floor octree file!" << std::endl;
        return -1;
    }



    /// Define ray origin and target nodes
    int lamp_count = 4;

    double z_origin = 1.025;
    double z_target = 0.025;

    // points in free space
    // point3d origin_2d(3.025, 0.025, 0.0);  
    // point3d target_2d(-1.025, 0.025, 0.0);
    // point3d origin_3d(3.025, 0.025, z_origin);
    
    // points with obstacle
    point3d origin_2d(-4.275, -3.425, 0.0);
    point3d target_2d(-1.025, 1.225, 0.0);
    point3d origin_3d(-4.275, -3.425, z_origin);

    point3d direction_2d = target_2d - origin_2d;

    point3d end;

    /// Visualization
    for (ColorOcTree::leaf_iterator it = color_model->begin_leafs(), end = color_model->end_leafs(); it != end; ++it)
    {
        if (it.getCoordinate().x() > 3.95 && it.getCoordinate().y() > 3.45)
        {
            ColorOcTreeNode* node = color_model->search(it.getKey());

            node->setColor(255, 0, 0);
        }
    }
    
    
    /*
    NodePtr node = color_model->updateNode(origin_2d, true);
    node->setColor(255, 0, 0);
    node = color_model->updateNode(target_2d, true);
    node->setColor(255, 0, 0);
    
    std::vector<OcTreeKey> break_keys;
    castRay(color_model, origin_3d, direction_3d, target_3d, end, true, 100, 16, 0.05, break_keys);

    for (const auto& key : break_keys)
    {
        node = color_model->updateNode(key, true);
        node->setColor(0, 0, 255);
    }
    */



    /// Compute break points in 2D

    // Perform original ray tracing 
    std::cout << "Perform original ray tracing." << std::endl;

    auto start1 = std::chrono::high_resolution_clock::now();

    int rays_on_target = 0;

    for (int i = 0; i < lamp_count; ++i)
    {
        // auto start4 = std::chrono::high_resolution_clock::now();

        point3d target_3d = target_2d + point3d(0, 0, z_target + double(i) * 0.05);

        NodePtr t_node = color_model->updateNode(target_3d, true);

        point3d direction_3d = target_3d - origin_3d;

        color_model->castRay(origin_3d, direction_3d, end, true, -1.0);

        if ((end.x() == target_3d.x() && end.y() == target_3d.y() && end.z() == target_3d.z()))
        {
            ++rays_on_target;
        }

        color_model->deleteNode(target_3d, false);

        // auto stop4 = std::chrono::high_resolution_clock::now();

        // auto duration4 = std::chrono::duration_cast<std::chrono::nanoseconds>(stop4-start4);

        // std::cout << "4 duration: " << duration4.count() << std::endl;
    }

    auto stop1 = std::chrono::high_resolution_clock::now();

    auto duration1 = std::chrono::duration_cast<std::chrono::nanoseconds>(stop1-start1);

    std::cout << "Original duration: " << duration1.count() << std::endl;

    std::cout << "Rays on target: " << rays_on_target << std::endl;




    // Perform enhanced ray tracing 
    rays_on_target = 0;

    auto start2 = std::chrono::high_resolution_clock::now();

    std::vector<OcTreeKey> break_keys;

    castRay(floor_model, origin_2d, direction_2d, target_2d, end, true, 100, 16, 0.05, break_keys);

    double distance_2d = (target_2d - origin_2d).norm();
    double distance_z = z_target - z_origin;

    std::vector<point3d> break_points(break_keys.size());
    std::vector<double> break_distances(break_keys.size()); // divided by 2d distance

    for (int i = 0; i < break_points.size(); ++i)
    {
        break_points[i] = color_model->keyToCoord(break_keys[i]);
        break_distances[i] = (break_points[i] - origin_2d).norm() / distance_2d;
    }

    if (break_keys.size() == 0)
    {
        rays_on_target = lamp_count;
    }
    else
    {
        for (int i = 0; i < lamp_count; ++i)
        {
            // auto start3 = std::chrono::high_resolution_clock::now();

            point3d target_3d = target_2d + point3d(0, 0, z_target + double(i) * 0.05);

            NodePtr t_node = color_model->updateNode(target_3d, true);

            for (int j = 0; j < break_points.size(); j += 2)
            {
                point3d p1 = break_points[j];
                point3d p2 = break_points[j + 1];

                double z1 = z_origin + break_distances[j]     * (distance_z + double(i) * 0.05);
                double z2 = z_origin + break_distances[j + 1] * (distance_z + double(i) * 0.05);

                // Instead of key and coordinate conversions
                p1.z() = (std::floor(std::abs(z1) / 0.05) * 0.05 + 0.025) * z1 / std::abs(z1);
                p2.z() = (std::floor(std::abs(z2) / 0.05) * 0.05 + 0.025) * z2 / std::abs(z2);

                // p1.z() = z1;
                // p2.z() = z2;

                // OcTreeKey p1k, p2k;
                // color_model->coordToKeyChecked(p1, 16, p1k);
                // color_model->coordToKeyChecked(p2, 16, p2k);

                // p1 = color_model->keyToCoord(p1k, 16);
                // p2 = color_model->keyToCoord(p2k, 16);

                NodePtr t_node = color_model->updateNode(p2, true);

                if (color_model->castRay(p1, p2 - p1, end, true, -1.0))
                {
                    if (end.x() != p2.x() || end.y() != p2.y() || end.z() != p2.z())
                    {
                        break;
                    }
                    else if (j == break_points.size() - 2)
                    {
                        ++rays_on_target;
                    }
                }

                color_model->deleteNode(target_3d, false);
            }

            // auto stop3 = std::chrono::high_resolution_clock::now();

            // auto duration3 = std::chrono::duration_cast<std::chrono::nanoseconds>(stop3-start3);

            // std::cout << "3 duration: " << duration3.count() << std::endl;
        }
    }

    auto stop2 = std::chrono::high_resolution_clock::now();

    auto duration2 = std::chrono::duration_cast<std::chrono::nanoseconds>(stop2-start2);

    std::cout << "Enhanced duration: " << duration2.count() << std::endl;

    std::cout << "Rays on target: " << rays_on_target << std::endl;


    // ROS Visualization
    ros::init(argc, argv, "rtt");

    ros::NodeHandle node_handle;
    ros::Publisher model_publisher = node_handle.advertise<octomap_msgs::Octomap>("/ray_tracing_test", 10);

    octomap_msgs::Octomap message;

    std::stringstream data_stream;
    color_model->writeData(data_stream);
    std::string datastring = data_stream.str();

    message.data = std::vector<int8_t>(datastring.begin(), datastring.end());
    message.resolution = color_model->getResolution();
    message.id = color_model->getTreeType();
    message.binary = false;
    message.header.frame_id = "map";
    message.header.stamp = ros::Time();

    while(ros::ok())
    {
        model_publisher.publish(message);
    }
}


bool castRay(const std::shared_ptr<ColorOcTree>& octree, const point3d& origin, const point3d& direction, const point3d& target, point3d& end, 
    bool ignore_unknown, double max_range, int depth, double resolution, std::vector<OcTreeKey>& break_keys)
{
    // Initialization phase -------------------------------------------------------

    OcTreeKey target_key;
    
    octree->coordToKeyChecked(target, depth, target_key);

    OcTreeKey current_key;

    if (!octree->coordToKeyChecked(origin, depth, current_key))
    {
        std::cout << "Coordinates out of bounds during ray casting" << std::endl;
        return false;
    }

    NodePtr starting_node = octree->search(current_key, depth);

    if (starting_node)
    {
        if (octree->isNodeOccupied(starting_node))
        {
            end = octree->keyToCoord(current_key, depth);
            // return true;
        }
    } 
    else if (!ignore_unknown)
    {
        end = octree->keyToCoord(current_key, depth);
        return false;
    }

    point3d normalized_direction = direction.normalized();

    bool max_range_set = (max_range > 0.0);

    int step[3];
    double t_max[3];
    double t_delta[3];

    for (unsigned int i = 0; i < 3; ++i)
    {
        if (normalized_direction(i) > 0.0)
        {
            step[i] = 1;
        }
        else if (normalized_direction(i) < 0.0)
        {
            step[i] = -1;
        }
        else
        {
            step[i] = 0;
        }

        if (step[i] != 0)
        {
            double voxel_border = octree->keyToCoord(current_key[i], depth);
            voxel_border += static_cast<double>(step[i] * resolution * 0.5);

            t_max[i] = (voxel_border - origin(i)) / normalized_direction(i);
            t_delta[i] = resolution / std::fabs(normalized_direction(i));
        }
        else
        {
            t_max[i] = std::numeric_limits<double>::max();
            t_delta[i] = std::numeric_limits<double>::max();
        }
    }

    if (step[0] == 0 && step[1] == 0 && step[2] == 0)
    {
        std::cout << "Raycasting in direction (0,0,0) is not possible!" << std::endl;
        return false;
    }

    double max_range_sq = max_range * max_range;

    // Incremental phase  ---------------------------------------------------------

    bool done = false;

    bool free = true;

    OcTreeKey previous_key;

    while (!done)
    {
        unsigned int dim;

        if (t_max[0] < t_max[1])
        {
            dim = (t_max[0] < t_max[2]) ? 0 : 2;
        }
        else
        {
            dim = (t_max[1] < t_max[2]) ? 1 : 2;
        }

        if ((step[dim] < 0 && current_key[dim] == 0) || (step[dim] > 0 && current_key[dim] == 2 * 32768 - 1))    
        {
            std::cout << "Coordinate hit bounds in dim" << dim << ", aborting raycast" << std::endl;
            end = octree->keyToCoord(current_key, depth);
            return false; 
        }

        current_key[dim] += step[dim];
        t_max[dim] += t_delta[dim];

        end = octree->keyToCoord(current_key, depth);

        if (max_range_set)
        {
            double distance_from_origin_sq = 0.0;

            for (unsigned int j = 0; j < 3; ++j)
            { 
                distance_from_origin_sq += ((end(j) - origin(j)) * (end(j) - origin(j)));
            }

            if (distance_from_origin_sq > max_range_sq)
            {
                return false;
            }
        }

        NodePtr current_node = octree->search(current_key, depth);

        if (target_key == current_key)
        {
            done = true;
            break;
        }


        if (free && current_node && octree->isNodeOccupied(current_node))
        {
            break_keys.push_back(previous_key);
            free = false;
        }
        else if (!free && (!current_node || (current_node && !octree->isNodeOccupied(current_node))))
        {
            break_keys.push_back(current_key);
            free = true;
        }
        else
        {
            // break_keys.push_back(current_key);
        }

        previous_key = current_key;
    }

    return true;
}

