#include "augmented_octree.h"

using namespace octomap;

namespace rpo
{
    std::istream& AugmentedOcTreeNode::readData(std::istream &s)
    {
        s.read((char*) &value, sizeof(value));
        s.read((char*) &m_normal, sizeof(point3d));
        s.read((char*) &m_type, sizeof(int));

        return s;
    }


    std::ostream& AugmentedOcTreeNode::writeData(std::ostream &s) const
    {
        s.write((const char*) &value, sizeof(value));
        s.write((char*) &m_normal, sizeof(point3d));
        s.write((char*) &m_type, sizeof(int));

        return s;
    }


    // Octomap related methods --------------------------------------------------------------

    AugmentedOcTree::AugmentedOcTree(double resolution) : OccupancyOcTreeBase<AugmentedOcTreeNode>(resolution)
    {
        m_augmented_octree_member_init.ensureLinking();
    }


    bool AugmentedOcTree::pruneNode(AugmentedOcTreeNode* node)
    {
        if (!isNodeCollapsible(node))
        {
            return false;
        }

        node->copyData(*(getNodeChild(node, 0)));

        for (unsigned int i = 0; i < 8; ++i)
        {
            deleteNodeChild(node, i);
        }

        delete[] node->children;
        node->children = NULL;

        return true;
    }


    // Note: expand node does not set values properly
    void AugmentedOcTree::expandNode(AugmentedOcTreeNode* node)
    {
        for (unsigned int i = 0; i < 8; ++i)
        {
            AugmentedOcTreeNode* child = createNodeChild(node, i);
            child->copyData(*node);
        }
    }


    bool AugmentedOcTree::isNodeCollapsible(const AugmentedOcTreeNode* node) const
    {
        if (!nodeChildExists(node, 0))
        {
            return false;
        }

        const AugmentedOcTreeNode* first_child = getNodeChild(node, 0);

        if (nodeHasChildren(first_child))
        {
            return false;
        }

        for (unsigned int i = 1; i < 8; ++i)
        {
            if (!nodeChildExists(node, i) || nodeHasChildren(getNodeChild(node, i)) || 
                !(getNodeChild(node, i)->getValue() == first_child->getValue()) || !node->hasSameAugmentations(*getNodeChild(node, i)))
            {
                return false;
            }
        }

        return true;
    }


    void AugmentedOcTree::updateInnerOccupancy()
    {
        this->updateInnerOccupancyRecurs(this->root, 0);
    }


    void AugmentedOcTree::updateInnerOccupancyRecurs(AugmentedOcTreeNode* node, unsigned int depth)
    {
        if (nodeHasChildren(node))
        {
            if (depth < this->tree_depth)
            {
                for (unsigned int i = 0; i < 8; ++i)
                {
                    if (nodeChildExists(node, i))
                    {
                        updateInnerOccupancyRecurs(getNodeChild(node, i), depth + 1);
                    }
                }
            }
            node->updateOccupancyChildren();
        }
    }


    AugmentedOcTree::StaticMemberInitializer AugmentedOcTree::m_augmented_octree_member_init;


    // RPO related methods ------------------------------------------------------------------

    // While creating the augmented octree, nodes are automatically pruned
    std::shared_ptr<AugmentedOcTree> AugmentedOcTree::convertToAugmentedOcTree(const ColorOcTree& color_octree)
    {
        std::shared_ptr<AugmentedOcTree> augmented_octree = std::make_shared<AugmentedOcTree>(color_octree.getResolution());

        for (ColorOcTree::leaf_iterator it = color_octree.begin_leafs(), end = color_octree.end_leafs(); it != end; ++it)
        {
            rpo::AugmentedOcTreeNode* node = augmented_octree->search(it.getKey());

            if (node == nullptr)
            {
                node = augmented_octree->updateNode(it.getKey(), true);
                node->setNormal(point3d(0, 0, 0));
                node->setType(0);
            }
        }

        augmented_octree->expand();

        return augmented_octree;
    }


    void AugmentedOcTree::pruneTree()
    {
        for (tree_iterator it = this->begin_tree(), end = this->end_tree(); it != end; ++it)
        {
            if (it.getDepth() == this->tree_depth - 1 && it->hasChildren())
            {
                NodePtr node = this->search(it.getKey(), this->tree_depth - 1);

                for (unsigned int i = 0; i < 8; ++i)
                {
                    if (it->childExists(i))
                    {
                        this->deleteNodeChild(node, i);
                    }
                }
            }
        }
    }


    // Ray cast related methods -------------------------------------------------------------

    bool AugmentedOcTree::castRay(const point3d& origin, const point3d& direction, const point3d& target, point3d& end, 
        bool ignore_unknown, double max_range, int depth, double resolution) const
    {
        // Initialization phase -------------------------------------------------------

        OcTreeKey current_key;

        if (!this->coordToKeyChecked(origin, depth, current_key))
        {
            std::cout << "Coordinates out of bounds during ray casting" << std::endl;
            return false;
        }

        NodePtr starting_node = this->search(current_key, depth);

        if (starting_node)
        {
            if (this->isNodeOccupied(starting_node))
            {
                end = this->keyToCoord(current_key, depth);
                return true;
            }
        } 
        else if (!ignore_unknown)
        {
            end = this->keyToCoord(current_key, depth);
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
                double voxel_border = this->keyToCoord(current_key[i], depth);
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

            if ((step[dim] < 0 && current_key[dim] == 0) || (step[dim] > 0 && current_key[dim] == 2 * this->tree_max_val - 1))    
            {
                std::cout << "Coordinate hit bounds in dim" << dim << ", aborting raycast" << std::endl;
                end = this->keyToCoord(current_key, depth);
                return false; 
            }

            current_key[dim] += step[dim];
            t_max[dim] += t_delta[dim];

            end = this->keyToCoord(current_key, depth);

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

            NodePtr current_node = this->search(current_key, depth);

            if (current_node)
            {
                if (this->isNodeOccupied(current_node))
                {
                    done = true;
                    break;
                }
                else if (!ignore_unknown)
                {
                    return false;
                }
            }
            else
            {
                if (evaluateRayCast(target, end, depth))
                {
                    done = true;
                    break;
                }
            }
        }

        return true;
    }


    bool AugmentedOcTree::evaluateRayCast(const point3d& target_point, const point3d& end_point, int depth) const
    {
        OcTreeKey target_key, end_key;

        if (this->coordToKeyChecked(target_point, depth, target_key) && this->coordToKeyChecked(end_point, depth, end_key))
        {
            return (target_key == end_key);
        }

        return false;
    }


    bool AugmentedOcTree::checkRayCast(bool good, const point3d& target_point, const point3d& origin, const point3d& direction, 
        point3d& end_point, double max_range, int depth, double resolution, bool ignore_unknown) const
    {
        if (castRay(origin, direction, target_point, end_point, ignore_unknown, 
            max_range, depth, resolution))
        {
            if (!good && !evaluateRayCast(target_point, end_point, depth) ||
                 good &&  evaluateRayCast(target_point, end_point, depth))
            {
                return true;
            }
        }

        return false;
    }


    void AugmentedOcTree::compute3DNormalVectors()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr octree_points (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr octree_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>());

        for (leaf_iterator it = this->begin_leafs(), end = this->end_leafs(); it != end; ++it)
        {
            const point3d point = it.getCoordinate();

            octree_points->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
        }

        kdtree->setInputCloud(octree_points);

        normal_estimation.setInputCloud(octree_points);
        normal_estimation.setSearchMethod(kdtree);
        normal_estimation.setRadiusSearch(5 * this->resolution);
        normal_estimation.compute(*octree_normals);

        for (size_t i = 0; i < octree_points->size(); ++i)
        {
            const pcl::PointXYZ pcl_point = octree_points->points[i];

            const point3d point(pcl_point.data[0], pcl_point.data[1], pcl_point.data[2]);

            if (NodePtr node = this->search(point, this->tree_depth); node != nullptr)
            {
                const pcl::Normal normal = octree_normals->at(i);

                node->setNormal(point3d(normal.normal_x, normal.normal_y, normal.normal_z));
            }
        }
    }


    void AugmentedOcTree::computeGroundLevel()
    {
        std::map<double, int> height_map;

        // First it is counted how many voxels belong to each height.
        for (leaf_iterator it = this->begin_leafs(), end = this->end_leafs(); it != end; ++it)
        {
            const point3d point = it.getCoordinate();

            if (height_map.find(point.z()) == height_map.end())
            {
                height_map[point.z()] = 1;
            }
            else
            {
                height_map[point.z()] += 1;
            }
        }

        double ground_level;

        bool ground_found = false;

        // The ground level will be the second layer which contains at least 5% of all model elements.
        for (const auto& height : height_map)
        {
            if (height.second > 0.05 * this->getNumLeafNodes())
            {
                ground_found = true;

                ground_level = height.first;
            }
            else
            {
                if (ground_found == true)
                {
                    break;
                }
            }
        }

        for (leaf_iterator it = this->begin_leafs(), end = this->end_leafs(); it != end; ++it)
        {
            if (std::abs(it.getCoordinate().z() - ground_level) < 0.001)
            {
                NodePtr node = this->search(it.getKey(), this->tree_depth);

                if (node != nullptr)
                {
                    node->setType(2);
                }
            }

            if (it.getCoordinate().z() < ground_level)
            {
                NodePtr node = this->search(it.getKey(), this->tree_depth);

                if (node != nullptr)
                {
                    node->setType(3);
                }
            }
        }

        std::cout << "Ground level: " << ground_level << std::endl;
    }


    void AugmentedOcTree::findObjects(bool surface)
    {
        this->calcMinMax();

        const float resolution = this->resolution;

        const int depth = this->tree_depth;

        static const point3d vertical(0, 0, 1);

        static const double quarter = M_PI / 2.0;

        // Based on the angle to the vertical direction, segment vertical surfaces
        for (leaf_iterator it = this->begin_leafs(), end = this->end_leafs(); it != end; ++it)
        {
            NodePtr node = this->search(it.getKey(), depth);

            if (node->getType() != 2 && node->getType() != 3 && !std::isnan(node->getNormal().norm()))
            {
                point3d top_point = it.getCoordinate();

                bool top = false;

                for (int z = 0; z < 3; ++z)
                {
                    top_point.z() = this->max_value[2] - z * resolution - 0.5 * resolution;

                    NodePtr top_node = this->search(top_point, depth);

                    if (top_node != nullptr)
                    {
                        top = true;
                        break;
                    }
                }

                const double angle_to_vertical = std::abs(node->getNormal().angleTo(vertical));

                if (top)
                {
                    node->setType(4);
                }
                else
                {
                    node->setType(5);
                }
            }
        }

        if (surface)
        {
            for (leaf_iterator it = this->begin_leafs(), end = this->end_leafs(); it != end; ++it)
            {
                NodePtr node = this->search(it.getKey(), depth);

                point3d upper_point = it.getCoordinate() + point3d(0, 0, resolution);

                NodePtr upper_node = this->search(upper_point, depth);

                if (upper_node != nullptr && upper_node->getType() == 5 && node->getType() == 5)
                {
                    node->setType(1);
                }
            }
        }
    }


    void AugmentedOcTree::visualize()
    {
        ColorOcTree color_model(this->resolution);

        int t1 = 0, t2 = 0, t3 = 0, t4 = 0, t5 = 0;

        for (AugmentedOcTree::leaf_iterator it = this->begin_leafs(), end = this->end_leafs(); it != end; ++it)
        {
            ColorOcTreeNode* color_node = color_model.updateNode(it.getKey(), true);
        }

        color_model.expand();

        for (AugmentedOcTree::leaf_iterator it = this->begin_leafs(), end = this->end_leafs(); it != end; ++it)
        {
            NodePtr node = this->search(it.getKey(), this->tree_depth);

            ColorOcTreeNode* color_node = color_model.search(it.getKey(), this->tree_depth);

            switch (node->getType())
            {
            case 1: // General
                color_node->setColor(255, 0, 0); ++t1;
                break;
            case 2: // Ground
                color_node->setColor(0, 255, 0); ++t2;
                break;
            case 3: // Underground
                color_node->setColor(0, 0, 255); ++t3;
                break;
            case 4: // Wall
                color_node->setColor(255, 255, 0); ++t4;
                break;
            case 5: // Object
                color_node->setColor(0, 255, 255); ++t5;
                break;
            default:
                break;
            }
        }

        std::cout << t1 << " " << t2 << " " << t3 << " " << t4 << " " << t5 << std::endl;

        ros::NodeHandle node_handle;
        ros::Publisher publisher = node_handle.advertise<octomap_msgs::Octomap>("/segmented_map", 10);

        octomap_msgs::Octomap message;

        std::stringstream data_stream;
        color_model.writeData(data_stream);
        std::string datastring = data_stream.str();

        message.data = std::vector<int8_t>(datastring.begin(), datastring.end());
        message.resolution = color_model.getResolution();
        message.id = color_model.getTreeType();
        message.binary = false;
        message.header.frame_id = "map";
        message.header.stamp = ros::Time();

        while (ros::ok())
        {
            publisher.publish(message);

            ros::spinOnce();
        }
        
    }


    std::array<double, 6> AugmentedOcTree::getBoundaries() const
    {
        std::array<double, 6> boundaries;

        this->getMetricMin(boundaries[0], boundaries[2], boundaries[4]);
        this->getMetricMax(boundaries[1], boundaries[3], boundaries[5]);

        return boundaries;
    }
}
