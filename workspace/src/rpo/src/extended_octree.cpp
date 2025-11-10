#include "extended_octree.h"

#include <iostream>
#include <fstream>

#include <octomap_msgs/Octomap.h>
#include <pcl/features/normal_3d_omp.h>
#include <ros/ros.h>

using namespace octomap;

namespace rpo
{
    std::istream& ExtendedOcTreeNode::readData(std::istream &s)
    {
        s.read((char*) &value, sizeof(value));
        s.read((char*) &m_normal, sizeof(point3d));
        s.read((char*) &m_dose, sizeof(double));
        s.read((char*) &m_type, sizeof(int));

        return s;
    }


    std::ostream& ExtendedOcTreeNode::writeData(std::ostream &s) const
    {
        s.write((const char*) &value, sizeof(value));
        s.write((char*) &m_normal, sizeof(point3d));
        s.write((char*) &m_dose, sizeof(double));
        s.write((char*) &m_type, sizeof(int));

        return s;
    }


    // Octomap related methods --------------------------------------------------------------

    ExtendedOcTree::ExtendedOcTree(double resolution) : OccupancyOcTreeBase<ExtendedOcTreeNode>(resolution)
    {
        m_extended_octree_member_init.ensureLinking();
    }


    bool ExtendedOcTree::pruneNode(ExtendedOcTreeNode* node)
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
    void ExtendedOcTree::expandNode(ExtendedOcTreeNode* node)
    {
        for (unsigned int i = 0; i < 8; ++i)
        {
            ExtendedOcTreeNode* child = createNodeChild(node, i);
            child->copyData(*node);
        }
    }


    bool ExtendedOcTree::isNodeCollapsible(const ExtendedOcTreeNode* node) const
    {
        if (!nodeChildExists(node, 0))
        {
            return false;
        }

        const ExtendedOcTreeNode* first_child = getNodeChild(node, 0);

        if (nodeHasChildren(first_child))
        {
            return false;
        }

        for (unsigned int i = 1; i < 8; ++i)
        {
            if (!nodeChildExists(node, i) || nodeHasChildren(getNodeChild(node, i)) || 
                !(getNodeChild(node, i)->getValue() == first_child->getValue()) || !node->hasSameExtensions(*getNodeChild(node, i)))
            {
                return false;
            }
        }

        return true;
    }


    void ExtendedOcTree::updateInnerOccupancy()
    {
        this->updateInnerOccupancyRecurs(this->root, 0);
    }


    void ExtendedOcTree::updateInnerOccupancyRecurs(ExtendedOcTreeNode* node, unsigned int depth)
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


    ExtendedOcTree::StaticMemberInitializer ExtendedOcTree::m_extended_octree_member_init;


    // RPO related methods ------------------------------------------------------------------

    // While creating the extended octree, nodes are automatically pruned
    std::shared_ptr<ExtendedOcTree> ExtendedOcTree::convertToExtendedOcTree(const ColorOcTree& color_octree)
    {
        std::shared_ptr<ExtendedOcTree> extended_octree = std::make_shared<ExtendedOcTree>(color_octree.getResolution());

        for (ColorOcTree::leaf_iterator it = color_octree.begin_leafs(), end = color_octree.end_leafs(); it != end; ++it)
        {
            rpo::ExtendedOcTreeNode* node = extended_octree->search(it.getKey());

            octomap::ColorOcTreeNode::Color color = it->getColor();

            if (node == nullptr)
            {
                node = extended_octree->updateNode(it.getKey(), true);
                node->setColor(point3d(color.r, color.g, color.b));
                node->setNormal(point3d(0, 0, 0));
                node->setDose(0);
                node->setType(0);
            }
        }

        extended_octree->expand();

        return extended_octree;
    }


    void ExtendedOcTree::pruneTree()
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

    bool ExtendedOcTree::castRay(const point3d& origin, const point3d& direction, const point3d& target, point3d& end, 
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
            std::cout << origin << " " << target << "\n"; std::cin.get();

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
                // std::cout << "Coordinate hit bounds in dim" << dim << ", aborting raycast" << std::endl;
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


    // Computes ray and collects the keys of break points
    bool ExtendedOcTree::castRay2(const point3d& origin, const point3d& direction, const point3d& target, point3d& end, 
        bool ignore_unknown, double max_range, int depth, double resolution, std::vector<OcTreeKey>& break_keys1, std::vector<OcTreeKey>& break_keys2, bool skip) const
    {
        // Initialization phase -------------------------------------------------------

        OcTreeKey target_key = this->coordToKey(target, depth);

        OcTreeKey current_key;

        if (!this->coordToKeyChecked(origin, depth, current_key))
        {
            std::cout << "Coordinates out of bounds during ray casting" << std::endl;
            return false;
        }

        NodePtr starting_node = this->search(current_key, depth);

        bool free = true;

        if (starting_node)
        {
            if (this->isNodeOccupied(starting_node))
            {
                end = this->keyToCoord(current_key, depth);

                break_keys1.push_back(current_key);
                break_keys2.push_back(current_key);
                free = false;
        }
        } 
        else if (!ignore_unknown)
        {
            end = this->keyToCoord(current_key, depth);
            return false;
        }

        if (break_keys1.size() == 0 && skip) break_keys1.push_back(current_key);

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
            // std::cout << "Raycasting in direction (0,0,0) is not possible!" << std::endl;
            return false;
        }

        double max_range_sq = max_range * max_range;

        // Incremental phase  ---------------------------------------------------------

        bool done = false;

        OcTreeKey previous_key;

        while (!done)
        {
            unsigned int dim;

            if (t_max[0] + PRECISION < t_max[1])
            {
                dim = (t_max[0] + PRECISION < t_max[2]) ? 0 : 2;
            }
            else
            {
                dim = (t_max[1] + PRECISION < t_max[2]) ? 1 : 2;
            }

            if ((step[dim] < 0 && current_key[dim] == 0) || (step[dim] > 0 && current_key[dim] == 2 * this->tree_max_val - 1))    
            {
                std::cout << "Coordinate hit bounds in dim" << dim << ", aborting raycast" << std::endl;
                end = this->keyToCoord(current_key, depth);
                return false; 
            }

            previous_key = current_key;

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
                    std::cout << "Max range reached\n";
                    
                    return false;
                }
            }

            NodePtr current_node = this->search(current_key, depth);

            if (target_key == current_key)
            {
                done = true;
                break;
            }

            if (skip)
            {
                break_keys1.push_back(current_key);
                continue;
            }

            if (free && current_node && this->isNodeOccupied(current_node))
            {
                break_keys1.push_back(current_key);
                break_keys2.push_back(previous_key); 
                free = false;
            }
            else if (!free && (!current_node || (current_node && !this->isNodeOccupied(current_node))))
            {
                break_keys1.push_back(previous_key);
                break_keys2.push_back(current_key); 
                free = true;
            }
        }

        return true;
    }




    bool ExtendedOcTree::castRay3(const point3d& origin, const point3d& direction, const point3d& target, point3d& end,
        bool ignore_unknown, double max_range, int depth, double resolution, double t0, double t1, double t2, bool show) const
    {
        // Initialization phase -------------------------------------------------------

        OcTreeKey target_key, origin_key;
    
        this->coordToKeyChecked(target, depth, target_key);
        this->coordToKeyChecked(origin, depth, origin_key);

        OcTreeKey current_key;

        if (!this->coordToKeyChecked(origin, depth, current_key))
        {
            std::cout << "Coordinates out of bounds during ray casting" << std::endl;

            std::cout << origin << std::endl;

            return false;
        }


        NodePtr starting_node = this->search(current_key, depth);

        if (starting_node)
        {
            if (this->isNodeOccupied(starting_node))
            {
                end = this->keyToCoord(current_key, depth);

                return false;
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
        double t_delta[3];
        double t_max[3];

        t_max[0] = t0;
        t_max[1] = t1;
        t_max[2] = t2;

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

                t_delta[i] = resolution / std::fabs(normalized_direction(i));
            }
            else
            {
                t_delta[i] = std::numeric_limits<double>::max();
            }
        }

        if (step[0] == 0 && step[1] == 0 && step[2] == 0)
        {
            // std::cout << "Raycasting in direction (0,0,0) is not possible!" << std::endl;
            return false;
        }

        double max_range_sq = max_range * max_range;

        // Incremental phase  ---------------------------------------------------------

        bool done = false;

        OcTreeKey previous_key;

        if (show) std::cout << std::setprecision(12) << this->keyToCoord(current_key).x() << " " << this->keyToCoord(current_key).y() 
                << " " << this->keyToCoord(current_key).z() << " " << t_max[0] << " " << t_max[1] << " " << t_max[2] << "\n";

        while (!done)
        {
            unsigned int dim;

            if (t_max[0] + PRECISION < t_max[1])
            {
                dim = (t_max[0] + PRECISION < t_max[2]) ? 0 : 2;
            }
            else
            {
                dim = (t_max[1] + PRECISION < t_max[2]) ? 1 : 2;
            }

            if ((step[dim] < 0 && current_key[dim] == 0) || (step[dim] > 0 && current_key[dim] == 2 * this->tree_max_val - 1))    
            {
                std::cout << "Coordinate hit bounds in dim" << dim << ", aborting raycast" << std::endl;
                end = this->keyToCoord(current_key, depth);
                return false; 
            }

            previous_key = current_key;

            current_key[dim] += step[dim];
            t_max[dim] += t_delta[dim];
            
            end = this->keyToCoord(current_key, depth);

            if (show) std::cout << std::setprecision(12) << this->keyToCoord(current_key).x() << " " << this->keyToCoord(current_key).y() 
                << " " << this->keyToCoord(current_key).z() << " " << t_max[0] << " " << t_max[1] << " " << t_max[2] << "\n";        

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
                    return false;
                }
            }

            if (current_key[0] == target_key[0] && current_key[1] == target_key[1] && t_max[2] > std::min(t_max[0], t_max[1]) + PRECISION) break;
        }

        return true;
    }   



    bool ExtendedOcTree::castRay4(const point3d& origin, const point3d& directionP, point3d& end, bool ignoreUnknown, double maxRange, OcTreeKey target, bool show) const
    {
        /// ----------  see OcTreeBase::computeRayKeys  -----------

        // Initialization phase -------------------------------------------------------
        OcTreeKey current_key;
        if ( !this->coordToKeyChecked(origin, current_key) )
        {
            OCTOMAP_WARNING_STR("Coordinates out of bounds during ray casting");
            return false;
        }

        if (current_key == target)
        {
            return true;
        }

        NodePtr starting_node = this->search(current_key);

        if (starting_node)
        {
            if (this->isNodeOccupied(starting_node))
            {
                end = this->keyToCoord(current_key);
                return false;
            }
        } 
        else if(!ignoreUnknown)
        {
            end = this->keyToCoord(current_key);
            return false;
        }

        point3d direction = directionP.normalized();
        bool max_range_set = (maxRange > 0.0);

        int step[3];
        double tMax[3];
        double tDelta[3];

        for(unsigned int i = 0; i < 3; ++i) 
        {
            // compute step direction
            if (direction(i) > 0.0) step[i] = 1;
            else if (direction(i) < 0.0)   step[i] = -1;
            else step[i] = 0;

            // compute tMax, tDelta
            if (step[i] != 0) 
            {
                // corner point of voxel (in direction of ray)
                double voxelBorder = this->keyToCoord(current_key[i]);
                voxelBorder += double(step[i] * this->resolution * 0.5);

                tMax[i] = ( voxelBorder - origin(i) ) / direction(i);
                tDelta[i] = this->resolution / fabs( direction(i) );
            }
            else 
            {
                tMax[i] =  std::numeric_limits<double>::max();
                tDelta[i] = std::numeric_limits<double>::max();
            }
        }

        if (step[0] == 0 && step[1] == 0 && step[2] == 0)
        {
            // OCTOMAP_ERROR("Raycasting in direction (0,0,0) is not possible!");
            return false;
        }

        // for speedup:
        double maxrange_sq = maxRange *maxRange;

        // Incremental phase  ---------------------------------------------------------

        bool done = false;

        if (show) std::cout << tMax[0] << " " << tMax[1] << " " << tMax[2] << " " << 
            this->keyToCoord(current_key[0]) <<  " " << this->keyToCoord(current_key[1]) << " " << this->keyToCoord(current_key[2]) << "\n";

        while (!done) 
        {
            unsigned int dim;

            // find minimum tMax:
            if (tMax[0] + PRECISION < tMax[1])
            {
                if (tMax[0] + PRECISION < tMax[2]) dim = 0;
                else                   dim = 2;
            }
            else 
            {
                if (tMax[1] + PRECISION < tMax[2]) dim = 1;
                else                   dim = 2;
            }

            // check for overflow:
            if ((step[dim] < 0 && current_key[dim] == 0) || (step[dim] > 0 && current_key[dim] == 2* this->tree_max_val-1))
            {
                OCTOMAP_WARNING("Coordinate hit bounds in dim %d, aborting raycast\n", dim);
                // return border point nevertheless:
                end = this->keyToCoord(current_key);
                return false;
            }

            // advance in direction "dim"
            current_key[dim] += step[dim];
            tMax[dim] += tDelta[dim];

            if (show) std::cout << tMax[0] << " " << tMax[1] << " " << tMax[2] << " " << 
                this->keyToCoord(current_key[0]) <<  " " << this->keyToCoord(current_key[1]) << " " << this->keyToCoord(current_key[2]) << "\n";

            // generate world coords from key
            end = this->keyToCoord(current_key);

            // check for maxrange:
            if (max_range_set)
            {
                double dist_from_origin_sq(0.0);
                for (unsigned int j = 0; j < 3; j++) 
                {
                    dist_from_origin_sq += ((end(j) - origin(j)) * (end(j) - origin(j)));
                }

                if (dist_from_origin_sq > maxrange_sq) return false;
            }

            if (current_key == target) 
            {
                return true;
            }

            NodePtr current_node = this->search(current_key);

            if (current_node)
            {
                if (this->isNodeOccupied(current_node)) 
                {
                    done = true;
                    break;
                }
                // otherwise: node is free and valid, raycasting continues
            } 
            else if (!ignoreUnknown)
            { // no node found, this usually means we are in "unknown" areas
                return false;
            }
        } // end while

        return false;
    }



    bool ExtendedOcTree::checkBreakPoints(const std::vector<OcTreeKey>& break_keys, const std::vector<OcTreeKey>& break_keys_n, 
        const point3d& p_target_3d, const point3d& p_origin_3d, const double floor_plan_level, bool show)
    {
        const double resolution = this->getResolution();
        const int depth = this->getTreeDepth();

        OcTreeKey k_target_3d = this->coordToKey(p_target_3d, depth);
        OcTreeKey k_origin_3d = this->coordToKey(p_origin_3d, depth);

        point3d p_target_2d = point3d(p_target_3d.x(), p_target_3d.y(), floor_plan_level);
        point3d p_origin_2d = point3d(p_origin_3d.x(), p_origin_3d.y(), floor_plan_level);

        double distance_2d = (p_target_2d - p_origin_2d).norm();

        point3d direction_3d = p_target_3d - p_origin_3d;
        point3d direction_3d_normalized = direction_3d.normalized();

        std::vector<point3d> break_points(break_keys.size());
        std::vector<double> break_distances(break_keys.size());

        for (int i = 0; i < break_points.size(); ++i)
        {
            break_points[i] = this->keyToCoord(break_keys[i], depth);
            break_distances[i] = (break_points[i] - p_origin_2d).norm() / distance_2d;
        }

        for (int i = 0; i < break_keys.size(); i += 2)
        {
            point3d p_break_3d_1 = break_points[i];
            point3d p_break_3d_2 = break_points[i + 1];

            int step_x = std::abs(break_keys[i][0] - k_origin_3d[0]);
            int step_y = std::abs(break_keys[i][1] - k_origin_3d[1]);

            double step_z = 0;

            double t_max[3];

            t_max[0] = std::numeric_limits<double>::max();
            t_max[1] = std::numeric_limits<double>::max();
            t_max[2] = std::numeric_limits<double>::max();

            double t_z = 0.5 * resolution / std::fabs(direction_3d_normalized.z());


            if (step_x == 0 && step_y == 0)
            {
                t_max[0] = (direction_3d_normalized.x() == 0) ? std::numeric_limits<double>::max() : (double(step_x) + 0.5) * resolution / std::fabs(direction_3d_normalized.x());

                t_max[1] = (direction_3d_normalized.y() == 0) ? std::numeric_limits<double>::max() : (double(step_y) + 0.5) * resolution / std::fabs(direction_3d_normalized.y());            
            }
            else if (break_keys[i][0] != break_keys_n[i][0])
            {
                t_max[0] = (double(step_x - 1) + 0.5) * resolution / std::fabs(direction_3d_normalized.x());

                t_max[1] = (direction_3d_normalized.y() == 0) ? std::numeric_limits<double>::max() : (double(step_y) + 0.5) * resolution / std::fabs(direction_3d_normalized.y());
            }
            else if (break_keys[i][1] != break_keys_n[i][1])
            {
                t_max[0] = (direction_3d_normalized.x() == 0) ? std::numeric_limits<double>::max() : (double(step_x) + 0.5) * resolution / std::fabs(direction_3d_normalized.x());

                t_max[1] = (double(step_y - 1) + 0.5) * resolution / std::fabs(direction_3d_normalized.y());
            }


            if (k_target_3d[2] == k_origin_3d[2])
            {
                t_max[2] = std::numeric_limits<double>::max();

                p_break_3d_1.z() = p_origin_3d.z();
            }
            else
            {
                if (step_x == 0 && step_y == 0)
                {
                    step_z = 0;
                }
                else
                {
                    step_z = std::ceil((std::min<double>(t_max[0], t_max[1]) - t_z + rpo::PRECISION) / (resolution / std::fabs(direction_3d_normalized.z())));
                }

                t_max[2] = t_z + step_z * resolution / std::fabs(direction_3d_normalized.z());

                if (k_target_3d[2] < k_origin_3d[2]) step_z *= -1;

                p_break_3d_1.z() = p_origin_3d.z() + step_z * resolution;
            }


            if (!(step_x == 0 && step_y == 0))
            {
                if (break_keys[i][0] != break_keys_n[i][0])
                {
                    t_max[0] += resolution / std::fabs(direction_3d_normalized.x());
                }
                else if (break_keys[i][1] != break_keys_n[i][1])
                {
                    t_max[1] += resolution / std::fabs(direction_3d_normalized.y());
                }
            }


            if (break_keys[i][0] == break_keys[i+1][0] && break_keys[i][1] == break_keys[i+1][1])
            {
                point3d p_query = p_break_3d_1;

                rpo::NodePtr n_single = this->search(p_query, depth);

                if (n_single && this->isNodeOccupied(n_single))
                {
                    return false;
                }

                while (t_max[2] <= std::min(t_max[0], t_max[1])) 
                {
                    if (k_target_3d[2] < k_origin_3d[2])
                    {
                        p_query.z() -= resolution;
                    }
                    else
                    {
                        p_query.z() += resolution;
                    }

                    n_single = this->search(p_query, depth);

                    if (n_single && this->isNodeOccupied(n_single))
                    {
                        return false;
                    }

                    t_max[2] += resolution / std::fabs(direction_3d_normalized.z());
                }
            }
            else
            {
                point3d p_end;

                if (!this->castRay3(p_break_3d_1, direction_3d, p_break_3d_2, p_end, true, 10, depth, resolution, t_max[0], t_max[1], t_max[2], show))
                {   
                    return false;
                }
            }
        }

        return true;
    }










    bool ExtendedOcTree::evaluateRayCast(const point3d& target_point, const point3d& end_point, int depth) const
    {
        OcTreeKey target_key, end_key;

        if (this->coordToKeyChecked(target_point, depth, target_key) && this->coordToKeyChecked(end_point, depth, end_key))
        {
            return (target_key == end_key);
        }

        return false;
    }


    bool ExtendedOcTree::checkRayCast(bool good, const point3d& target_point, const point3d& origin, const point3d& direction, 
        point3d& end_point, double max_range, int depth, double resolution, bool ignore_unknown) const
    {
        if (castRay(origin, direction, target_point, end_point, ignore_unknown, max_range, depth, resolution))
        {
            if (!good && !evaluateRayCast(target_point, end_point, depth) ||
                 good &&  evaluateRayCast(target_point, end_point, depth))
            {
                return true;
            }
        }

        return false;
    }


    void ExtendedOcTree::compute3DNormalVectors()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr octree_points (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr octree_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimation;
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


    void ExtendedOcTree::computeGroundLevel()
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


    void ExtendedOcTree::findObjects(bool surface)
    {
        std::cout << "Find objects ... " << std::endl;

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

                for (int z = 0; z < 10; ++z)
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


    void ExtendedOcTree::visualize()
    {
        ColorOcTree color_model(this->resolution);

        for (ExtendedOcTree::leaf_iterator it = this->begin_leafs(), end = this->end_leafs(); it != end; ++it)
        {
            ColorOcTreeNode* color_node = color_model.updateNode(it.getKey(), true);
        }

        color_model.expand();

        for (ExtendedOcTree::leaf_iterator it = this->begin_leafs(), end = this->end_leafs(); it != end; ++it)
        {
            NodePtr node = this->search(it.getKey(), this->tree_depth);

            ColorOcTreeNode* color_node = color_model.search(it.getKey(), this->tree_depth);

            switch (node->getType())
            {
            case 1: // General
                color_node->setColor(GOLD);
                break;
            case 2: // Ground
                color_node->setColor(GREEN);
                break;
            case 3: // Underground
                color_node->setColor(BLACK);
                break;
            case 4: // Wall
                color_node->setColor(CRIMSON_RED);
                break;
            case 5: // Object
                color_node->setColor(SAPPHIRE_BLUE);
                break;
            default:
                break;
            }
        }

        ros::NodeHandle node_handle;
        ros::Publisher publisher = node_handle.advertise<octomap_msgs::Octomap>("/segmented_map", 1);

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


    std::array<double, 6> ExtendedOcTree::getBoundaries() const
    {
        std::array<double, 6> boundaries;

        this->getMetricMin(boundaries[0], boundaries[2], boundaries[4]);
        this->getMetricMax(boundaries[1], boundaries[3], boundaries[5]);

        return boundaries;
    }
}
