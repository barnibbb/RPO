#include "ros_visualizer.h"

namespace rpo
{
    void handler2(int s)
    {
        exit(1);
    }



    ROSVisualizer::ROSVisualizer(const std::shared_ptr<AugmentedOcTree> augmented_model, 
        const std::shared_ptr<ColorOcTree> color_model, const Parameters& parameters) : 
        DoseCalculator(augmented_model, parameters)
    {
        m_model_publisher = m_node_handle.advertise<octomap_msgs::Octomap>("/exposure_map", 10);

        m_color_model = color_model;

        readLampModel();
    }



    void ROSVisualizer::readLampModel()
    {
        std::ifstream file;

        file.open(m_parameters.lamp_voxel_model_file);

        if (file.is_open())
        {
            std::string line;

            while(getline(file, line))
            {
                std::vector<std::string> data;

                boost::split(data, line, boost::is_any_of(","));

                bool color = (data[3] == "1");

                LampModelElement lamp_element { stoi(data[0]), stoi(data[1]), stoi(data[2]), color };
            
                m_lamp_model.push_back(lamp_element);
            }
            
            file.close();
        }
        else
        {
            throw std::runtime_error { "Could not open the lamp model file!" };
        }
    }



    void ROSVisualizer::placeLamp(double x, double y)
    {
        OcTreeKey key;

        const double resolution = m_parameters.resolution;

        point3d base(x, y, m_parameters.ground_level + resolution);

        m_color_model->coordToKeyChecked(base, m_parameters.depth, key);

        base = m_color_model->keyToCoord(key, m_parameters.depth);

        for (const LampModelElement& element : m_lamp_model)
        {
            const point3d lamp_point = base + point3d(
                element.x * resolution, 
                element.y * resolution, 
                element.z * resolution);
            
            const ColorOcTreeNode::Color color = element.color ? 
                  ColorOcTreeNode::Color(255, 0, 255) : 
                  ColorOcTreeNode::Color(80, 80, 80);

            ColorOcTreeNode* node = m_color_model->search(lamp_point, m_parameters.depth);

            if (node == nullptr)
            {
                m_color_model->updateNode(lamp_point, true);
                node = m_color_model->search(lamp_point, m_parameters.depth);
            }

            m_color_model->expandNode(node);
            m_color_model->pruneNode(node);

            node->setColor(color);

            m_color_model->coordToKeyChecked(lamp_point, m_parameters.depth, key);

            m_placed_lamps.insert(key);
        }

        m_color_model->expand();
    }



    void ROSVisualizer::deleteLamps()
    {
        for (const auto& key : m_placed_lamps)
        {
            ColorOcTreeNode* node = m_color_model->search(key, m_parameters.depth);

            if (node != nullptr)
            {
                m_color_model->deleteNode(key, m_parameters.depth);
            }
        }
    }



    void ROSVisualizer::cutUnderGround()
    {
        KeySet points_to_delete;

        for (AugmentedOcTree::leaf_iterator it = m_augmented_model->begin_leafs(), end = m_augmented_model->end_leafs(); it != end; ++it)
        {
            NodePtr node = m_augmented_model->search(it.getKey(), m_parameters.depth);

            if (node != nullptr && node->getType() == 3)
            {
                points_to_delete.insert(it.getKey());
            }
        }

        for (const auto& key : points_to_delete)
        {
            m_augmented_model->deleteNode(key, m_parameters.depth);
            m_color_model->deleteNode(key, m_parameters.depth);
        }
    }



    void ROSVisualizer::showOptimizationElements()
    {
        for (ColorOcTree::leaf_iterator it = m_color_model->begin_leafs(), end = m_color_model->end_leafs(); it != end; ++it)
        {
            if (ColorOcTreeNode* node = m_color_model->search(it.getKey(), m_parameters.depth); node != nullptr)
            {
                if (m_optimization_elements.find(it.getKey()) != m_optimization_elements.end())
                {
                    node->setColor(255, 255, 255);
                }
                else
                {
                    node->setColor(0, 0, 0);
                }
            }
        }

        publish();
    }



    void ROSVisualizer::showElementTypes()
    {
        for (AugmentedOcTree::leaf_iterator it = m_augmented_model->begin_leafs(), end = m_augmented_model->end_leafs(); it != end; ++it)
        {
            AugmentedOcTreeNode* augmented_node = m_augmented_model->search(it.getKey());

            ColorOcTreeNode* color_node = m_color_model->search(it.getKey());

            if (augmented_node != nullptr && color_node != nullptr)
            {
                if (m_verification_elements.find(it.getKey()) != m_verification_elements.end())
                {
                    switch (augmented_node->getType())
                    {
                        case 1: // General - orange
                            color_node->setColor(ORANGE);
                            break;
                        case 2: // Ground - red
                            color_node->setColor(RED);
                            break;
                        case 3: // Underground - black
                            color_node->setColor(BLACK);
                            break;
                        case 4: // Wall - orange
                            color_node->setColor(ORANGE);
                            break;
                        case 5: // Object - blue
                            color_node->setColor(ORANGE);
                            break;
                        default:
                            break;
                    }

                    if (m_ground_zone_elements.find(it.getKey()) != m_ground_zone_elements.end())
                    {
                        // Ground zone - green
                        color_node->setColor(GREEN);
                    }   
                }
                else
                {
                    // Unreachable elements - black
                    color_node->setColor(BLACK);
                }
            }   
        }

        // for (const auto& grid_element : m_grid_elements)
        // {
        //     point3d center = m_color_model->keyToCoord(grid_element, m_parameters.depth);

        //     for (int x = -2; x < 3; ++x)
        //     {
        //         for (int y = -2; y < 3; ++y)
        //         {   
        //             point3d point = center + point3d(x * m_parameters.resolution, y * m_parameters.resolution, 0);

        //             ColorOcTreeNode* color_node = m_color_model->search(point);

        //             // Grid points - purple
        //             color_node->setColor(PURPLE);
        //         }
        //     }  
        // }

        publish();
    }



    void ROSVisualizer::showCoverage(ExposureMap& dose_map, bool binary)
    {
        for (ColorOcTree::leaf_iterator it = m_color_model->begin_leafs(), end = m_color_model->end_leafs(); it != end; ++it)
        {
            const OcTreeKey key = it.getKey();

            if (m_placed_lamps.find(key) == m_placed_lamps.end())
            {
                if (ColorOcTreeNode* node = m_color_model->search(key, m_parameters.depth); node != nullptr)
                {
                    if (dose_map.find(key) != dose_map.end())
                    {
                        const double exposure = dose_map[key];

                        if (exposure >= m_parameters.exposure_limit)
                        {
                            node->setColor(static_cast<int>(255 * R[255]), static_cast<int>(255 * G[255]), static_cast<int>(255 * B[255]));
                        }
                        else
                        {
                            if (binary)
                            {
                                node->setColor(static_cast<int>(255 * R[0]), static_cast<int>(255 * G[0]), static_cast<int>(255 * B[0]));
                            }
                            else
                            {
                                const int scale = static_cast<int>(round(255 * exposure / m_parameters.exposure_limit));

                                node->setColor(static_cast<int>(255 * R[scale]), static_cast<int>(255 * G[scale]), static_cast<int>(255 * B[scale]));
                            }
                        }
                
                    }
                    else
                    {
                        node->setColor(0, 0, 0);
                    }
                }
            }
        }

        publish();
    }



    Score ROSVisualizer::showResult(const RadiationPlan& radiation_plan, bool verify)
    {
        int computation_type = m_parameters.computation_type;

        if (verify)
        {
            m_parameters.computation_type = 7;
        }

        const int element_size = m_parameters.plan_element_size;

        std::vector<double> elements = radiation_plan.first;

        std::vector<ExposureMap> exposure_maps(radiation_plan.first.size() / 3);

        const double z = m_parameters.lamp_center;

        #pragma omp parallel for
        for (int i = 0; i < elements.size(); i += element_size)
        {   
            point3d lamp_position(elements[i], elements[i + 1], z);

            PlanElement plan_element { lamp_position, elements[i + 2] };

            exposure_maps[i / element_size] = computeDoseForPlanElement(plan_element, verify);
        }

        for (int i = 0; i < elements.size(); i += element_size)
        {
            placeLamp(elements[i], elements[i + 1]);
        }

        int general_over = 0, object_over = 0, object_sum = 0;

        for (auto& element : exposure_maps[0])
        {
            for (int i = 1; i < exposure_maps.size(); ++i)
            {
                element.second += exposure_maps[i][element.first];
            }

            if (std::isnan(element.second))
            {
                element.second = 0;
            }

            if (element.second > m_parameters.exposure_limit)
            {
                general_over += 1;
            }

            NodePtr node = m_augmented_model->search(element.first, m_parameters.depth);

            if (node != nullptr && node->getType() == 5)
            {
                if ((!verify && (m_optimization_elements.find(element.first) != m_optimization_elements.end())) ||
                    ( verify && (m_verification_elements.find(element.first) != m_verification_elements.end())))
                {
                    object_sum += 1;
                    
                    if (element.second >= m_parameters.exposure_limit)
                    {
                        object_over += 1;
                    }
                }
            }
        }

        Score score;

        if (!verify)
        {
            score.general_coverage = static_cast<double>(general_over) / static_cast<double>(m_optimization_elements.size());
        }
        else
        {
            score.general_coverage = static_cast<double>(general_over) / static_cast<double>(m_verification_elements.size());
        }

        score.object_coverage  = static_cast<double>(object_over)  / static_cast<double>(object_sum);

        showCoverage(exposure_maps[0], false);

        publish();

        deleteLamps();

        m_parameters.computation_type = computation_type;
    
        return score;
    }



    void ROSVisualizer::publish()
    {
        signal (SIGINT, handler2);

        if (m_augmented_model != nullptr)
        {
            octomap_msgs::Octomap message;

            std::stringstream data_stream;
            m_color_model->writeData(data_stream);
            std::string datastring = data_stream.str();

            message.data = std::vector<int8_t>(datastring.begin(), datastring.end());
            message.resolution = m_color_model->getResolution();
            message.id = m_color_model->getTreeType();
            message.binary = false;
            message.header.frame_id = "map";
            message.header.stamp = ros::Time();
        
            m_model_publisher.publish(message);
        }
    }
}
 