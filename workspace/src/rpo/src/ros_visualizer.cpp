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

        file.open(m_parameters.paths.lamp_model);

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

        point3d base(x, y, m_ground_level + m_resolution);

        m_color_model->coordToKeyChecked(base, m_depth, key);

        base = m_color_model->keyToCoord(key, m_depth);

        for (const LampModelElement& element : m_lamp_model)
        {
            const point3d lamp_point = base + point3d(
                element.x * m_resolution, 
                element.y * m_resolution, 
                element.z * m_resolution);
            
            const ColorOcTreeNode::Color color = element.color ? 
                  ColorOcTreeNode::Color(RUBY_RED2) : 
                  ColorOcTreeNode::Color(80, 80, 80);

            ColorOcTreeNode* node = m_color_model->search(lamp_point, m_depth);

            if (node == nullptr)
            {
                m_color_model->updateNode(lamp_point, true);
                node = m_color_model->search(lamp_point, m_depth);
            }

            m_color_model->expandNode(node);
            m_color_model->pruneNode(node);

            node->setColor(color);

            m_color_model->coordToKeyChecked(lamp_point, m_depth, key);

            m_placed_lamps.insert(key);
        }

        m_color_model->expand();
    }




    void ROSVisualizer::placeLamp2(double x, double y)
    {
        point3d center(x, y, m_ground_level);

        for (int x = -2; x < 3; ++x)
        {
            for (int y = -2; y < 3; ++y)
            {   
                point3d point = center + point3d(x * m_resolution, y * m_resolution, 0);

                ColorOcTreeNode* color_node = m_color_model->search(point);

                // Grid points - ruby red
                color_node->setColor(RUBY_RED2);
            }
        }  
    }




    void ROSVisualizer::deleteLamps()
    {
        for (const auto& key : m_placed_lamps)
        {
            ColorOcTreeNode* node = m_color_model->search(key, m_depth);

            if (node != nullptr)
            {
                m_color_model->deleteNode(key, m_depth);
            }
        }
    }



    void ROSVisualizer::cutUnderGround()
    {
        std::cout << "Before cut: " << m_augmented_model->getNumLeafNodes() << std::endl;

        KeySet points_to_delete;

        for (AugmentedOcTree::leaf_iterator it = m_augmented_model->begin_leafs(), end = m_augmented_model->end_leafs(); it != end; ++it)
        {
            NodePtr node = m_augmented_model->search(it.getKey(), m_depth);

            if (node != nullptr && node->getType() == 3)
            {
                points_to_delete.insert(it.getKey());
            }
        }

        for (const auto& key : points_to_delete)
        {
            m_augmented_model->deleteNode(key, m_depth);
            m_color_model->deleteNode(key, m_depth);
        }
    
        std::cout << "After cut: " << m_augmented_model->getNumLeafNodes() << std::endl;
    }



    void ROSVisualizer::cutUnderGround2()
    {
        
        for (AugmentedOcTree::leaf_iterator it = m_augmented_model->begin_leafs(), end = m_augmented_model->end_leafs(); it != end; ++it)
        {
            NodePtr node = m_augmented_model->search(it.getKey(), m_depth);

            if (node != nullptr && it.getCoordinate().z() < m_ground_level - 0.5)
            {
                ColorOcTreeNode* cnode = m_color_model->updateNode(it.getKey(), true);
            }
        }
        

        KeySet points_to_delete;

        for (AugmentedOcTree::leaf_iterator it = m_augmented_model->begin_leafs(), end = m_augmented_model->end_leafs(); it != end; ++it)
        {
            NodePtr node = m_augmented_model->search(it.getKey(), m_depth);

            if (node != nullptr && it.getCoordinate().z() < m_ground_level - 0.5)
            {
                points_to_delete.insert(it.getKey());
            }
        }

        for (const auto& key : points_to_delete)
        {
            m_augmented_model->deleteNode(key, m_depth);
            // m_color_model->deleteNode(key, m_depth);
        }
    }



    void ROSVisualizer::filter()
    {
        auto boundaries = m_augmented_model->getBoundaries();

        point3d p(boundaries[1] + 0.025 - 1.7, boundaries[3] + 0.025 - 1.7, boundaries[5] + 0.025);

        KeySet points_to_delete;

        for (AugmentedOcTree::leaf_iterator it = m_augmented_model->begin_leafs(), end = m_augmented_model->end_leafs(); it != end; ++it)
        {
            point3d query_point = it.getCoordinate();

            if (query_point.x() > p.x() && query_point.y() > p.y())
            {
                points_to_delete.insert(it.getKey());
            }
        }

        std::cout << points_to_delete.size() << " points found\n";

        for (const auto& key : points_to_delete)
        {
            NodePtr node = m_augmented_model->search(key, m_depth); 

            if (node != nullptr)
            {
                m_augmented_model->deleteNode(key, m_depth);
                m_color_model->deleteNode(key, m_depth);
            }
        }

        std::cout << m_augmented_model->getNumLeafNodes() << "\n";
    }






    void ROSVisualizer::showOptimizationElements()
    {
        for (ColorOcTree::leaf_iterator it = m_color_model->begin_leafs(), end = m_color_model->end_leafs(); it != end; ++it)
        {
            if (ColorOcTreeNode* node = m_color_model->search(it.getKey(), m_depth); node != nullptr)
            {
                if (m_optimization_elements.find(it.getKey()) != m_optimization_elements.end())
                {
                    // node->setColor(255, 255, 255);
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
                            color_node->setColor(RUBY_RED2);
                            break;
                        case 3: // Underground - black
                            color_node->setColor(BLACK);
                            break;
                        case 4: // Wall - orange
                            color_node->setColor(ORANGE);
                            break;
                        case 5: // Object - blue
                            color_node->setColor(SAPPHIRE_BLUE);
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
        

        
        for (const auto& grid_element : m_grid_elements)
        {
            point3d center = m_color_model->keyToCoord(grid_element, m_depth);

            for (int x = -2; x < 3; ++x)
            {
                for (int y = -2; y < 3; ++y)
                {   
                    point3d point = center + point3d(x * m_resolution, y * m_resolution, 0);

                    ColorOcTreeNode* color_node = m_color_model->search(point);

                    // Grid points - purple
                    // color_node->setColor(RED);
                }
            }  
        }

        publish();
    }



    void ROSVisualizer::showCoverage(ExposureMap& dose_map, bool binary, bool grid)
    {
        for (ColorOcTree::leaf_iterator it = m_color_model->begin_leafs(), end = m_color_model->end_leafs(); it != end; ++it)
        {
            const OcTreeKey key = it.getKey();

            if (m_placed_lamps.find(key) == m_placed_lamps.end())
            {
                if (ColorOcTreeNode* node = m_color_model->search(key, m_depth); node != nullptr)
                {
                    if (dose_map.find(key) != dose_map.end())
                    {
                        const double exposure = dose_map[key];

                        if (exposure >= m_parameters.computation.exposure_limit)
                        {
                            node->setColor(static_cast<int>(255 * R[255]), static_cast<int>(255 * G[255]), static_cast<int>(255 * B[255]));
                            // node->setColor(255, 255, 0);
                            // node->setColor(0, 255, 255);
                        }
                        else
                        {
                            if (binary)
                            {
                                node->setColor(static_cast<int>(255 * R[0]), static_cast<int>(255 * G[0]), static_cast<int>(255 * B[0]));
                            }
                            else
                            {
                                const int scale = static_cast<int>(round(255 * exposure / m_parameters.computation.exposure_limit));

                                node->setColor(static_cast<int>(255 * R[scale]), static_cast<int>(255 * G[scale]), static_cast<int>(255 * B[scale]));

                                // node->setColor(scale, scale, 0);
                                // node->setColor(0, scale, scale);
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
        int computation_type = m_parameters.computation.type;

        if (verify)
        {
            m_parameters.computation.type = 7;
        }

        const int element_size = m_parameters.optimization.element_size;

        std::vector<double> elements = radiation_plan.first;

        std::vector<ExposureMap> exposure_maps(radiation_plan.first.size() / 3);

        const double z = m_parameters.lamp.center;

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

        int general_over = 0, object_over = 0, object_sum = m_object_elements.size();

        rpo::ExposureMap exposure_map;

        for (auto& element : m_verification_elements)
        {
            exposure_map[element] = 0;

            for (int i = 0; i < exposure_maps.size(); ++i)
            {
                exposure_map[element] += exposure_maps[i][element];
            }


            if (std::isnan(exposure_map[element]))
            {
                exposure_map[element] = 0;
            }


            if (exposure_map[element] >= m_parameters.computation.exposure_limit)
            {
                general_over += 1;
            }


            NodePtr node = m_augmented_model->search(element, m_depth);

            if (node != nullptr && node->getType() == 5)
            {
                if ((!verify && (m_optimization_elements.find(element) != m_optimization_elements.end())) ||
                    ( verify && (m_verification_elements.find(element) != m_verification_elements.end())))
                {
                    if (exposure_map[element] >= m_parameters.computation.exposure_limit)
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

        showCoverage(exposure_map, false, true);

        for (int i = 0; i < elements.size(); i += element_size)
        {
            placeLamp2(elements[i], elements[i + 1]);
        }


        publish();

        deleteLamps();

        m_parameters.computation.type = computation_type;
    
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
 