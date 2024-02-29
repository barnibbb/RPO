#include "dose_calculator.h"
namespace rpo
{
    void handler(int s)
    {
        exit(1);
    }



    DoseCalculator::DoseCalculator(const std::shared_ptr<AugmentedOcTree> augmented_model, const Parameters& parameters)
    {
        m_augmented_model = augmented_model;

        m_parameters = parameters;
    }



    Parameters DoseCalculator::getParameters() const
    {
        return m_parameters;
    }



    void DoseCalculator::computeGroundZone()
    {
        const float resolution = m_parameters.resolution;

        const int depth = m_parameters.depth;

        KeySet ground_zone;

        bool ground_level_set = false;

        for (rpo::AugmentedOcTree::leaf_iterator it = m_augmented_model->begin_leafs(), 
            end = m_augmented_model->end_leafs(); it != end; ++it)
        {
            NodePtr node = m_augmented_model->search(it.getKey(), depth);

            if (node != nullptr && node->getType() == 2)
            {
                point3d point = it.getCoordinate();

                if (!ground_level_set)
                {
                    m_parameters.ground_level = point.z();

                    m_parameters.lamp_center = m_parameters.ground_level + resolution / 2 +
                        m_parameters.lamp_offset + m_parameters.lamp_height / 2;

                    m_parameters.lamp_top = m_parameters.lamp_center + m_parameters.lamp_height / 2;

                    ground_level_set = true;
                }

                if (ground_level_set)
                {
                    bool ground = true;

                    point.z() += resolution;

                    while (point.z() < m_parameters.lamp_top)
                    {
                        NodePtr upper_node = m_augmented_model->search(point, depth);

                        if (upper_node == nullptr)
                        {
                            point.z() += resolution;
                        }
                        else
                        {
                            ground = false;
                            break;
                        }
                    }

                    if (ground)
                    {
                        ground_zone.insert(it.getKey());
                    }
                }
            }
        }

        for (const auto& key : ground_zone)
        {
            const point3d center = m_augmented_model->keyToCoord(key, depth);

            bool ground = true;

            // For each possible ground zone element it checked whether its
            // neighbors are also ground zone elements. If all neighbors in
            // a given region also belong to the ground zone, then the center
            // is considered as real ground zone element.
            int min_dist = static_cast<int>(m_parameters.safety_radius / m_parameters.resolution) + 1;

            for (int i = -min_dist; i <= min_dist; ++i)
            {
                for (int j = -min_dist; j <= min_dist; ++j)
                {
                    const point3d neighbor = center + point3d(i * resolution, j * resolution, 0);

                    if (OcTreeKey neighbor_key; m_augmented_model->coordToKeyChecked(neighbor, depth, neighbor_key))
                    {
                        if (ground_zone.find(neighbor_key) == ground_zone.end())
                        {
                            ground = false;
                            break;
                        }
                    }
                }

                if (!ground)
                {
                    break;
                }
            }

            if (ground)
            {
                if (NodePtr node = m_augmented_model->search(key, depth); node != nullptr)
                {
                    m_ground_zone_elements.insert(key);
                }
            }
        }

        std::cout << "Ground zone elements: " << m_ground_zone_elements.size() << std::endl;
    }   


    
    void DoseCalculator::computeGridElements()
    {
        const double grid_distance = m_parameters.grid_distance * m_parameters.resolution;

        for (AugmentedOcTree::leaf_iterator it = m_augmented_model->begin_leafs(), end = m_augmented_model->end_leafs(); it != end; ++it)
        {
            point3d point = it.getCoordinate();

            if (m_ground_zone_elements.find(it.getKey()) != m_ground_zone_elements.end())
            {
                bool place_new = true;

                for (const auto& key : m_grid_elements)
                {
                    point3d grid_point = m_augmented_model->keyToCoord(key);

                    if (point.distance(grid_point) < grid_distance)
                    {
                        place_new = false;
                        break;
                    }
                }

                if (place_new)
                {
                    m_grid_elements.push_back(it.getKey());
                }
            }
        }

        std::cout << "Grid elements: " << m_grid_elements.size() << std::endl;
    }



    void DoseCalculator::computeRayTargets()
    {
        double height = m_parameters.ground_level + m_parameters.resolution / 2.0 + m_parameters.lamp_offset;

        point3d continuous_height(0, 0, height);

        if (OcTreeKey key; m_augmented_model->coordToKeyChecked(continuous_height, m_parameters.depth, key))
        {
            point3d discrete_height = m_augmented_model->keyToCoord(key, m_parameters.depth);

            height = discrete_height.z();

            while (height < m_parameters.lamp_top)
            {
                m_ray_targets.push_back(height);

                height += m_parameters.resolution;
            }
        }

        std::cout << "Lamp elements: " << m_ray_targets.size() << std::endl;
    }



    std::multimap<double, double> DoseCalculator::getGroundZone() const
    {
        std::multimap<double, double> ground_zone;

        for (const auto& key : m_ground_zone_elements)
        {
            const point3d& point = m_augmented_model->keyToCoord(key, m_parameters.depth);

            ground_zone.insert({ point.x(), point.y() });
        }

        return ground_zone;
    }



    RadiationPlan DoseCalculator::getGrid(const RadiationPlan& plan) const
    {
        const int element_size = m_parameters.plan_element_size;

        RadiationPlan grid_plan = plan;

        for (int i = 0; i < grid_plan.first.size(); i += element_size)
        {
            double min_distance = std::numeric_limits<double>::max();

            OcTreeKey grid_key;
                
            for (const auto& element : m_grid_elements)
            {
                point3d grid_position = m_augmented_model->keyToCoord(element, m_parameters.depth);

                point3d lamp_position(grid_plan.first[i], grid_plan.first[i + 1], m_parameters.ground_level);

                const double distance = (grid_position - lamp_position).norm();
                
                if (distance < min_distance)
                {
                    min_distance = distance;
                    grid_key = element;
                }
            }

            point3d closest_point = m_augmented_model->keyToCoord(grid_key, m_parameters.depth);

            grid_plan.first[i] = closest_point.x();
            grid_plan.first[i + 1] = closest_point.y();
        }

        return grid_plan;
    }



    void DoseCalculator::computeIrradianceMaps()
    {
        if (m_parameters.store_maps)
        {
            m_irradiance_maps.resize(m_grid_elements.size());
        }
        
        #pragma omp parallel for
        for (int i = 0; i < m_grid_elements.size(); ++i)
        {
            point3d position = m_augmented_model->keyToCoord(m_grid_elements[i], m_parameters.depth);

            position.z() = m_parameters.lamp_center;

            ExposureMap irradiance_map;

            if (m_parameters.store_maps)
            {
                m_irradiance_maps[i] = computeIrradianceForPosition(position, false);
            }
            else
            {
                irradiance_map = computeIrradianceForPosition(position, false);
            }
            

            if (m_parameters.save_maps)
            {
                OcTreeKey plan_element_key;

                m_augmented_model->coordToKeyChecked(position, m_parameters.depth, plan_element_key);

                if (m_parameters.store_maps)
                {
                    saveIrradianceMap(plan_element_key, m_irradiance_maps[i]);
                }
                else
                {
                    saveIrradianceMap(plan_element_key, irradiance_map);
                }
            }
        }


        if (m_parameters.store_maps)
        {
            for (int i = 0; i < m_grid_elements.size(); ++i)      
            {
                for (const auto& element : m_irradiance_maps[i])
                {
                    if (element.second > 0)
                    {
                        m_verification_elements.insert(element.first);
                    }
                }
            } 
        }
        else
        {
            for (int i = 0; i < m_grid_elements.size(); ++i)      
            {
                ExposureMap irradiance_map = loadIrradianceMap(m_grid_elements[i]);

                for (const auto& element : irradiance_map)
                {
                    if (element.second > 0)
                    {
                        m_verification_elements.insert(element.first);
                    }
                }
            }
        }
    }



    void DoseCalculator::loadIrradianceMaps()
    {
        if (m_parameters.store_maps)
        {
            m_irradiance_maps.resize(m_grid_elements.size());

            #pragma omp parallel for
            for (int i = 0; i < m_grid_elements.size(); ++i)
            {
                m_irradiance_maps[i] = loadIrradianceMap(m_grid_elements[i]);
            }

            for (int i = 0; i < m_grid_elements.size(); ++i)
            {
                for (const auto& element : m_irradiance_maps[i])
                {
                    if (element.second > 0)
                    {
                        m_verification_elements.insert(element.first);
                    }
                }
            }

            if (m_parameters.computation_type != 7)
            {
                m_irradiance_maps.erase(m_irradiance_maps.begin(), m_irradiance_maps.end());
            }
        }
        else
        {
            for (int i = 0; i < m_grid_elements.size(); ++i)      
            {
                ExposureMap irradiance_map = loadIrradianceMap(m_grid_elements[i]);

                for (const auto& element : irradiance_map)
                {
                    if (element.second > 0)
                    {
                        m_verification_elements.insert(element.first);
                    }
                }
            }
        }  
    }



    void DoseCalculator::saveIrradianceMap(const OcTreeKey& plan_element_key, const ExposureMap& irradiance_map)
    {
        const std::string output_file = m_parameters.irradiance_maps_folder + 
            std::to_string(plan_element_key[0]) + "_" + std::to_string(plan_element_key[1]) + ".txt";
    
        std::ofstream file(output_file);

        if (file.is_open())
        {
            for (const auto& element : irradiance_map)
            {
                file << element.first[0] << ' ' << element.first[1] << ' ' << element.first[2] << ' ' << element.second << '\n';
            }

            file.close();
        }
        else
        {
            std::cerr << "Could not open output file for saving irradiance map!" << std::endl;
        }
    }



    ExposureMap DoseCalculator::loadIrradianceMap(const OcTreeKey& plan_element_key) const
    {
        ExposureMap irradiance_map;

        const std::string input_file = m_parameters.irradiance_maps_folder + 
            std::to_string(plan_element_key[0]) + "_" + std::to_string(plan_element_key[1]) + ".txt";

        std::ifstream file(input_file);

        if (file.is_open())
        {
            std::string line;

            while (std::getline(file, line))
            {
                std::vector<std::string> data;

                boost::split(data, line, boost::is_any_of(" "));

                OcTreeKey key(static_cast<uint16_t>(std::stoi(data[0])), 
                              static_cast<uint16_t>(std::stoi(data[1])), 
                              static_cast<uint16_t>(std::stoi(data[2])));

                irradiance_map[key] = std::stod(data[3]);
            }

            file.close();
        }
        else
        {
            std::cerr << "Could not open input file for loading irradiance map!" << std::endl;
        }

        return irradiance_map;
    }



    void DoseCalculator::setOptimizationElements()
    {
        if (m_parameters.computation_type == 1 || m_parameters.computation_type == 2)
        {
            create2DModel();
        }
        else if (m_parameters.computation_type == 3 || m_parameters.computation_type == 4)
        {
            computeLeastEfficientElements();
        }
        else if (m_parameters.computation_type == 5 || m_parameters.computation_type == 6)
        {
            for (AugmentedOcTree::leaf_iterator it = m_augmented_model->begin_leafs(), end = m_augmented_model->end_leafs(); it != end; ++it)
            {
                m_optimization_elements.insert(it.getKey());
            }
        }
        else if (m_parameters.computation_type == 7)
        {
            for (const auto& element : m_verification_elements)
            {
                m_optimization_elements.insert(element);
            }
        }

        std::cout << "Number of elements for optimization: " << m_optimization_elements.size() << std::endl;
        std::cout << "Number of elements for verification: " << m_verification_elements.size() << std::endl;
    }



    void DoseCalculator::create2DModel()
    {
        signal (SIGINT, handler);

        KeySet obstacle_2d;

        // Step 1: Map obstacles to the x-y plane
        for (AugmentedOcTree::leaf_iterator it = m_augmented_model->begin_leafs(), end = m_augmented_model->end_leafs(); it != end; ++it)
        {
            point3d point_3d = it.getCoordinate();
        
            point3d point_2d(point_3d.x(), point_3d.y(), m_parameters.ground_level - 1);

            if (OcTreeKey key; m_augmented_model->coordToKeyChecked(point_2d, m_parameters.depth, key))
            {
                m_optimization_elements.insert(key);

                if ((point_3d.z() - m_parameters.ground_level) > 0.0001)
                {
                    obstacle_2d.insert(key);
                }
            }
        }

        // Step 2: Insert 2D obstacles to the augmented 3D model
        for (const auto& key : obstacle_2d)
        {
            NodePtr node = m_augmented_model->search(key, m_parameters.depth);

            if (node == nullptr)
            {
                node = m_augmented_model->updateNode(key, true);
            }
        }

        if (m_parameters.computation_type == 2)
        {
            compute2DNormalVectors();
        }
    }



    void DoseCalculator::compute2DNormalVectors()
    {
        signal (SIGINT, handler);

        pcl::PointCloud<pcl::PointXYZ>::Ptr octree_points (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::Normal>::Ptr octree_normals (new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZ>());

        for (AugmentedOcTree::leaf_iterator it = m_augmented_model->begin_leafs(), end = m_augmented_model->end_leafs(); it != end; ++it)
        {
            const point3d point = it.getCoordinate();      

            if (std::abs(point.z() - (m_parameters.ground_level - 1)) < 0.0001 )
            {
                octree_points->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
            }
        }

        kdtree->setInputCloud(octree_points);

        normal_estimation.setInputCloud(octree_points);
        normal_estimation.setSearchMethod(kdtree);
        normal_estimation.setRadiusSearch(2 * m_parameters.resolution);

        normal_estimation.compute(*octree_normals);

        for (size_t i = 0; i < octree_points->size(); ++i)
        {
            const pcl::PointXYZ pcl_point = octree_points->points[i];

            const point3d point(pcl_point.data[0], pcl_point.data[1], pcl_point.data[2]);

            if (NodePtr node = m_augmented_model->search(point, m_parameters.depth); node != nullptr)
            {
                const pcl::Normal normal = octree_normals->at(i);

                point3d normal_2d = point3d(normal.normal_x, normal.normal_y, normal.normal_z);

                normal_2d.z() = 0;

                normal_2d.normalize();

                node->setNormal(normal_2d);
            }
        }
    }



    void DoseCalculator::computeLeastEfficientElements()
    {
        const double resolution = m_parameters.resolution;

        double x_min, y_min, z_min, x_max, y_max, z_max;

        m_augmented_model->getMetricMin(x_min, y_min, z_min);
        m_augmented_model->getMetricMax(x_max, y_max, z_max);

        double x = x_min + resolution / 2;
        double y = y_min + resolution / 2;

        const double center = m_parameters.lamp_center;

        while (x < x_max)
        {
            while (y < y_max)
            {
                point3d normal_sum = point3d(0, 0, 0);

                double z_closest, z_furthest;
                bool z_set = false;

                double z = m_parameters.ground_level;

                while(z < z_max)
                {
                    if (OcTreeKey key; m_augmented_model->coordToKeyChecked(point3d(x, y, z), m_parameters.depth, key))
                    {
                        point3d point = m_augmented_model->keyToCoord(key, m_parameters.depth);

                        x = point.x();
                        y = point.y();
                        z = point.z();
                    }

                    NodePtr node = m_augmented_model->search(point3d(x, y, z), m_parameters.depth);

                    if (node != nullptr && !std::isnan(node->getNormal().norm()) && !isHiddenElement(point3d(x, y, z)))
                    {
                        normal_sum += node->getNormal();

                        if (z_set)
                        {
                            if (std::abs(center - z) < std::abs(center - z_closest))
                            {
                                z_closest = z;
                            }

                            if (std::abs(center - z) > std::abs(center - z_furthest))
                            {
                                z_furthest = z;
                            }
                        }
                        else
                        {
                            z_closest = z;
                            z_furthest = z;

                            z_set = true;
                        }
                    }

                    z += resolution;
                }

                point3d least_efficient_point = (std::abs(normal_sum.normalized().z()) > sqrt(0.5)) ? point3d(x, y, z_closest) : point3d(x, y, z_furthest);

                if (OcTreeKey least_efficient_key; m_augmented_model->coordToKeyChecked(least_efficient_point, m_parameters.depth, least_efficient_key))
                {
                    m_optimization_elements.insert(least_efficient_key);
                }

                y += resolution;

                z = 0;
            }
        
            x += resolution;

            y = 0;
        }
    }



    bool DoseCalculator::isHiddenElement(const point3d& point) const
    {
        const float resolution = static_cast<float>(m_parameters.resolution);

        const std::vector<point3d> steps {
            { -resolution, 0, 0 }, { resolution, 0, 0 }, 
            { 0, -resolution, 0 }, { 0, resolution, 0 },
            { 0, 0, -resolution }, { 0, 0, resolution }
        };

        if (std::abs(point.z() - m_parameters.ground_level) < 0.0001)
        {
            NodePtr node = m_augmented_model->search(point + point3d(0, 0, m_parameters.ground_level), m_parameters.depth);

            if (node != nullptr)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
        else
        {
            for (const auto& step : steps)
            {
                NodePtr node = m_augmented_model->search(point + step, m_parameters.depth);

                if (node == nullptr)
                {
                    return false;
                }
            }

            return true;
        }
    }



    void DoseCalculator::compute(std::vector<RadiationPlan>& radiation_plans, IndexVector& index_vector)
    {
        signal (SIGINT, handler);

        m_parameters.number_of_positions = radiation_plans[0].first.size() / m_parameters.plan_element_size;

        #pragma omp parallel for
        for (int i = 0; i < index_vector.size(); ++i)
        {
            radiation_plans[index_vector[i]].second = computeCoverageForPlan(radiation_plans[index_vector[i]]);
        }

        index_vector.erase(index_vector.begin(), index_vector.end());
    }



    double DoseCalculator::computeCoverageForPlan(RadiationPlan& radiation_plan)
    {
        signal (SIGINT, handler);

        const int element_size = m_parameters.plan_element_size;

        std::vector<double> elements = radiation_plan.first;

        std::vector<ExposureMap> exposure_maps(m_parameters.number_of_positions);

        for (int i = 0; i < elements.size(); i += element_size)
        {
            point3d lamp_position(elements[i], elements[i + 1], m_parameters.ground_level);

            OcTreeKey key; 

            m_augmented_model->coordToKeyChecked(lamp_position, m_parameters.depth, key);

            if (m_ground_zone_elements.find(key) != m_ground_zone_elements.end())
            {
                lamp_position.z() = m_parameters.lamp_center;

                PlanElement plan_element { lamp_position, elements[i + 2] };

                exposure_maps[i / element_size] = computeDoseForPlanElement(plan_element, false);
            }
            else
            {  
                for (AugmentedOcTree::leaf_iterator it = m_augmented_model->begin_leafs(), end = m_augmented_model->end_leafs(); it != end; ++it)
                {
                    exposure_maps[i / element_size][it.getKey()] = 0;
                }
            }
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

            if (element.second >= m_parameters.exposure_limit)
            {
                general_over += 1;
            }

            NodePtr node = m_augmented_model->search(element.first, m_parameters.depth);

            if (node != nullptr && node->getType() == 5)
            {
                if (m_optimization_elements.find(element.first) != m_optimization_elements.end())
                {
                    object_sum += 1;
                    
                    if (element.second >= m_parameters.exposure_limit)
                    {
                        object_over += 1;
                    }
                }
            }
        }

        double fitness;

        if (m_parameters.semantic)
        {
            fitness = static_cast<double>(object_over)  / static_cast<double>(object_sum);
        }
        else
        {
            fitness = static_cast<double>(general_over) / static_cast<double>(m_optimization_elements.size());
        }

        return fitness;
    }



    ExposureMap DoseCalculator::computeDoseForPlanElement(const PlanElement& plan_element, bool verify)
    {
        signal (SIGINT, handler);

        ExposureMap dose_map;

        if (verify || m_parameters.computation_type != 7)
        {
            dose_map = computeIrradianceForPosition(plan_element.first, verify);
        }   
        else
        {
            double min_distance = std::numeric_limits<double>::max();
            
            int grid_index;

            for (int i = 0; i < m_grid_elements.size(); ++i)
            {
                point3d grid_position = m_augmented_model->keyToCoord(m_grid_elements[i], m_parameters.depth);

                grid_position.z() = m_parameters.lamp_center; 

                const double distance = (grid_position - plan_element.first).norm();

                if (distance < min_distance)
                {
                    min_distance = distance;
                    grid_index = i;
                }
            }

            if (m_parameters.store_maps)
            {
                dose_map = m_irradiance_maps[grid_index];
            }
            else
            {
                dose_map = loadIrradianceMap(m_grid_elements[grid_index]);
            }
            
        }

        for (auto& element : dose_map)
        {
            element.second *= plan_element.second;
        }

        return dose_map;
    }



    ExposureMap DoseCalculator::computeIrradianceForPosition(const point3d& lamp_position, bool verify)
    {
        signal (SIGINT, handler);

        ExposureMap irradiance_map;

        if (verify)
        {
            for (const auto& element : m_verification_elements)
            {
                irradiance_map[element] = computeIrradianceForElement(lamp_position, element);
            }
        }
        else
        {
            if (m_parameters.computation_type == 7)
            {
                for (AugmentedOcTree::leaf_iterator it = m_augmented_model->begin_leafs(), end = m_augmented_model->end_leafs(); it != end; ++it)
                {
                    irradiance_map[it.getKey()] = computeIrradianceForElement(lamp_position, it.getKey());
                }
            }
            else
            {
                for (const auto& element : m_optimization_elements)
                {
                    irradiance_map[element] = computeIrradianceForElement(lamp_position, element);
                }
            }
        }

        return irradiance_map;
    }



    double DoseCalculator::computeIrradianceForElement(const point3d& lamp_position, const OcTreeKey& key) const
    {
        signal (SIGINT, handler);

        double irradiance = 0;

        switch (m_parameters.computation_type)
        {
        case 1:
            irradiance = computeIrradianceType1(lamp_position, key);
            break;
        case 2:
            irradiance = computeIrradianceType2(lamp_position, key);
            break;
        case 3:
            irradiance = computeIrradianceType3(lamp_position, key);
            break;
        case 4:
            irradiance = computeIrradianceType4(lamp_position, key);
            break;
        case 5:
            irradiance = computeIrradianceType3(lamp_position, key);
            break;
        case 6:
            irradiance = computeIrradianceType3(lamp_position, key);
            break;
        case 7:
            irradiance = computeIrradianceType4(lamp_position, key);
            break;
        default:
            break;
        }
        
        return irradiance;
    }



    // 1 - Pierson - 2D model, 2D distance, no angle, point light source
    double DoseCalculator::computeIrradianceType1(const point3d& lamp_position, const OcTreeKey& key) const
    {
        double irradiance = 0;

        const point3d point = m_augmented_model->keyToCoord(key, m_parameters.depth);

        const point3d lamp_position_2d(lamp_position.x(), lamp_position.y(), m_parameters.ground_level - 1);

        const double distance = (point - lamp_position_2d).norm();

        if (!std::isnan(distance) && distance < m_parameters.lamp_range && distance > m_parameters.safety_radius)
        {
            if(compute2DVisibility(lamp_position_2d, point))
            {
                irradiance = m_parameters.lamp_power / (4 * M_PI * std::pow(distance, 2));
            }
        }

        return irradiance;
    }



    // 2 - Conroy - 2D model, d = d_2D + h/2, 2D angle (lenghtened ray), point light source 
    double DoseCalculator::computeIrradianceType2(const point3d& lamp_position, const OcTreeKey& key) const
    {
        double irradiance = 0;

        const point3d point = m_augmented_model->keyToCoord(key, m_parameters.depth);

        const point3d lamp_position_2d(lamp_position.x(), lamp_position.y(), m_parameters.ground_level - 1);

        const double d2 = (point - lamp_position_2d).norm();

        // Modification: instead of h/2 we use lamp_offset + lamp_height / 2
        const double distance = std::sqrt(std::pow(d2, 2) + 
            std::pow(m_parameters.lamp_offset + m_parameters.lamp_height / 2, 2));

        if (!std::isnan(distance) && distance < m_parameters.lamp_range && d2 > m_parameters.safety_radius)
        {
            NodePtr node = m_augmented_model->search(key, m_parameters.depth);

            if (node != nullptr) // 2D element with known normal vector
            {
                const point3d normal_2d = node->getNormal();
                
                if (!std::isnan(normal_2d.norm()))
                {
                    if(compute2DVisibility(lamp_position_2d, point))
                    {
                        const point3d difference_2d = point - lamp_position_2d;

                        irradiance = m_parameters.lamp_power * std::abs(std::cos(normal_2d.angleTo(difference_2d))) / (4 * M_PI * std::pow(distance, 2));
                    }
                }   
            }
            else // 2D element where the ray needs to be lengthened
            {
                if(compute2DVisibility(lamp_position_2d, point))
                {
                    const point3d difference_2d = point - lamp_position_2d;

                    point3d end_point;

                    if (m_augmented_model->castRay(point, difference_2d, lamp_position_2d, end_point, true, -1, m_parameters.depth, m_parameters.resolution))
                    {
                        NodePtr end_node = m_augmented_model->search(end_point, m_parameters.depth);

                        if (end_node != nullptr)
                        {
                            const point3d normal_2d = end_node->getNormal();

                            if (!std::isnan(normal_2d.norm()))
                            {
                                const point3d difference_end = end_point - lamp_position_2d;

                                irradiance = m_parameters.lamp_power * std::abs(std::cos(normal_2d.angleTo(difference_end))) / (4 * M_PI * std::pow(distance, 2));
                            }
                        }
                    }    
                }
            }
        }

        return irradiance;
    }



    // 3 - Tiseni (point), Kurniawan - 3D model, 3D distance, 3D or no angle, point light source
    double DoseCalculator::computeIrradianceType3(const point3d& lamp_position, const OcTreeKey& key) const
    {
        double irradiance = 0;

        const point3d point = m_augmented_model->keyToCoord(key, m_parameters.depth);

        static double base_coverage = (((m_parameters.lamp_height + m_parameters.lamp_offset) / m_parameters.lamp_height)) * m_parameters.safety_radius;

        NodePtr node = m_augmented_model->search(key, m_parameters.depth);

        if (node->getType() == 2)
        {
            const point3d base_point = point3d(lamp_position.x(), lamp_position.y(), m_parameters.ground_level);

            if (std::abs(base_point.x() - point.x()) < base_coverage && std::abs(base_point.y() - point.y()) < base_coverage)
            {
                return 0;
            }
        }

        const double distance = (point - lamp_position).norm();

        if (!std::isnan(distance) && distance < m_parameters.lamp_range)
        {
            if (m_parameters.computation_type == 5)
            {
                if (compute3DVisibility(lamp_position, point))
                {
                    irradiance = m_parameters.lamp_power / (4 * M_PI * std::pow(distance, 2));
                }
            }
            else
            {
                const point3d normal = node->getNormal();

                if (!std::isnan(normal.norm()))
                {
                    if (compute3DVisibility(lamp_position, point))
                    {
                        const point3d difference = point - lamp_position;

                        irradiance = m_parameters.lamp_power * std::abs(std::cos(normal.angleTo(difference))) / (4 * M_PI * std::pow(distance, 2));     
                    }
                }
            }
        }

        return irradiance;
    }



    // 4 - Tiseni (cylindrical), Mine - 3D model, 3D distance, 3D angle, cylindrical light source
    double DoseCalculator::computeIrradianceType4(const point3d& lamp_position, const OcTreeKey& key) const
    {
        double irradiance = 0;

        const point3d point = m_augmented_model->keyToCoord(key, m_parameters.depth);

        static double base_coverage = (((m_parameters.lamp_height + m_parameters.lamp_offset) / m_parameters.lamp_height)) * m_parameters.safety_radius;

        NodePtr node = m_augmented_model->search(key, m_parameters.depth);

        if (node->getType() == 2)
        {
            const point3d base_point = point3d(lamp_position.x(), lamp_position.y(), m_parameters.ground_level);

            if (std::abs(base_point.x() - point.x()) < base_coverage && std::abs(base_point.y() - point.y()) < base_coverage)
            {
                return 0;
            }
        }

        const double distance = (point - lamp_position).norm();

        if (!std::isnan(distance) && distance < m_parameters.lamp_range)
        {
            const point3d normal = node->getNormal();

            if (!std::isnan(normal.norm()))
            {
                const double L = m_parameters.resolution;

                const double coefficient = m_parameters.lamp_power / (4 * M_PI * m_parameters.lamp_height);

                point3d center = lamp_position;

                for (int i = 1; i < m_ray_targets.size() - 1; ++i)
                {
                    center.z() = m_ray_targets[i];

                    if (compute3DVisibility(center, point))
                    {
                        const point3d difference = point - center;

                        const double integral = computeIrradianceIntegral(difference, normal, L);

                        irradiance += coefficient * integral;
                    }
                }
            }
        }

        return irradiance;
    }



    double DoseCalculator::computeIrradianceIntegral(const point3d& difference, const point3d& normal, double L) const
    {
        const double F_lower = (normal.z() * (pow(difference.x(), 2) + pow(difference.y(), 2)) +
            (-L/2 - difference.z()) * (normal.x() * difference.x() + normal.y() * difference.y())) /
            ((pow(difference.x(), 2) + pow(difference.y(), 2)) * pow(pow(-L/2 - difference.z(), 2) + 
            pow(difference.x(), 2) + pow(difference.y(), 2), 0.5));

        const double F_upper = (normal.z() * (pow(difference.x(), 2) + pow(difference.y(), 2)) +
            (L/2 - difference.z()) * (normal.x() * difference.x() + normal.y() * difference.y())) /
            ((pow(difference.x(), 2) + pow(difference.y(), 2)) * pow(pow(L/2 - difference.z(), 2) + 
            pow(difference.x(), 2) + pow(difference.y(), 2), 0.5));

        return std::abs(F_upper - F_lower);
    }



    bool DoseCalculator::compute2DVisibility(const point3d& lamp_position, const point3d& element) const
    {
        signal (SIGINT, handler);

        const double resolution = m_parameters.resolution + 0.001;

        const int depth = m_parameters.depth;

        point3d target(lamp_position.x(), lamp_position.y(), m_parameters.ground_level - 1);

        if (OcTreeKey key; m_augmented_model->coordToKeyChecked(target, depth, key))
        {
            target = m_augmented_model->keyToCoord(key, depth);
        }

        point3d point(element.x(), element.y(), m_parameters.ground_level - 1);

        const point3d direction = target - point;

        const double distance = 1.1 * direction.norm();

        double offset = (direction.x() >= 0) ? resolution : -resolution;

        point3d origin = point + point3d(offset, 0, 0);

        point3d target_direction = target - origin;

        point3d end_point;

        if (m_augmented_model->checkRayCast(false, target, origin, target_direction, end_point, distance, depth, resolution, true))
        {
            offset = (direction.y() >= 0) ? resolution : -resolution;

            origin = point + point3d(0, offset, 0);

            target_direction = target - origin;

            if (m_augmented_model->checkRayCast(true, target, origin, target_direction, end_point, distance, depth, resolution, true))
            {
                return true;
            }
        }
        else
        {
            return true;
        }

        return false;
    }



    bool DoseCalculator::compute3DVisibility(const point3d& lamp_position, const point3d& element) const
    {
        signal (SIGINT, handler);

        const double resolution = m_parameters.resolution + 0.001;

        const int depth = m_parameters.depth;

        point3d target;

        if (OcTreeKey key; m_augmented_model->coordToKeyChecked(lamp_position, depth, key))
        {
            target = m_augmented_model->keyToCoord(key, depth);
        }

        const point3d direction = target - element;

        const double distance = 1.1 * direction.norm();

        double offset = (direction.x() >= 0) ? resolution : -resolution;

        point3d origin = element + point3d(offset, 0, 0);

        point3d target_direction = target - origin;

        point3d end_point;

        if (m_augmented_model->checkRayCast(false, target, origin, target_direction, end_point, distance, depth, resolution, true))
        {
            offset = (direction.y() >= 0) ? resolution : -resolution;

            origin = element + point3d(0, offset, 0);

            target_direction = target - origin;

            if (m_augmented_model->checkRayCast(false, target, origin, target_direction, end_point, distance, depth, resolution, true))
            {
                offset = (direction.z() >= 0) ? resolution : -resolution;

                origin = element + point3d(0, 0, offset);
                
                target_direction = target - origin;

                if (m_augmented_model->checkRayCast(true, target, origin, target_direction, end_point, distance, depth, resolution, true))
                {
                    return true;
                }
            }
            else
            {
                return true;
            }
        } 
        else
        {
            return true;
        }          

        return false;
    }
}


















