#include "dose_calculator.h"

#include <boost/algorithm/string.hpp>
#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <signal.h>

namespace rpo
{
    void handler(int s)
    {
        exit(1);
    }



    double estimateMemoryUsage(const std::vector<BreakPoints>& data)
    {
        size_t total_size = sizeof(data);

        for (const auto& umap : data)
        {
            total_size += sizeof(umap);

            total_size += umap.bucket_count() * sizeof(void*);

            for (const auto& kv : umap)
            {
                const auto& key = kv.first;
                const auto& value_vector = kv.second;

                total_size += sizeof(key);

                total_size += sizeof(value_vector);

                total_size += value_vector.capacity() * sizeof(octomap::OcTreeKey);

                total_size += sizeof(void*) * 2;
            }

        }
    
        total_size += data.capacity() * sizeof(BreakPoints);

        return double(total_size) / double(pow(1024, 3));
    }



    DoseCalculator::DoseCalculator(const std::shared_ptr<AugmentedOcTree> augmented_model, const Parameters& parameters)
    {
        m_augmented_model = augmented_model;

        m_parameters = parameters;

        m_depth = m_parameters.preprocessing.depth;
        m_resolution = m_parameters.preprocessing.resolution;
        m_ground_level = m_parameters.preprocessing.ground_level;
    }



    Parameters DoseCalculator::getParameters() const
    {
        return m_parameters;
    }



    void DoseCalculator::computeGroundZone()
    {
        KeySet ground_zone;

        bool ground_level_set = false;

        for (rpo::AugmentedOcTree::leaf_iterator it = m_augmented_model->begin_leafs(), end = m_augmented_model->end_leafs(); it != end; ++it)
        {
            NodePtr node = m_augmented_model->search(it.getKey(), m_depth);

            if (node != nullptr && node->getType() == 2)
            {
                point3d point = it.getCoordinate();

                if (!ground_level_set)
                {
                    m_ground_level = point.z();

                    m_parameters.lamp.center = m_ground_level + m_resolution / 2 + m_parameters.lamp.offset + m_parameters.lamp.height / 2;

                    m_parameters.lamp.top = m_parameters.lamp.center + m_parameters.lamp.height / 2;

                    ground_level_set = true;
                }

                if (ground_level_set)
                {
                    bool ground = true;

                    point.z() += m_resolution;

                    while (point.z() < m_parameters.lamp.top)
                    {
                        NodePtr upper_node = m_augmented_model->search(point, m_depth);

                        if (upper_node == nullptr)
                        {
                            point.z() += m_resolution;
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

        KeySet ground_zone_limited;

        for (const auto& key : ground_zone)
        {
            const point3d center = m_augmented_model->keyToCoord(key, m_depth);

            bool ground = true;

            // For each possible ground zone element it checked whether its
            // neighbors are also ground zone elements. If all neighbors in
            // a given region also belong to the ground zone, then the center
            // is considered as real ground zone element.
            int min_dist = static_cast<int>(m_parameters.preprocessing.safety_radius / m_resolution) + 1;

            for (int i = -min_dist; i <= min_dist; ++i)
            {
                for (int j = -min_dist; j <= min_dist; ++j)
                {
                    const point3d neighbor = center + point3d(i * m_resolution, j * m_resolution, 0);

                    if (OcTreeKey neighbor_key; m_augmented_model->coordToKeyChecked(neighbor, m_depth, neighbor_key))
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
                if (NodePtr node = m_augmented_model->search(key, m_depth); node != nullptr)
                {
                    ground_zone_limited.insert(key);
                }
            }
        }


        
        /// Selecting largest ground zone
        KeySet visited;

        int region_count = 0;
        int max_region_size = 0;

        std::vector<octomap::OcTreeKey> largest_region;

        const std::vector<std::pair<int, int>> neighbors = { {1, 0}, {-1, 0}, {0, 1}, {0, -1} };

        for (const auto& key : ground_zone_limited)
        {
            if (visited.find(key) == visited.end())
            {
                ++region_count;

                std::stack<octomap::OcTreeKey> stack;

                stack.push(key);

                std::vector<octomap::OcTreeKey> current_region;

                int current_region_size = 0;

                while (!stack.empty())
                {
                    auto current = stack.top();

                    stack.pop();

                    if (visited.find(current) != visited.end())
                    {
                        continue;
                    }

                    visited.insert(current);

                    current_region.push_back(current);
                    
                    ++current_region_size;

                    // Explore 4-connected neighbors
                    for (const auto& offset : neighbors)
                    {
                        octomap::OcTreeKey neighbor(current[0] + offset.first, current[1] + offset.second, current[2]);

                        // If neighbor is in the set and not visited, add it to the stack
                        if (ground_zone_limited.find(neighbor) != ground_zone_limited.end() && visited.find(neighbor) == visited.end()) 
                        {
                            stack.push(neighbor);
                        }
                    }
                }

                // Check if the current region is the largest one
                if (current_region_size > max_region_size) 
                {
                    max_region_size = current_region_size;
                    largest_region = current_region;
                }
            }
        }


        for (int i = 0; i < largest_region.size(); ++i)
        {
            m_ground_zone_elements.insert(largest_region[i]);
        }


        std::cout << "Ground zone elements: " << m_ground_zone_elements.size() << std::endl;
    }   


    
    void DoseCalculator::computeGridElements()
    {
        const double grid_distance = m_parameters.computation.grid_distance * m_resolution;

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
        double height = m_ground_level + m_resolution / 2.0 + m_parameters.lamp.offset;

        point3d continuous_height(0, 0, height);

        if (OcTreeKey key; m_augmented_model->coordToKeyChecked(continuous_height, m_depth, key))
        {
            point3d discrete_height = m_augmented_model->keyToCoord(key, m_depth);

            height = discrete_height.z();

            while (height < m_parameters.lamp.top)
            {
                m_ray_targets.push_back(height);

                height += m_resolution;
            }
        }

        std::cout << "Lamp elements: " << m_ray_targets.size() << std::endl;

        std::cout << m_augmented_model->getNumLeafNodes() << std::endl;
    }



    std::multimap<double, double> DoseCalculator::getGroundZone() const
    {
        std::multimap<double, double> ground_zone;

        for (const auto& key : m_ground_zone_elements)
        {
            const point3d& point = m_augmented_model->keyToCoord(key, m_depth);

            ground_zone.insert({ point.x(), point.y() });
        }

        return ground_zone;
    }



    RadiationPlan DoseCalculator::getGrid(const RadiationPlan& plan) const
    {
        const int element_size = m_parameters.optimization.element_size;

        RadiationPlan grid_plan = plan;

        for (int i = 0; i < grid_plan.first.size(); i += element_size)
        {
            double min_distance = std::numeric_limits<double>::max();

            OcTreeKey grid_key;
                
            for (const auto& element : m_grid_elements)
            {
                point3d grid_position = m_augmented_model->keyToCoord(element, m_depth);

                point3d lamp_position(grid_plan.first[i], grid_plan.first[i + 1], m_ground_level);

                const double distance = (grid_position - lamp_position).norm();
                
                if (distance < min_distance)
                {
                    min_distance = distance;
                    grid_key = element;
                }
            }

            point3d closest_point = m_augmented_model->keyToCoord(grid_key, m_depth);

            grid_plan.first[i] = closest_point.x();
            grid_plan.first[i + 1] = closest_point.y();
        }

        return grid_plan;
    }



    std::vector<OcTreeKey> DoseCalculator::getGridElements() const
    {
        return m_grid_elements;
    }



    void DoseCalculator::computeIrradianceMaps()
    {
        if (m_parameters.computation.filter)
        {
            computeGeneralVisibility();
        }

        std::cout << "Computing irradiance maps" << std::endl;

        if (m_parameters.computation.type == 8)
        {
            create2DModel();

            if (!m_parameters.computation.filter)
            {
                for (AugmentedOcTree::leaf_iterator it = m_augmented_model->begin_leafs(), end = m_augmented_model->end_leafs(); it != end; ++it)
                {
                    m_base_reachable_elements.insert(it.getKey());
                }
            }

            computeBreakPoints();
        }

        if (m_parameters.computation.store_maps)
        {
            m_irradiance_maps.resize(m_grid_elements.size());
        }
        
        #pragma omp parallel for
        for (int i = 0; i < m_grid_elements.size(); ++i)
        {
            point3d position = m_augmented_model->keyToCoord(m_grid_elements[i], m_depth);

            position.z() = m_parameters.lamp.center;

            ExposureMap irradiance_map;

            if (m_parameters.computation.store_maps)
            {
                m_irradiance_maps[i] = computeIrradianceForPosition(position, false, i);
            }
            else
            {
                irradiance_map = computeIrradianceForPosition(position, false, i);
            }
            

            if (m_parameters.computation.save_maps)
            {
                OcTreeKey plan_element_key;

                m_augmented_model->coordToKeyChecked(position, m_depth, plan_element_key);

                if (m_parameters.computation.store_maps)
                {
                    saveBinaryMap(plan_element_key, m_irradiance_maps[i]);
                }
                else
                {
                    saveBinaryMap(plan_element_key, irradiance_map);
                }
            }
        }


        if (m_parameters.computation.store_maps)
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
                ExposureMap irradiance_map = loadBinaryMap(m_grid_elements[i]);

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
        if (m_parameters.computation.store_maps)
        {
            m_irradiance_maps.resize(m_grid_elements.size());

            #pragma omp parallel for
            for (int i = 0; i < m_grid_elements.size(); ++i)
            {
                m_irradiance_maps[i] = loadBinaryMap(m_grid_elements[i]);
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

            if (m_parameters.computation.type != 7 && m_parameters.computation.type != 8)
            {
                m_irradiance_maps.erase(m_irradiance_maps.begin(), m_irradiance_maps.end());
            }
        }
        else
        {
            for (int i = 0; i < m_grid_elements.size(); ++i)      
            {
                ExposureMap irradiance_map = loadBinaryMap(m_grid_elements[i]);

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
        const std::string output_file = m_parameters.paths.irradiance_maps + 
            std::to_string(plan_element_key[0]) + "_" + std::to_string(plan_element_key[1]) + ".txt";
    
        std::ofstream file(output_file);

        if (file.is_open())
        {
            for (const auto& element : irradiance_map)
            {
                if (element.second > 0)
                {
                    file << element.first[0] << ' ' << element.first[1] << ' ' << element.first[2] << ' ' << element.second << '\n';
                }
            }

            file.close();
        }
        else
        {
            std::cerr << "Could not open output file for saving irradiance map!" << std::endl;
        }
    }



    void DoseCalculator::saveBinaryMap(const OcTreeKey& plan_element_key, const ExposureMap& irradiance_map)
    {
        const std::string output_file = m_parameters.paths.irradiance_maps + 
            std::to_string(plan_element_key[0]) + "_" + std::to_string(plan_element_key[1]) + ".bin";

        std::ofstream file(output_file, std::ios::binary);

        if (file.is_open())
        {
            for (const auto& element : irradiance_map)
            {
                if (element.second > 0)
                {
                    file.write(reinterpret_cast<const char*>(&element.first[0]), sizeof(element.first[0]));
                    file.write(reinterpret_cast<const char*>(&element.first[1]), sizeof(element.first[1]));
                    file.write(reinterpret_cast<const char*>(&element.first[2]), sizeof(element.first[2]));
                    file.write(reinterpret_cast<const char*>(&element.second),   sizeof(element.second));
                }
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

        const std::string input_file = m_parameters.paths.irradiance_maps + 
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



    ExposureMap DoseCalculator::loadBinaryMap(const OcTreeKey& plan_element_key) const
    {
        ExposureMap irradiance_map;

        const std::string input_file = m_parameters.paths.irradiance_maps + 
            std::to_string(plan_element_key[0]) + "_" + std::to_string(plan_element_key[1]) + ".bin";

        std::ifstream file(input_file, std::ios::binary);
    
        if (file.is_open())
        {
            OcTreeKey key;
            float value;

            while (file.read(reinterpret_cast<char*>(&key[0]), sizeof(key[0])))
            {
                file.read(reinterpret_cast<char*>(&key[1]), sizeof(key[1]));
                file.read(reinterpret_cast<char*>(&key[2]), sizeof(key[2]));
                file.read(reinterpret_cast<char*>(&value), sizeof(value));

                irradiance_map[key] = value;
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
        int type = m_parameters.computation.type;

        if (type == 1 || type == 2)
        {
            create2DModel();
        }
        else if (type == 3 || type == 4)
        {
            computeLeastEfficientElements();
        }
        else if (type == 5 || type == 6 || type == 9)
        {
            for (AugmentedOcTree::leaf_iterator it = m_augmented_model->begin_leafs(), end = m_augmented_model->end_leafs(); it != end; ++it)
            {
                m_optimization_elements.insert(it.getKey());
                
            }
        }
        else if (type == 10)
        {
            computeGeneralVisibility();

            for (const auto& element : m_base_reachable_elements)
            {
                m_optimization_elements.insert(element);
            }
        }
        else if (type == 7 || type == 8)
        {
            for (const auto& element : m_verification_elements)
            {
                m_optimization_elements.insert(element);

                // TODO: extension for object computation
                AugmentedOcTreeNode* augmented_node = m_augmented_model->search(element);

                if (augmented_node->getType() == 5)
                {
                    m_object_elements.insert(element);
                }

            }
        }

        std::cout << "Number of elements for optimization: " << m_optimization_elements.size() << std::endl;
        std::cout << "Number of elements for verification: " << m_verification_elements.size() << std::endl;
    }



    void DoseCalculator::create2DModel()
    {
        signal (SIGINT, handler);

        std::cout << "Original size: " << m_augmented_model->getNumLeafNodes() << std::endl;

        // Step 1: Map obstacles to the x-y plane
        for (AugmentedOcTree::leaf_iterator it = m_augmented_model->begin_leafs(), end = m_augmented_model->end_leafs(); it != end; ++it)
        {
            point3d point_3d = it.getCoordinate();
        
            point3d point_2d(point_3d.x(), point_3d.y(), m_ground_level - 1);

            if (OcTreeKey key; m_augmented_model->coordToKeyChecked(point_2d, m_depth, key))
            {
                if (m_parameters.computation.type != 8)
                {
                    m_optimization_elements.insert(key);
                }

                if ((point_3d.z() - m_ground_level) > 0.0001)
                {
                    m_obstacle_2d.insert(key);
                }
            }
        }

        // Step 2: Insert 2D obstacles to the augmented 3D model
        for (const auto& key : m_obstacle_2d)
        {
            NodePtr node = m_augmented_model->search(key, m_depth);

            if (node == nullptr)
            {
                node = m_augmented_model->updateNode(key, true);
            }
        }

        std::cout << "Extended size: " << m_augmented_model->getNumLeafNodes() << std::endl;


        if (m_parameters.computation.type == 2)
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

            if (std::abs(point.z() - (m_ground_level - 1)) < 0.0001 )
            {
                octree_points->push_back(pcl::PointXYZ(point.x(), point.y(), point.z()));
            }
        }

        kdtree->setInputCloud(octree_points);

        normal_estimation.setInputCloud(octree_points);
        normal_estimation.setSearchMethod(kdtree);
        normal_estimation.setRadiusSearch(2 * m_resolution);

        normal_estimation.compute(*octree_normals);

        for (size_t i = 0; i < octree_points->size(); ++i)
        {
            const pcl::PointXYZ pcl_point = octree_points->points[i];

            const point3d point(pcl_point.data[0], pcl_point.data[1], pcl_point.data[2]);

            if (NodePtr node = m_augmented_model->search(point, m_depth); node != nullptr)
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
        double x_min, y_min, z_min, x_max, y_max, z_max;

        m_augmented_model->getMetricMin(x_min, y_min, z_min);
        m_augmented_model->getMetricMax(x_max, y_max, z_max);

        double x = x_min + m_resolution / 2;
        double y = y_min + m_resolution / 2;

        const double center = m_parameters.lamp.center;

        while (x < x_max)
        {
            while (y < y_max)
            {
                point3d normal_sum = point3d(0, 0, 0);

                double z_closest, z_furthest;
                bool z_set = false;

                double z = m_ground_level;

                while(z < z_max)
                {
                    if (OcTreeKey key; m_augmented_model->coordToKeyChecked(point3d(x, y, z), m_depth, key))
                    {
                        point3d point = m_augmented_model->keyToCoord(key, m_depth);

                        x = point.x();
                        y = point.y();
                        z = point.z();
                    }

                    NodePtr node = m_augmented_model->search(point3d(x, y, z), m_depth);

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

                    z += m_resolution;
                }

                point3d least_efficient_point = (std::abs(normal_sum.normalized().z()) > sqrt(0.5)) ? point3d(x, y, z_closest) : point3d(x, y, z_furthest);

                if (NodePtr lep_node = m_augmented_model->search(least_efficient_point, m_depth); lep_node != nullptr)
                {
                    if (OcTreeKey least_efficient_key; m_augmented_model->coordToKeyChecked(least_efficient_point, m_depth, least_efficient_key))
                    {
                        m_optimization_elements.insert(least_efficient_key);
                    }
                }

                y += m_resolution;

                z = m_ground_level;
            }
        
            x += m_resolution;

            y = y_min + m_resolution / 2;
        }
    }



    bool DoseCalculator::isHiddenElement(const point3d& point) const
    {
        const float resolution = static_cast<float>(m_resolution);

        const std::vector<point3d> steps {
            { -resolution, 0, 0 }, { resolution, 0, 0 }, 
            { 0, -resolution, 0 }, { 0, resolution, 0 },
            { 0, 0, -resolution }, { 0, 0, resolution }
        };

        if (std::abs(point.z() - m_ground_level) < 0.0001)
        {
            NodePtr node = m_augmented_model->search(point + point3d(0, 0, m_ground_level), m_depth);

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
                NodePtr node = m_augmented_model->search(point + step, m_depth);

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

        m_parameters.optimization.num_positions = radiation_plans[0].first.size() / m_parameters.optimization.element_size;

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

        const int element_size = m_parameters.optimization.element_size;

        std::vector<double> elements = radiation_plan.first;

        std::vector<ExposureMap> exposure_maps(m_parameters.optimization.num_positions);

        for (int i = 0; i < elements.size(); i += element_size)
        {
            point3d lamp_position(elements[i], elements[i + 1], m_ground_level);

            OcTreeKey key; 

            m_augmented_model->coordToKeyChecked(lamp_position, m_depth, key);

            if (m_ground_zone_elements.find(key) != m_ground_zone_elements.end())
            {
                lamp_position.z() = m_parameters.lamp.center;

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

        int general_over = 0, object_over = 0, object_sum = m_object_elements.size();

        rpo::ExposureMap exposure_map;

        for (auto& element : m_optimization_elements)
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
                if (m_optimization_elements.find(element) != m_optimization_elements.end())
                {
                    if (exposure_map[element] >= m_parameters.computation.exposure_limit)
                    {
                        object_over += 1;
                    }
                }
            }
        }

        double fitness;

        if (m_parameters.computation.semantic)
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

        if (verify || (m_parameters.computation.type != 7 && m_parameters.computation.type != 8))
        {
            dose_map = computeIrradianceForPosition(plan_element.first, verify);
        }   
        else
        {
            double min_distance = std::numeric_limits<double>::max();
            
            int grid_index;

            for (int i = 0; i < m_grid_elements.size(); ++i)
            {
                point3d grid_position = m_augmented_model->keyToCoord(m_grid_elements[i], m_depth);

                grid_position.z() = m_parameters.lamp.center; 

                const double distance = (grid_position - plan_element.first).norm();

                if (distance < min_distance)
                {
                    min_distance = distance;
                    grid_index = i;
                }
            }

            if (m_parameters.computation.store_maps)
            {
                dose_map = m_irradiance_maps[grid_index];
            }
            else
            {
                dose_map = loadBinaryMap(m_grid_elements[grid_index]);
            }
            
        }

        for (auto& element : dose_map)
        {
            element.second *= plan_element.second;
        }

        return dose_map;
    }



    ExposureMap DoseCalculator::computeIrradianceForPosition(const point3d& lamp_position, bool verify, int index)
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
            if (m_parameters.computation.type == 7 || m_parameters.computation.type == 8)
            {
                if (m_parameters.computation.filter)
                {
                    for (const auto& element : m_base_reachable_elements)
                    {
                        irradiance_map[element] = computeIrradianceForElement(lamp_position, element, index);
                    } 
                }
                else
                {
                    for (AugmentedOcTree::leaf_iterator it = m_augmented_model->begin_leafs(), end = m_augmented_model->end_leafs(); it != end; ++it)
                    {
                        irradiance_map[it.getKey()] = computeIrradianceForElement(lamp_position, it.getKey(), index);
                    } 
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



    double DoseCalculator::computeIrradianceForElement(const point3d& lamp_position, const OcTreeKey& key, int index)
    {
        signal (SIGINT, handler);

        double irradiance = 0;

        switch (m_parameters.computation.type)
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
        case 8:
            irradiance = computeIrradianceType4(lamp_position, key, index);
            break;
        case 9:
            irradiance = computeIrradianceType4(lamp_position, key);
            break;
        case 10:
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

        const point3d point = m_augmented_model->keyToCoord(key, m_depth);

        const point3d lamp_position_2d(lamp_position.x(), lamp_position.y(), m_ground_level - 1);

        const double distance = (point - lamp_position_2d).norm();

        if (!std::isnan(distance) && distance < m_parameters.lamp.range && distance > m_parameters.preprocessing.safety_radius)
        {
            if(compute2DVisibility(lamp_position_2d, point))
            {
                irradiance = m_parameters.lamp.power / (4 * M_PI * std::pow(distance, 2));
            }
        }

        return irradiance;
    }



    // 2 - Conroy - 2D model, d = d_2D + h/2, 2D angle (lenghtened ray), point light source 
    double DoseCalculator::computeIrradianceType2(const point3d& lamp_position, const OcTreeKey& key) const
    {
        double irradiance = 0;

        const point3d point = m_augmented_model->keyToCoord(key, m_depth);

        const point3d lamp_position_2d(lamp_position.x(), lamp_position.y(), m_ground_level - 1);

        const double d2 = (point - lamp_position_2d).norm();

        // Modification: instead of h/2 we use lamp_offset + lamp_height / 2
        const double distance = std::sqrt(std::pow(d2, 2) + 
            std::pow(m_parameters.lamp.offset + m_parameters.lamp.height / 2, 2));

        if (!std::isnan(distance) && distance < m_parameters.lamp.range && d2 > m_parameters.preprocessing.safety_radius)
        {
            NodePtr node = m_augmented_model->search(key, m_depth);

            if (node != nullptr) // 2D element with known normal vector
            {
                const point3d normal_2d = node->getNormal();
                
                if (!std::isnan(normal_2d.norm()))
                {
                    if(compute2DVisibility(lamp_position_2d, point))
                    {
                        const point3d difference_2d = point - lamp_position_2d;

                        irradiance = m_parameters.lamp.power * std::abs(std::cos(normal_2d.angleTo(difference_2d))) / (4 * M_PI * std::pow(distance, 2));
                    }
                }   
            }
            else // 2D element where the ray needs to be lengthened
            {
                if(compute2DVisibility(lamp_position_2d, point))
                {
                    const point3d difference_2d = point - lamp_position_2d;

                    point3d end_point;

                    if (m_augmented_model->castRay(point, difference_2d, lamp_position_2d, end_point, true, 1.1 * std::abs(difference_2d.norm()), m_depth, m_resolution))
                    {
                        NodePtr end_node = m_augmented_model->search(end_point, m_depth);

                        if (end_node != nullptr)
                        {
                            const point3d normal_2d = end_node->getNormal();

                            if (!std::isnan(normal_2d.norm()))
                            {
                                const point3d difference_end = end_point - lamp_position_2d;

                                irradiance = m_parameters.lamp.power * std::abs(std::cos(normal_2d.angleTo(difference_end))) / (4 * M_PI * std::pow(distance, 2));
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

        const point3d point = m_augmented_model->keyToCoord(key, m_depth);

        static double base_coverage = (((m_parameters.lamp.height + m_parameters.lamp.offset) / m_parameters.lamp.height)) * m_parameters.preprocessing.safety_radius;

        NodePtr node = m_augmented_model->search(key, m_depth);

        if (node->getType() == 2)
        {
            const point3d base_point = point3d(lamp_position.x(), lamp_position.y(), m_ground_level);

            if (std::abs(base_point.x() - point.x()) < base_coverage && std::abs(base_point.y() - point.y()) < base_coverage)
            {
                return 0;
            }
        }

        const double distance = (point - lamp_position).norm();

        if (!std::isnan(distance) && distance < m_parameters.lamp.range)
        {
            if (m_parameters.computation.type == 5)
            {
                if (compute3DVisibility(lamp_position, point))
                {
                    irradiance = m_parameters.lamp.power / (4 * M_PI * std::pow(distance, 2));
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

                        irradiance = m_parameters.lamp.power * std::abs(std::cos(normal.angleTo(difference))) / (4 * M_PI * std::pow(distance, 2));     
                    }
                }
            }
        }

        return irradiance;
    }



    // 4 - Tiseni (cylindrical), Mine - 3D model, 3D distance, 3D angle, cylindrical light source
    double DoseCalculator::computeIrradianceType4(const point3d& lamp_position, const OcTreeKey& key, int index)
    {
        double irradiance = 0;

        const point3d point = m_augmented_model->keyToCoord(key, m_depth);

        static double base_coverage = (((m_parameters.lamp.height + m_parameters.lamp.offset) / m_parameters.lamp.height)) * m_parameters.preprocessing.safety_radius;

        NodePtr node = m_augmented_model->search(key, m_depth);

        if (node->getType() == 2)
        {
            const point3d base_point = point3d(lamp_position.x(), lamp_position.y(), m_ground_level);

            if (std::abs(base_point.x() - point.x()) < base_coverage && std::abs(base_point.y() - point.y()) < base_coverage)
            {
                return 0;
            }
        }

        const double distance = (point - lamp_position).norm();

        if (!std::isnan(distance) && distance < m_parameters.lamp.range)
        {
            const point3d normal = node->getNormal();

            if (!std::isnan(normal.norm()))
            {
                const double L = m_resolution;

                const double coefficient = m_parameters.lamp.power / (4 * M_PI * m_parameters.lamp.height);

                point3d center = lamp_position;

                for (int i = 1; i < m_ray_targets.size() - 1; ++i)
                {
                    center.z() = m_ray_targets[i];

                    if ((m_parameters.computation.type != 8 && compute3DVisibility(center, point)) ||
                        (m_parameters.computation.type == 8 && compute3DVisibility2(center, point, index)))
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

        point3d target(lamp_position.x(), lamp_position.y(), m_ground_level - 1);

        if (OcTreeKey key; m_augmented_model->coordToKeyChecked(target, m_depth, key))
        {
            target = m_augmented_model->keyToCoord(key, m_depth);
        }

        point3d point(element.x(), element.y(), m_ground_level - 1);

        const point3d direction = target - point;

        const double distance = 1.1 * direction.norm();

        double offset = (direction.x() >= 0) ? m_resolution : -m_resolution;

        point3d origin = point + point3d(offset, 0, 0);

        point3d target_direction = target - origin;

        point3d end_point;

        if (m_augmented_model->checkRayCast(false, target, origin, target_direction, end_point, distance, m_depth, m_resolution, true))
        {
            offset = (direction.y() >= 0) ? m_resolution : -m_resolution;

            origin = point + point3d(0, offset, 0);

            target_direction = target - origin;

            if (m_augmented_model->checkRayCast(true, target, origin, target_direction, end_point, distance, m_depth, m_resolution, true))
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

        const int ground_level_key = m_augmented_model->coordToKey(m_ground_level, m_depth);

        OcTreeKey k_lamp_3d = m_augmented_model->coordToKey(lamp_position, m_depth);

        OcTreeKey k_element_3d = m_augmented_model->coordToKey(element);

        point3d target = m_augmented_model->keyToCoord(k_lamp_3d, m_depth);

        if (m_augmented_model->coordToKey(element.z(), m_depth) == ground_level_key)
        {
            rpo::NodePtr n_augmented = m_augmented_model->search(point3d(element.x(), element.y(), element.z() + m_resolution));

            if (n_augmented != nullptr && m_augmented_model->isNodeOccupied(n_augmented))
            {
                return false;
            } 
        }

        const point3d direction = target - element;

        const double distance = 1.1 * direction.norm();

        double offset = (direction.x() > 0) ? m_resolution : -m_resolution;

        point3d origin = element + point3d(offset, 0, 0);

        point3d target_direction = target - origin;

        point3d end_point;


        if (k_element_3d[0] == k_lamp_3d[0] || !m_augmented_model->castRay4(origin, target_direction, end_point, true, distance, k_lamp_3d))
        {
            offset = (direction.y() > 0) ? m_resolution : -m_resolution;

            origin = element + point3d(0, offset, 0);

            target_direction = target - origin;

            if (k_element_3d[1] == k_lamp_3d[1] || !m_augmented_model->castRay4(origin, target_direction, end_point, true, distance, k_lamp_3d))
            {
                double offset = (direction.z() > 0) ? m_resolution : -m_resolution;

                point3d origin = element + point3d(0, 0, offset);
                
                point3d target_direction = target - origin;

                if (k_element_3d[2] != k_lamp_3d[2] && m_augmented_model->castRay4(origin, target_direction, end_point, true, distance, k_lamp_3d))
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



    void DoseCalculator::computeGeneralVisibility()
    {
        m_augmented_model->expand();

        const int margin = 10;

        double min_x, min_y, min_z, max_x, max_y, max_z;

        m_augmented_model->getMetricMin(min_x, min_y, min_z);
        m_augmented_model->getMetricMax(max_x, max_y, max_z);

        OcTreeKey k_min = m_augmented_model->coordToKey(min_x + m_resolution / 2.0, min_y + m_resolution / 2.0, min_z + m_resolution / 2.0, m_depth);
        OcTreeKey k_max = m_augmented_model->coordToKey(max_x - m_resolution / 2.0, max_y - m_resolution / 2.0, max_z - m_resolution / 2.0, m_depth);

        // Check: all 6 neighbors: being too close to border or direct obstacle
        for (AugmentedOcTree::leaf_iterator it = m_augmented_model->begin_leafs(), end = m_augmented_model->end_leafs(); it != end; ++it)
        {
            OcTreeKey key = it.getKey();

            bool reachable = true;

            // Check -x shift
            OcTreeKey k_neighbor_xm = key;

            k_neighbor_xm[0] -= 1;

            NodePtr n_neighbor_xm = m_augmented_model->search(k_neighbor_xm, m_depth);

            if (key[0] - k_min[0] < margin || n_neighbor_xm != nullptr && m_augmented_model->isNodeOccupied(n_neighbor_xm))
            {
                // Check +x shift
                OcTreeKey k_neighbor_xp = key;

                k_neighbor_xp[0] += 1;

                NodePtr n_neighbor_xp = m_augmented_model->search(k_neighbor_xp, m_depth);

                if (k_max[0] - key[0] < margin || n_neighbor_xp != nullptr && m_augmented_model->isNodeOccupied(n_neighbor_xp))
                {
                    // Check -y shift
                    OcTreeKey k_neighbor_ym = key;

                    k_neighbor_ym[1] -= 1;

                    NodePtr n_neighbor_ym = m_augmented_model->search(k_neighbor_ym, m_depth);

                    if (key[1] - k_min[1] < margin || n_neighbor_ym != nullptr && m_augmented_model->isNodeOccupied(n_neighbor_ym))
                    {
                        // Check +y shift
                        OcTreeKey k_neighbor_yp = key;

                        k_neighbor_yp[1] += 1;

                        NodePtr n_neighbor_yp = m_augmented_model->search(k_neighbor_yp, m_depth);

                        if (k_max[1] - key[1] < margin || n_neighbor_yp != nullptr && m_augmented_model->isNodeOccupied(n_neighbor_yp))
                        {
                            // Check -z shift
                            OcTreeKey k_neighbor_zm = key;

                            k_neighbor_zm[2] -= 1;

                            NodePtr n_neighbor_zm = m_augmented_model->search(k_neighbor_zm, m_depth);

                            if (key[2] - k_min[2] < 2 || n_neighbor_zm != nullptr && m_augmented_model->isNodeOccupied(n_neighbor_zm))
                            {
                                // Check +z shift
                                OcTreeKey k_neighbor_zp = key;

                                k_neighbor_zp[2] += 1;

                                NodePtr n_neighbor_zp = m_augmented_model->search(k_neighbor_zp, m_depth);

                                if (k_max[2] - key[2] < 2 || n_neighbor_zp != nullptr && m_augmented_model->isNodeOccupied(n_neighbor_zp))
                                {
                                    reachable = false;
                                }
                            }
                        }
                    }
                }
            }

            if (reachable)
            {   
                m_base_reachable_elements.insert(key);
            } 
        }

        std::cout << "Filter: " << m_base_reachable_elements.size() << std::endl;
    }




    void DoseCalculator::computeBreakPoints()
    {
        std::cout << "Compute break points!" << std::endl;

        m_break_points_x.resize(m_grid_elements.size());
        m_break_points_y.resize(m_grid_elements.size());
        m_break_points_z.resize(m_grid_elements.size());
        m_break_points_xn.resize(m_grid_elements.size());
        m_break_points_yn.resize(m_grid_elements.size());
        m_break_points_zn.resize(m_grid_elements.size());


        #pragma omp parallel for
        for (int i = 0; i < m_grid_elements.size(); ++i)
        {
            point3d p_lamp_2d = m_augmented_model->keyToCoord(m_grid_elements[i], m_depth);

            p_lamp_2d.z() = m_ground_level - 1;

            

            for (const auto& r_element : m_base_reachable_elements)
            {
                point3d p_element_2d = m_augmented_model->keyToCoord(r_element, m_depth);

                p_element_2d.z() = m_ground_level - 1;

                OcTreeKey k_element_2d = m_augmented_model->coordToKey(p_element_2d, m_depth);

                if (m_break_points_x[i].find(k_element_2d) == m_break_points_x[i].end())
                {
                    const point3d direction_2d = p_lamp_2d - p_element_2d;

                    const double distance_2d = 1.1 * direction_2d.norm();

                    // Check x direction shift
                    point3d p_element_2d_x = point3d(getOrigin(p_element_2d.x(), direction_2d.x()), p_element_2d.y(), p_element_2d.z());

                    point3d direction_2d_x = p_lamp_2d - p_element_2d_x;

                    if (direction_2d_x.norm() > 0.001)
                    {
                        point3d p_end_x;

                        m_augmented_model->castRay2(p_element_2d_x, direction_2d_x, p_lamp_2d, p_end_x, true, 
                            distance_2d, m_depth, m_resolution, m_break_points_x[i][k_element_2d], m_break_points_xn[i][k_element_2d], false);
                    }

                    // Check y direction shift
                    point3d p_element_2d_y = point3d(p_element_2d.x(), getOrigin(p_element_2d.y(), direction_2d.y()), p_element_2d.z());

                    point3d direction_2d_y = p_lamp_2d - p_element_2d_y;

                    if (direction_2d_y.norm() > 0.001)
                    {
                        point3d p_end_y;

                        m_augmented_model->castRay2(p_element_2d_y, direction_2d_y, p_lamp_2d, p_end_y, true,
                            distance_2d, m_depth, m_resolution, m_break_points_y[i][k_element_2d], m_break_points_yn[i][k_element_2d], false);
                    }
                    
                    // No shift for z direction
                    point3d p_element_2d_z = p_element_2d;

                    point3d direction_2d_z = p_lamp_2d - p_element_2d_z;

                    point3d p_end_z;

                    m_augmented_model->castRay2(p_element_2d_z, direction_2d_z, p_lamp_2d, p_end_z, true,
                        distance_2d, m_depth, m_resolution, m_break_points_z[i][k_element_2d], m_break_points_zn[i][k_element_2d], false);
                }
            }
        }


        std::cout << "Break points: " << m_break_points_x.size() << " " << m_break_points_y.size() << " " << m_break_points_z.size() << "\n";
    }

    inline double DoseCalculator::getOrigin(const double base, const double direction) const
    {
        if (direction < 0)
        {
            return base - m_resolution;
        }
        else if (direction == 0)
        {
            return base;
        }
        else
        {
            return base + m_resolution;
        }
    }


    bool DoseCalculator::compute3DVisibility2(const point3d& lamp_position, const point3d& element, int index)
    {
        signal (SIGINT, handler);

        const int ground_level_key = m_augmented_model->coordToKey(m_ground_level, m_depth);

        OcTreeKey k_lamp_3d = m_augmented_model->coordToKey(lamp_position, m_depth);

        point3d p_lamp_3d = m_augmented_model->keyToCoord(k_lamp_3d, m_depth);

        point3d p_element_3d = element;

        OcTreeKey k_element_3d = m_augmented_model->coordToKey(p_element_3d, m_depth);

        point3d direction_3d = p_lamp_3d - p_element_3d;

        point3d p_element_2d = point3d(p_element_3d.x(), p_element_3d.y(), m_ground_level - 1);

        OcTreeKey k_element_2d = m_augmented_model->coordToKey(p_element_2d, m_depth);

        double distance_3d_z = p_lamp_3d.z() - p_element_3d.z();

        // Check if there are zero break points in direction x, y, z
        if (k_element_3d[2] != ground_level_key && (m_break_points_x[index][k_element_2d].size() == 0 || 
            m_break_points_y[index][k_element_2d].size() == 0 || m_break_points_z[index][k_element_2d].size() == 0))
        {
            return true;
        }
        else if (k_element_3d[2] == ground_level_key && m_break_points_z[index][k_element_2d].size() == 0)
        {
            // Take into account elements at the bottom of walls
            NodePtr node = m_augmented_model->search(point3d(p_element_3d.x(), p_element_3d.y(), p_element_3d.z() + m_resolution));

            if (node == nullptr || !m_augmented_model->isNodeOccupied(node))
            {
                return true;
            } 
        }
        else
        {
            // Check break keys in x direction
            point3d p_element_3d_x = point3d(getOrigin(p_element_3d.x(), direction_3d.x()), p_element_3d.y(), p_element_3d.z());

            if (k_element_3d[2] != ground_level_key && m_augmented_model->checkBreakPoints(m_break_points_x[index][k_element_2d], 
                m_break_points_xn[index][k_element_2d], p_lamp_3d, p_element_3d_x, m_ground_level))
            {
                return true;
            }
            else
            {
                // Check break keys in y direction
                point3d p_element_3d_y = point3d(p_element_3d.x(), getOrigin(p_element_3d.y(), direction_3d.y()), p_element_3d.z());

                if (k_element_3d[2] != ground_level_key && m_augmented_model->checkBreakPoints(m_break_points_y[index][k_element_2d], 
                    m_break_points_yn[index][k_element_2d], p_lamp_3d, p_element_3d_y, m_ground_level))
                {
                    return true;
                }
                else
                {
                    point3d p_element_3d_z = point3d(p_element_3d.x(), p_element_3d.y(), getOrigin(p_element_3d.z(), direction_3d.z()));

                    NodePtr n_query = m_augmented_model->search(p_element_3d_z, m_depth);

                    if (n_query == nullptr || !m_augmented_model->isNodeOccupied(n_query))
                    {
                        if (m_augmented_model->checkBreakPoints(m_break_points_z[index][k_element_2d], 
                            m_break_points_zn[index][k_element_2d], p_lamp_3d, p_element_3d_z, m_ground_level))
                        {
                            return true;
                        }
                    }
                }
            }
        }

        return false;
    }
}


