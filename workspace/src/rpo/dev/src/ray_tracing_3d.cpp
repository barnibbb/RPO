#include <string>
#include <memory>
#include <chrono>

#include "extended_octree.h"
#include "dose_calculator.h"

bool checkBreakPoints(
    const std::shared_ptr<rpo::ExtendedOcTree>& extended_model,
    const std::vector<OcTreeKey>& break_keys,
    const std::vector<OcTreeKey>& break_keys_n, 
    const point3d& p_target_3d,
    const point3d& p_origin_3d,
    const double floor_plan_level,
    bool show = false);

double getOrigin(const double base, const double direction);

KeySet readKeys(const std::string& path);

KeySet checkAvailability(std::shared_ptr<rpo::ExtendedOcTree>& extended_model, std::shared_ptr<octomap::ColorOcTree>& color_model);

void adjustModel(std::shared_ptr<rpo::ExtendedOcTree>& extended_model, std::shared_ptr<octomap::ColorOcTree>& color_model);

int main (int argc, char** argv)
{
    double param_z = 0.325;
    const int steps = 22;

    // 1. Read extended and color models
    // 2. Perform necessary modifications on both
    // 3. Compute reachable elements by traditional ray tracing --> model1
    // 4. Compute break points using 2D ray tracing
    // 5. Compute reachable elements based on break points --> model2
    // 6. Compare model1 and model2

    // 1. Read 3D models, and the difference elements
    // std::string extra_element_file = "/home/barni/rpo_ws/src/rpo/experiments/test/extra_keys.txt";
    // std::string missing_element_file = "/home/barni/rpo_ws/src/rpo/experiments/test/missing_keys.txt";

    // KeySet extra_element_keys = readKeys(extra_element_file);
    // KeySet missing_element_keys = readKeys(missing_element_file);

    // std::cout << "Number of extra elements: " << extra_element_keys.size() << std::endl;
    // std::cout << "Number of missing elements: " << missing_element_keys.size() << std::endl;

    std::string color_model_file = "/home/barni/rpo_ws/src/rpo/experiments/test/models/infirmary_color.ot";
    std::string extended_model_file = "/home/barni/rpo_ws/src/rpo/experiments/test/models/infirmary_extended.ot";

    std::shared_ptr<ColorOcTree> color_model = nullptr;
    std::shared_ptr<rpo::ExtendedOcTree> extended_model = nullptr;

    std::ifstream file(color_model_file);

    color_model.reset(dynamic_cast<ColorOcTree*>(AbstractOcTree::read(file)));

    file.close();

    file.open(extended_model_file);

    extended_model.reset(dynamic_cast<rpo::ExtendedOcTree*>(AbstractOcTree::read(file)));

    file.close();

    std::cout << "Number of elements: " << color_model->getNumLeafNodes() << std::endl;


    // Initialize constants
    const double ground_level = -0.125; // infirmary
    //const double ground_level = 0.075; // office
    //const double ground_level = 0.075; // café
    const double floor_plan_level = -1.125;
    const double resolution = 0.05;
    const int depth = 16;
    const int ground_level_key = extended_model->coordToKey(ground_level, depth);
    const int floor_plan_level_key = extended_model->coordToKey(floor_plan_level, depth);
    const double pos_x = 0.375;
    const double pos_y = -0.225;

    // 2. - Delete elements under ground_level
    KeySet keys_to_delete;

    for (ColorOcTree::leaf_iterator it = color_model->begin_leafs(), end = color_model->end_leafs(); it != end; ++it)
    {
        if (it.getCoordinate().z() < (ground_level - 0.025))
        {
            keys_to_delete.insert(it.getKey());
        }
    }

    for (const auto& key_to_delete : keys_to_delete)
    {
        color_model->deleteNode(key_to_delete, depth);
        extended_model->deleteNode(key_to_delete, depth);
    }

    std::cout << "Cut size: " << extended_model->getNumLeafNodes() << std::endl;

    // Support function to fill in 'gaps' in the wall for infirmary model, should be moved to separate code.
    adjustModel(extended_model, color_model);

    // Checking general reachability of elements thus limiting the number of voxels to be considered during the ray tracing.
    KeySet reachable_elements = checkAvailability(extended_model, color_model);

    // 2. - Create 2D model
    KeySet obstacles_2d;

    // Step 1: Map obstacles to the x-y plane
    for (rpo::ExtendedOcTree::leaf_iterator it = extended_model->begin_leafs(), end = extended_model->end_leafs(); it != end; ++it)
    {
        point3d point_3d = it.getCoordinate();
    
        point3d point_2d(point_3d.x(), point_3d.y(), floor_plan_level);

        if (OcTreeKey key_2d; extended_model->coordToKeyChecked(point_2d, depth, key_2d))
        {
            if ((point_3d.z() - ground_level) > 0.0001)
            {
                obstacles_2d.insert(key_2d);
            }
        }
    }

    // Step 2: Insert 2D obstacles to the extended 3D model
    for (const auto& obstacle_key : obstacles_2d)
    {
        rpo::NodePtr extended_node = extended_model->search(obstacle_key, depth);
        ColorOcTreeNode* color_node = color_model->search(obstacle_key, depth);

        if (extended_node == nullptr)
        {
            extended_node = extended_model->updateNode(obstacle_key, true);
            color_node = color_model->updateNode(obstacle_key, true);
        }
    }

    std::cout << "Extended size: " << extended_model->getNumLeafNodes() << std::endl;


    
    // 3. Check the number of reachable elements using the original ray tracing  
    // Variable convention: {type: p | k | n}_{role: lamp | element}_{geomtery: 2d | 3d}_{additional_info: e.g. shifted}

    // Infirmary:
    point3d p_lamp_3d(pos_x, pos_y, param_z);

    // Office:
    // point3d p_lamp_3d(0.525, 0.525, param_z);

    // Café:
    // point3d p_lamp_3d(-0.325, 0.125, param_z);

    KeySet reachable_elements_1;

    // If criteria == 0 then use extended octree ray tracing, if criteria == 1 then use color octree ray tracing
    unsigned int criteria = 0;

    std::cout << "Criteria: " << criteria << std::endl;

    KeySet missing_keys_xp, missing_keys_xm;
    KeySet missing_keys_yp, missing_keys_ym;
    KeySet missing_keys_zp, missing_keys_zm;

    auto s1 = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < steps; ++i)
    {
        ColorOcTreeNode* n_color = color_model->updateNode(p_lamp_3d, true);

        OcTreeKey k_lamp_3d = extended_model->coordToKey(p_lamp_3d, depth);

        // for (rpo::ExtendedOcTree::leaf_iterator it = extended_model->begin_leafs(), end = extended_model->end_leafs(); it != end; ++it)
        for (const auto& r_element : reachable_elements)
        {
            // point3d p_element_3d = it.getCoordinate();
            // OcTreeKey k_element_3d = it.getKey();

            point3d p_element_3d = extended_model->keyToCoord(r_element, depth);
            OcTreeKey k_element_3d = r_element;

            if (p_element_3d.z() > -0.5)
            {
                if (extended_model->coordToKey(p_element_3d.z(), depth) == ground_level_key)
                {
                    rpo::NodePtr n_extended = extended_model->search(point3d(p_element_3d.x(), p_element_3d.y(), p_element_3d.z() + resolution));

                    if (n_extended != nullptr && extended_model->isNodeOccupied(n_extended))
                    {
                        continue;
                    } 
                }


                if (k_element_3d[0] < k_lamp_3d[0])
                {
                    point3d p_element_xp = p_element_3d + point3d(resolution, 0, 0);

                    point3d direction_xp = p_lamp_3d - p_element_xp;

                    point3d p_end_xp;

                    if ((criteria == 0 && extended_model->castRay4(p_element_xp, direction_xp, p_end_xp, true, 100, k_lamp_3d)) ||
                        (criteria == 1 && color_model->castRay(p_element_xp, direction_xp, p_end_xp, true, 100) && color_model->coordToKey(p_end_xp, depth) == k_lamp_3d))
                    {
                        reachable_elements_1.insert(k_element_3d);

                        continue;
                    }
                }
                else if (k_element_3d[0] > k_lamp_3d[0])
                {
                    point3d p_element_xm = p_element_3d + point3d(-resolution, 0, 0);

                    point3d direction_xm = p_lamp_3d - p_element_xm;

                    point3d p_end_xm;

                    if ((criteria == 0 && extended_model->castRay4(p_element_xm, direction_xm, p_end_xm, true, 100, k_lamp_3d)) ||
                        (criteria == 1 && color_model->castRay(p_element_xm, direction_xm, p_end_xm, true, 100) && color_model->coordToKey(p_end_xm, depth) == k_lamp_3d))
                    {
                        reachable_elements_1.insert(k_element_3d);

                        continue;
                    }
                }
                
                
                if (k_element_3d[1] < k_lamp_3d[1])
                {
                    point3d p_element_yp = p_element_3d + point3d(0, resolution, 0);

                    point3d direction_yp = p_lamp_3d - p_element_yp;

                    point3d p_end_yp;

                    if ((criteria == 0 && extended_model->castRay4(p_element_yp, direction_yp, p_end_yp, true, 100, k_lamp_3d)) ||
                        (criteria == 1 && color_model->castRay(p_element_yp, direction_yp, p_end_yp, true, 100) && color_model->coordToKey(p_end_yp, depth) == k_lamp_3d))
                    {
                        reachable_elements_1.insert(k_element_3d);

                        continue;
                    }
                }
                else if (k_element_3d[1] > k_lamp_3d[1])
                {
                    point3d p_element_ym = p_element_3d + point3d(0, -resolution, 0);

                    point3d direction_ym = p_lamp_3d - p_element_ym;

                    point3d p_end_ym;

                    if ((criteria == 0 && extended_model->castRay4(p_element_ym, direction_ym, p_end_ym, true, 100, k_lamp_3d)) ||
                        (criteria == 1 && color_model->castRay(p_element_ym, direction_ym, p_end_ym, true, 100) && color_model->coordToKey(p_end_ym, depth) == k_lamp_3d))
                    {
                        reachable_elements_1.insert(k_element_3d);

                        continue;
                    }
                }
                


                if (k_element_3d[2] < k_lamp_3d[2])
                {
                    point3d p_element_zp = p_element_3d + point3d(0, 0, resolution);

                    point3d direction_zp = p_lamp_3d - p_element_zp;

                    point3d p_end_zp;

                    if ((criteria == 0 && extended_model->castRay4(p_element_zp, direction_zp, p_end_zp, true, 100, k_lamp_3d)) ||
                        (criteria == 1 && color_model->castRay(p_element_zp, direction_zp, p_end_zp, true, 100) && color_model->coordToKey(p_end_zp, depth) == k_lamp_3d))
                    {
                        reachable_elements_1.insert(k_element_3d);

                        continue;
                    }
                }
                else if (k_element_3d[2] > k_lamp_3d[2])
                {
                    point3d p_element_zm = p_element_3d + point3d(0, 0, -resolution);

                    point3d direction_zm = p_lamp_3d - p_element_zm;

                    point3d p_end_zm;

                    if ((criteria == 0 && extended_model->castRay4(p_element_zm, direction_zm, p_end_zm, true, 100, k_lamp_3d)) ||
                        (criteria == 1 && color_model->castRay(p_element_zm, direction_zm, p_end_zm, true, 100) && color_model->coordToKey(p_end_zm, depth) == k_lamp_3d))
                    {   
                        reachable_elements_1.insert(k_element_3d);

                        continue;
                    }
                }
            }         
        }

        color_model->deleteNode(p_lamp_3d, depth);

        p_lamp_3d.z() += resolution;
    }
    
    auto e1 = std::chrono::high_resolution_clock::now();

    std::cout << "Duration 1: " << std::chrono::duration_cast<std::chrono::seconds>(e1 - s1).count() << std::endl;


    for (ColorOcTree::leaf_iterator it = color_model->begin_leafs(), end = color_model->end_leafs(); it != end; ++it)
    {
        ColorOcTreeNode* n_color = color_model->search(it.getKey(), depth);

        if (reachable_elements_1.find(it.getKey()) != reachable_elements_1.end())
        {
            n_color->setColor(255, 0, 0);
        }
        else
        {
            n_color->setColor(0, 0, 255);
        }
    }


    std::cout << "Reachable elements 3D: " << reachable_elements_1.size() << std::endl; 

    std::string reachable_elements_3d_file = "/home/barni/rpo_ws/src/rpo/experiments/test/reachable_3d.ot";

    color_model->write(reachable_elements_3d_file);



    // 4. - Compute break keys in each directions

    // Break keys are strored for each lamp position by the keys of the 2D elements
    std::unordered_map<OcTreeKey, std::vector<OcTreeKey>, OcTreeKey::KeyHash> break_keys_x;
    std::unordered_map<OcTreeKey, std::vector<OcTreeKey>, OcTreeKey::KeyHash> break_keys_y;
    std::unordered_map<OcTreeKey, std::vector<OcTreeKey>, OcTreeKey::KeyHash> break_keys_z;

    std::unordered_map<OcTreeKey, std::vector<OcTreeKey>, OcTreeKey::KeyHash> break_keys_x_n;
    std::unordered_map<OcTreeKey, std::vector<OcTreeKey>, OcTreeKey::KeyHash> break_keys_y_n;
    std::unordered_map<OcTreeKey, std::vector<OcTreeKey>, OcTreeKey::KeyHash> break_keys_z_n;


    // Set target in 2D
    point3d p_lamp_2d = p_lamp_3d;

    p_lamp_2d.z() = floor_plan_level;

    auto s2 = std::chrono::high_resolution_clock::now();

    // for (rpo::ExtendedOcTree::leaf_iterator it = extended_model->begin_leafs(), end = extended_model->end_leafs(); it != end; ++it)
    for (const auto& r_element : reachable_elements)
    {
        // point3d p_element_2d = it.getCoordinate();

        point3d p_element_2d = extended_model->keyToCoord(r_element, depth);

        p_element_2d.z() = floor_plan_level;

        OcTreeKey k_element_2d = extended_model->coordToKey(p_element_2d, depth);

        if (break_keys_x.find(k_element_2d) == break_keys_x.end())
        {
            const point3d direction_2d = p_lamp_2d - p_element_2d;

            const double distance_2d = 1.1 * direction_2d.norm();

            // Check x directon shift
            point3d p_element_2d_x = point3d(getOrigin(p_element_2d.x(), direction_2d.x()), p_element_2d.y(), p_element_2d.z());

            point3d direction_2d_x = p_lamp_2d - p_element_2d_x;

            point3d p_end_x;

            if (direction_2d_x.norm() > 0.001)
            {
                extended_model->castRay2(p_element_2d_x, direction_2d_x, p_lamp_2d, p_end_x, true, distance_2d, depth, resolution, break_keys_x[k_element_2d], break_keys_x_n[k_element_2d], false);
            }
            
            // Check y direction shift
            point3d p_element_2d_y = point3d(p_element_2d.x(), getOrigin(p_element_2d.y(), direction_2d.y()), p_element_2d.z());

            point3d direction_2d_y = p_lamp_2d - p_element_2d_y;

            point3d p_end_y;

            if (direction_2d_y.norm() > 0.001)
            { 
                extended_model->castRay2(p_element_2d_y, direction_2d_y, p_lamp_2d, p_end_y, true, distance_2d, depth, resolution, break_keys_y[k_element_2d], break_keys_y_n[k_element_2d],false);
            }

            // No shift for z direction
            point3d p_element_2d_z = p_element_2d;

            point3d direction_2d_z = p_lamp_2d - p_element_2d_z;

            point3d p_end_z;

            extended_model->castRay2(p_element_2d_z, direction_2d_z, p_lamp_2d, p_end_z, true, distance_2d, depth, resolution, break_keys_z[k_element_2d], break_keys_z_n[k_element_2d], false);
        }
    }

    // auto e2 = std::chrono::high_resolution_clock::now();

    // std::cout << "Duration 2: " << std::chrono::duration_cast<std::chrono::seconds>(e2 - s2).count() << std::endl;

    // std::cout << "Break keys x: " << break_keys_x.size() << " " << break_keys_x_n.size() << "\n";
    // std::cout << "Break keys y: " << break_keys_y.size() << " " << break_keys_y_n.size() << "\n";
    // std::cout << "Break keys z: " << break_keys_z.size() << " " << break_keys_z_n.size() << "\n";



    // 5. - Check sections between break points in 3d
    KeySet reachable_elements_2;

    // Infirmary:
    p_lamp_3d = point3d(pos_x, pos_y, param_z);

    // Office:
    // p_lamp_3d = point3d(0.525, 0.525, param_z);

    // Café:
    // p_lamp_3d = point3d(-0.325, 0.125, param_z);

    // auto s3 = std::chrono::high_resolution_clock::now();

    for (int i = 0; i < steps; ++i)
    {
        OcTreeKey k_lamp_3d = extended_model->coordToKey(p_lamp_3d, depth);

        //for (rpo::ExtendedOcTree::leaf_iterator it = extended_model->begin_leafs(), end = extended_model->end_leafs(); it != end; ++it)
        for (const auto& r_element : reachable_elements)
        {
            // point3d p_element_3d = it.getCoordinate();

            // OcTreeKey k_element_3d = it.getKey();

            point3d p_element_3d = extended_model->keyToCoord(r_element, depth);

            OcTreeKey k_element_3d = r_element;

            if (p_element_3d.z() > -0.5)
            {
                point3d direction_3d = p_lamp_3d - p_element_3d;

                point3d p_element_2d = point3d(p_element_3d.x(), p_element_3d.y(), floor_plan_level);

                OcTreeKey k_element_2d = extended_model->coordToKey(p_element_2d, depth);

                double distance_3d_z = p_lamp_3d.z() - p_element_3d.z();

                bool good = false;

                // Check if there are zero break points in direction x, y, z
                if (k_element_3d[2] != ground_level_key && (break_keys_x[k_element_2d].size() == 0 || break_keys_y[k_element_2d].size() == 0 || break_keys_z[k_element_2d].size() == 0))
                {
                    good = true;
                }
                else if (k_element_3d[2] == ground_level_key && break_keys_z[k_element_2d].size() == 0)
                {
                    // Take into account elements at the bottom of walls
                    rpo::NodePtr n_extended = extended_model->search(point3d(p_element_3d.x(), p_element_3d.y(), p_element_3d.z() + resolution));

                    if (n_extended == nullptr || !extended_model->isNodeOccupied(n_extended))
                    {
                        good = true;
                    } 
                }
                else
                {   
                    // Check break keys in x direction
                    point3d p_element_3d_x = point3d(getOrigin(p_element_3d.x(), direction_3d.x()), p_element_3d.y(), p_element_3d.z());

                    if (k_element_3d[2] != ground_level_key && checkBreakPoints(extended_model, break_keys_x[k_element_2d], break_keys_x_n[k_element_2d], p_lamp_3d, p_element_3d_x, floor_plan_level))
                    {
                        good = true;
                    }
                    else
                    {
                        // Check break keys in y direction
                        point3d p_element_3d_y = point3d(p_element_3d.x(), getOrigin(p_element_3d.y(), direction_3d.y()), p_element_3d.z());
                        
                        if (k_element_3d[2] != ground_level_key && checkBreakPoints(extended_model, break_keys_y[k_element_2d], break_keys_y_n[k_element_2d], p_lamp_3d, p_element_3d_y, floor_plan_level))
                        {
                            good = true;
                        }
                        else
                        {
                            // Check break keys in z direction
                            point3d p_element_3d_z = point3d(p_element_3d.x(), p_element_3d.y(), getOrigin(p_element_3d.z(), direction_3d.z()));

                            rpo::NodePtr n_query = extended_model->search(p_element_3d_z, depth);

                            if (n_query == nullptr || !extended_model->isNodeOccupied(n_query))
                            {
                                if (checkBreakPoints(extended_model, break_keys_z[k_element_2d], break_keys_z_n[k_element_2d], p_lamp_3d, p_element_3d_z, floor_plan_level))
                                {
                                    good = true;
                                }
                            }
                        }
                    }
                }
                

                if (good)
                {
                    reachable_elements_2.insert(k_element_3d);
                }
            }
        }

        p_lamp_3d.z() += resolution;
    }

    
    auto e3 = std::chrono::high_resolution_clock::now();

    std::cout << "Duration 3: " << std::chrono::duration_cast<std::chrono::seconds>(e3 - s2).count() << std::endl;

    for (ColorOcTree::leaf_iterator it = color_model->begin_leafs(), end = color_model->end_leafs(); it != end; ++it)
    {
        ColorOcTreeNode* n_color = color_model->search(it.getKey(), depth);

        if (reachable_elements_2.find(it.getKey()) != reachable_elements_2.end()) 
        {
            n_color->setColor(255, 0, 0);
        }
        else 
        {
            n_color->setColor(0, 0, 255);
        }
    }

    std::cout << "Reachable elements 2D: " << reachable_elements_2.size() << std::endl; 

    std::string out_file2 = "/home/barni/rpo_ws/src/rpo/experiments/test/reachable_2d.ot";

    color_model->write(out_file2);

    return 0;
}



bool checkBreakPoints(
    const std::shared_ptr<rpo::ExtendedOcTree>& extended_model,
    const std::vector<OcTreeKey>& break_keys, 
    const std::vector<OcTreeKey>& break_keys_n, 
    const point3d& p_target_3d,
    const point3d& p_origin_3d,
    const double floor_plan_level,
    bool show)
{
    auto fs = std::chrono::high_resolution_clock::now();

    const double resolution = 0.05;
    const int depth = 16;

    OcTreeKey k_target_3d = extended_model->coordToKey(p_target_3d, depth);
    OcTreeKey k_origin_3d = extended_model->coordToKey(p_origin_3d, depth);

    point3d p_target_2d = point3d(p_target_3d.x(), p_target_3d.y(), floor_plan_level);
    point3d p_origin_2d = point3d(p_origin_3d.x(), p_origin_3d.y(), floor_plan_level);

    double distance_2d = (p_target_2d - p_origin_2d).norm();

    point3d direction_3d = p_target_3d - p_origin_3d;
    point3d direction_3d_normalized = direction_3d.normalized();

    std::vector<point3d> break_points(break_keys.size());
    std::vector<double> break_distances(break_keys.size());

    for (int i = 0; i < break_points.size(); ++i)
    {
        break_points[i] = extended_model->keyToCoord(break_keys[i]);
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

        /*
        if (step_x == 0 && step_y == 0)
        {
            t_max[0] = (direction_3d_normalized.x() == 0) ? std::numeric_limits<double>::max() : (double(step_x) + 0.5) * resolution / std::fabs(direction_3d_normalized.x());

            t_max[1] = (direction_3d_normalized.y() == 0) ? std::numeric_limits<double>::max() : (double(step_y) + 0.5) * resolution / std::fabs(direction_3d_normalized.y());
        }
        else if ((step_x >= step_y && step_x % 2 == 0) || (step_x < step_y && step_y % 2 != 0) || step_y == 0)
        {
            t_max[0] = (double(step_x - 1) + 0.5) * resolution / std::fabs(direction_3d_normalized.x());

            t_max[1] = (direction_3d_normalized.y() == 0) ? std::numeric_limits<double>::max() : (double(step_y) + 0.5) * resolution / std::fabs(direction_3d_normalized.y());
        }
        else if ((step_x >= step_y && step_x % 2 != 0) || (step_x < step_y && step_y % 2 == 0) || step_x == 0)
        {
            t_max[0] = (direction_3d_normalized.x() == 0) ? std::numeric_limits<double>::max() : (double(step_x) + 0.5) * resolution / std::fabs(direction_3d_normalized.x());

            t_max[1] = (double(step_y - 1) + 0.5) * resolution / std::fabs(direction_3d_normalized.y());
        }
        */
                            
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

            rpo::NodePtr n_single = extended_model->search(p_query, depth);

            if (n_single && extended_model->isNodeOccupied(n_single))
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

                n_single = extended_model->search(p_query, depth);

                if (n_single && extended_model->isNodeOccupied(n_single))
                {
                    return false;
                }

                t_max[2] += resolution / std::fabs(direction_3d_normalized.z());
            }
        }
        else
        {
            point3d p_end;

            if (!extended_model->castRay3(p_break_3d_1, direction_3d, p_break_3d_2, p_end, true, 10, depth, resolution, t_max[0], t_max[1], t_max[2], show))
            {   
                return false;
            }
        }
    }

    return true;
}


double getOrigin(const double base, const double direction)
{
    if (direction < 0)
    {
        return base - 0.05;
    }
    else if (direction == 0)
    {
        return base;
    }
    else
    {
        return base + 0.05;
    }
}


KeySet readKeys(const std::string& path)
{
    KeySet keys;

    std::ifstream file(path);

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

            keys.insert(key);
        }

        file.close();
    }
    else
    {
        std::cerr << "Could not open input file for loading irradiance map!" << std::endl;
    }

    return keys;
}


KeySet checkAvailability(std::shared_ptr<rpo::ExtendedOcTree>& extended_model, std::shared_ptr<octomap::ColorOcTree>& color_model)
{
    extended_model->expand();

    KeySet reachable_elements;

    int margin = 10;

    double min_x, min_y, min_z, max_x, max_y, max_z;

    extended_model->getMetricMin(min_x, min_y, min_z);
    extended_model->getMetricMax(max_x, max_y, max_z);

    std::cout << min_x << "\t" << max_x << "\t" << min_y << "\t" << max_y << "\t" << min_z << "\t" << max_z << "\n";

    OcTreeKey k_min = extended_model->coordToKey(min_x + 0.025, min_y + 0.025, min_z + 0.025, 16);
    OcTreeKey k_max = extended_model->coordToKey(max_x - 0.025, max_y - 0.025, max_z - 0.025, 16);

    // Check: all 6 neighbors: being too close to border or direct obstacle
    for (rpo::ExtendedOcTree::leaf_iterator it = extended_model->begin_leafs(), end = extended_model->end_leafs(); it != end; ++it)
    {
        OcTreeKey key = it.getKey();

        bool reachable = true;

        // Check -x shift
        OcTreeKey k_neighbor_xm = key;

        k_neighbor_xm[0] -= 1;

        rpo::NodePtr n_neighbor_xm = extended_model->search(k_neighbor_xm, 16);

        if (key[0] - k_min[0] < margin || n_neighbor_xm != nullptr && extended_model->isNodeOccupied(n_neighbor_xm))
        {
            // Check +x shift
            OcTreeKey k_neighbor_xp = key;

            k_neighbor_xp[0] += 1;

            rpo::NodePtr n_neighbor_xp = extended_model->search(k_neighbor_xp, 16);

            if (k_max[0] - key[0] < margin || n_neighbor_xp != nullptr && extended_model->isNodeOccupied(n_neighbor_xp))
            {
                // Check -y shift
                OcTreeKey k_neighbor_ym = key;

                k_neighbor_ym[1] -= 1;

                rpo::NodePtr n_neighbor_ym = extended_model->search(k_neighbor_ym, 16);

                if (key[1] - k_min[1] < margin || n_neighbor_ym != nullptr && extended_model->isNodeOccupied(n_neighbor_ym))
                {
                    // Check +y shift
                    OcTreeKey k_neighbor_yp = key;

                    k_neighbor_yp[1] += 1;

                    rpo::NodePtr n_neighbor_yp = extended_model->search(k_neighbor_yp, 16);

                    if (k_max[1] - key[1] < margin || n_neighbor_yp != nullptr && extended_model->isNodeOccupied(n_neighbor_yp))
                    {
                        // Check -z shift
                        OcTreeKey k_neighbor_zm = key;

                        k_neighbor_zm[2] -= 1;

                        rpo::NodePtr n_neighbor_zm = extended_model->search(k_neighbor_zm, 16);

                        if (key[2] - k_min[2] < 2 || n_neighbor_zm != nullptr && extended_model->isNodeOccupied(n_neighbor_zm))
                        {
                            // Check +z shift
                            OcTreeKey k_neighbor_zp = key;

                            k_neighbor_zp[2] += 1;

                            rpo::NodePtr n_neighbor_zp = extended_model->search(k_neighbor_zp, 16);

                            if (k_max[2] - key[2] < 2 || n_neighbor_zp != nullptr && extended_model->isNodeOccupied(n_neighbor_zp))
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
            reachable_elements.insert(key);
        } 
    }

    for (const auto& key : reachable_elements)
    {
        ColorOcTreeNode* c_node = color_model->search(key, 16);

        c_node->setColor(255, 0, 0);
    }

    // std::cout << "All elements: " << extended_model->getNumLeafNodes() << "\n";

    std::cout << "Reachable elements: " << reachable_elements.size() << "\n";

    color_model->expand();

    color_model->write("/home/barni/rpo_ws/src/rpo/experiments/test/t_reachable.ot");

    return reachable_elements;
}


void adjustModel(std::shared_ptr<rpo::ExtendedOcTree>& extended_model, std::shared_ptr<octomap::ColorOcTree>& color_model)
{
    extended_model->expand();
    color_model->expand();

    double min_x, min_y, min_z, max_x, max_y, max_z;

    extended_model->getMetricMin(min_x, min_y, min_z);
    extended_model->getMetricMax(max_x, max_y, max_z);

    // Fill first wall
    for (int i = 2; i < 4; ++i)
    {
        double x = min_x + 10.5 * 0.05;

        while (x < max_x)
        {
            double z = min_z + 0.025;

            while (z < max_z)
            {
                point3d p(x, max_y - i * 0.05 - 0.025, z);

                ColorOcTreeNode* c_node = color_model->updateNode(p, true);

                c_node->setColor(255, 255, 255);

                rpo::NodePtr a_node = extended_model->updateNode(p, true);

                z += 0.05;
            }

            x += 0.05;;
        }
    }

    // Second wall
    for (int i = 2; i < 4; ++i)
    {
        double x = min_x + 10.5 * 0.05;

        while (x < max_x)
        {
            double z = min_z + 0.025;

            while (z < max_z)
            {
                point3d p(x, min_y + i * 0.05 + 0.025, z);

                ColorOcTreeNode* c_node = color_model->updateNode(p, true);

                c_node->setColor(255, 255, 255);
                
                rpo::NodePtr a_node = extended_model->updateNode(p, true);

                z += 0.05;
            }

            x += 0.05;;
        }
    }

    // Third wall
    for (int i = 2; i < 4; ++i)
    {
        double y = min_y + 0.025;

        while (y < max_y)
        {
            double z = min_z + 0.025;

            while (z < max_z)
            {
                point3d p(max_x - i * 0.05 - 0.025, y, z);

                ColorOcTreeNode* c_node = color_model->updateNode(p, true);

                c_node->setColor(255, 255, 255);

                rpo::NodePtr a_node = extended_model->updateNode(p, true);

                z += 0.05;
            }

            y += 0.05;;
        }
    }

    color_model->expand();

    extended_model->expand();

    color_model->write("/home/barni/rpo_ws/src/rpo/experiments/test/infirmary_color_mod.ot");

    extended_model->write("/home/barni/rpo_ws/src/rpo/experiments/test/infirmary_extended_mod.ot");

    std::cout << "Extended size: " << color_model->getNumLeafNodes() << std::endl;
}


