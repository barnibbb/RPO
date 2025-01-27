#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include "augmented_octree.h"

struct Vertex { double x, y, z; unsigned char r, g, b; };
void readPlyBinary(const std::string& filename, std::vector<Vertex>& vertices); 
void cutModel(rpo::AugmentedOcTree& a_tree, octomap::ColorOcTree& c_tree);

Eigen::Matrix4f createRotationMatrix(float angle_x, float angle_y, float angle_z);


int main(int argc, char** argv)
{
    // Read input ply file
    std::vector<Vertex> vertices;

    const std::string ply_file = "/home/barni/rpo_ws/src/rpo/models/K408_bin.ply";

    readPlyBinary(ply_file, vertices);

    std::cout << "Points in ply file: " << vertices.size() << std::endl;



    // Convert to pcl format
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    cloud->width = vertices.size();
    cloud->height = 1;
    cloud->points.resize(cloud->width * cloud->height);

    for (size_t i = 0; i < vertices.size(); ++i)
    {
        pcl::PointXYZRGB point;

        point.x = vertices[i].x;
        point.y = vertices[i].y;
        point.z = vertices[i].z;

        point.r = vertices[i].r;
        point.g = vertices[i].g;
        point.b = vertices[i].b;

        cloud->points[i] = point;
    }

    std::cout << "Points in pcl " << cloud->points.size() << std::endl;
    


    // Perform Principal Component Analysis
    pcl::PCA<pcl::PointXYZRGB> pca;
    pca.setInputCloud(cloud);

    Eigen::Matrix3f eigenvectors = pca.getEigenVectors();
    Eigen::Vector3f eigenvalues = pca.getEigenValues();

    std::cout << std::setprecision(12) << "Eigenvectors:\n" << eigenvectors << std::endl;
    std::cout << std::setprecision(12) << "Eigenvalues:\n" << eigenvalues.transpose() << std::endl;



    // Align 3D model based on PCA
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

    transformation.block<3, 3>(0, 0) = eigenvectors.transpose();

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);

    transformation.block<3, 1>(0, 3) = -eigenvectors.transpose() * centroid.head<3>();

    pcl::transformPointCloud(*cloud, *transformed_cloud, transformation);





    // Convert transformed cloud to octomap
    rpo::AugmentedOcTree a_tree(0.1);

    octomap::ColorOcTree c_tree(0.1);

    std::unordered_map<octomap::OcTreeKey, int, octomap::OcTreeKey::KeyHash> points_in_voxel;

    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        octomap::point3d point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z + 55.7);

        octomap::OcTreeKey key = a_tree.coordToKey(point, 16);

        rpo::AugmentedOcTreeNode* a_node = a_tree.search(key, 16);

        octomap::ColorOcTreeNode* c_node = c_tree.search(key, 16);

        if (c_node == nullptr)
        {
            a_node = a_tree.updateNode(key, true);

            c_node = c_tree.updateNode(key, true);

            points_in_voxel[key] = 1;

            c_node->setColor(vertices[i].r, vertices[i].g, vertices[i].b);
        }
        else
        {
            int point_already_in_voxel = points_in_voxel[key];

            points_in_voxel[key] += 1;

            octomap::ColorOcTreeNode::Color color = c_node->getColor();

            c_node->setColor(
                static_cast<int>((point_already_in_voxel * color.r + cloud->points[i].r) / (point_already_in_voxel + 1)),
                static_cast<int>((point_already_in_voxel * color.g + cloud->points[i].g) / (point_already_in_voxel + 1)),
                static_cast<int>((point_already_in_voxel * color.b + cloud->points[i].b) / (point_already_in_voxel + 1)));
        }
    }
    
    c_tree.expand();
    a_tree.expand();

    double x1, y1, z1, x2, y2, z2;

    c_tree.getMetricMin(x1, y1, z1);
    c_tree.getMetricMax(x2, y2, z2);

    std::cout << "Model size: " << x1 << " " << x2 << " " << y1 << " " << y2 << " " << z1 << " " << z2 << "\n";

    cutModel(a_tree, c_tree);



    // Expand octomap
    rpo::AugmentedOcTree a_tree2(0.05);

    octomap::ColorOcTree c_tree2(0.05);

    for (octomap::ColorOcTree::leaf_iterator it = c_tree.begin_leafs(), end = c_tree.end_leafs(); it != end; ++it)
    {
        octomap::point3d p = it.getCoordinate();

        octomap::ColorOcTreeNode* c_node2 = nullptr;

        octomap::ColorOcTreeNode* c_node = c_tree.search(it.getKey(), 16);

        rpo::AugmentedOcTreeNode* a_node2 = nullptr;

        c_node2 = c_tree2.updateNode(octomap::point3d(p.x() - 0.025, p.y() - 0.025, p.z() - 0.025), true); c_node2->setColor(c_node->getColor());
        c_node2 = c_tree2.updateNode(octomap::point3d(p.x() - 0.025, p.y() - 0.025, p.z() + 0.025), true); c_node2->setColor(c_node->getColor());
        c_node2 = c_tree2.updateNode(octomap::point3d(p.x() - 0.025, p.y() + 0.025, p.z() - 0.025), true); c_node2->setColor(c_node->getColor());
        c_node2 = c_tree2.updateNode(octomap::point3d(p.x() - 0.025, p.y() + 0.025, p.z() + 0.025), true); c_node2->setColor(c_node->getColor());
        c_node2 = c_tree2.updateNode(octomap::point3d(p.x() + 0.025, p.y() - 0.025, p.z() - 0.025), true); c_node2->setColor(c_node->getColor());
        c_node2 = c_tree2.updateNode(octomap::point3d(p.x() + 0.025, p.y() - 0.025, p.z() + 0.025), true); c_node2->setColor(c_node->getColor());
        c_node2 = c_tree2.updateNode(octomap::point3d(p.x() + 0.025, p.y() + 0.025, p.z() - 0.025), true); c_node2->setColor(c_node->getColor());
        c_node2 = c_tree2.updateNode(octomap::point3d(p.x() + 0.025, p.y() + 0.025, p.z() + 0.025), true); c_node2->setColor(c_node->getColor());

        a_node2 = a_tree2.updateNode(octomap::point3d(p.x() - 0.025, p.y() - 0.025, p.z() - 0.025), true);
        a_node2 = a_tree2.updateNode(octomap::point3d(p.x() - 0.025, p.y() - 0.025, p.z() + 0.025), true);
        a_node2 = a_tree2.updateNode(octomap::point3d(p.x() - 0.025, p.y() + 0.025, p.z() - 0.025), true);
        a_node2 = a_tree2.updateNode(octomap::point3d(p.x() - 0.025, p.y() + 0.025, p.z() + 0.025), true);
        a_node2 = a_tree2.updateNode(octomap::point3d(p.x() + 0.025, p.y() - 0.025, p.z() - 0.025), true);
        a_node2 = a_tree2.updateNode(octomap::point3d(p.x() + 0.025, p.y() - 0.025, p.z() + 0.025), true);
        a_node2 = a_tree2.updateNode(octomap::point3d(p.x() + 0.025, p.y() + 0.025, p.z() - 0.025), true);
        a_node2 = a_tree2.updateNode(octomap::point3d(p.x() + 0.025, p.y() + 0.025, p.z() + 0.025), true);
    }

    a_tree2.expand();
    c_tree2.expand();

    const std::string color_file = "/home/barni/rpo_ws/src/rpo/models/K408_color_mod.ot";
    const std::string aug_file = "/home/barni/rpo_ws/src/rpo/models/K408_augmented_mod.ot";

    c_tree2.write(color_file);
    a_tree2.write(aug_file);

    return 0;
}



void readPlyBinary(const std::string& filename, std::vector<Vertex>& vertices) 
{
    std::ifstream file(filename, std::ios::binary);

    if (!file.is_open())
    {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    std::string line;
    bool is_binary = false;
    size_t vertex_count = 0;

    // Read header
    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        std::string token;
        iss >> token;

        if (token == "format")
        {
            std::string format;
            iss >> format;

            if (format == "binary_little_endian")
            {
                is_binary = true;
            }
            else
            {
                std::cerr << "Unsupported PLY format: " << format << std::endl;
                return;
            }
        }
        else if (token == "element")
        {
            std::string element_type;
            iss >> element_type;

            if (element_type == "vertex")
            {
                iss >> vertex_count;
            }
        }
        else if (token == "end_header")
        {
            break;
        }
    }

    // Read binary
    vertices.resize(vertex_count);

    for (size_t i = 0; i < vertex_count; ++i)
    {
        Vertex v;

        file.read(reinterpret_cast<char*>(&v.x), sizeof(double));
        file.read(reinterpret_cast<char*>(&v.y), sizeof(double));
        file.read(reinterpret_cast<char*>(&v.z), sizeof(double));

        file.read(reinterpret_cast<char*>(&v.r), sizeof(unsigned char));
        file.read(reinterpret_cast<char*>(&v.g), sizeof(unsigned char));
        file.read(reinterpret_cast<char*>(&v.b), sizeof(unsigned char));

        vertices[i] = v;
    }

    file.close();
}



void cutModel(rpo::AugmentedOcTree& a_tree, octomap::ColorOcTree& c_tree)
{
    rpo::KeySet points_to_delete;

    for (rpo::AugmentedOcTree::leaf_iterator it = a_tree.begin_leafs(), end = a_tree.end_leafs(); it != end; ++it)
    {
        rpo::NodePtr node = a_tree.search(it.getKey(), 16);

        if (node != nullptr && it.getCoordinate().z() > 2.2)
        {
            points_to_delete.insert(it.getKey());
        }
    }

    for (const auto& key : points_to_delete)
    {
        a_tree.deleteNode(key, 16);
        c_tree.deleteNode(key, 16);
    }
}

Eigen::Matrix4f createRotationMatrix(float angle_x, float angle_y, float angle_z)
{
    // Convert degrees to radians
    float rad_x = angle_x * M_PI / 180.0f;
    float rad_y = angle_y * M_PI / 180.0f;
    float rad_z = angle_z * M_PI / 180.0f;

    // Rotation matrix around X-axis
    Eigen::Matrix4f rot_x;
    rot_x << 1, 0, 0, 0,
              0, cos(rad_x), -sin(rad_x), 0,
              0, sin(rad_x), cos(rad_x), 0,
              0, 0, 0, 1;

    // Rotation matrix around Y-axis
    Eigen::Matrix4f rot_y;
    rot_y << cos(rad_y), 0, sin(rad_y), 0,
              0, 1, 0, 0,
              -sin(rad_y), 0, cos(rad_y), 0,
              0, 0, 0, 1;

    // Rotation matrix around Z-axis
    Eigen::Matrix4f rot_z;
    rot_z << cos(rad_z), -sin(rad_z), 0, 0,
              sin(rad_z), cos(rad_z), 0, 0,
              0, 0, 1, 0,
              0, 0, 0, 1;

    // Combine rotations (Z * Y * X)
    return rot_z * rot_y * rot_x;
}
