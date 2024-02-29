#include <octomap/OcTree.h>
#include <octomap/ColorOcTree.h>

#include <boost/algorithm/string.hpp>

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <memory>
#include <unordered_map>
#include <unordered_set>

using Point = octomap::point3d;
using KeySet = std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash>;

struct Face { int first, second, third; };

std::unordered_map<int, std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash>> elements_of_faces; 

double computeArea(const Point& A, const Point& B, const Point& C);
double computeArea2(const Point& A, const Point& B, const Point& C);

int main (int argc, char** argv)
{
    const std::string input_file = "/home/barni/Downloads/remeshed_hospital_room_full_35k.ply";
    const std::string output_file = "/home/barni/Downloads/remeshed_hospital_room_full_35k.ot";

    std::fstream f(input_file, std::ios::in);

    std::string line;

    std::vector<Point> points;

    std::vector<Face> faces;

    int line_counter = 0;

    double res = 0.05;

    double xi = 10'000, yi = 10'000, zi = 10'000;
    double xa = -10'000, ya = -10'000, za = -10'000;

    if (f.is_open())
    {
        while (std::getline(f, line))
        {
            ++line_counter;

            if (line_counter > 10)
            {
                std::vector<std::string> data;

                boost::split(data, line, boost::is_any_of(" "));

                if (data.size() == 4)
                {
                    Point point(std::stod(data[0]), std::stod(data[1]), std::stod(data[2]));

                    if (point.x() < xi) xi = point.x();
                    if (point.y() < yi) yi = point.y();
                    if (point.z() < zi) zi = point.z();

                    if (point.x() > xa) xa = point.x();
                    if (point.y() > ya) ya = point.y();
                    if (point.z() > za) za = point.z();

                    points.push_back(point);
                }

                if (data.size() == 6)
                {
                    Face face;
                    
                    face.first = std::stoi(data[1]);
                    face.second = std::stoi(data[2]);
                    face.third = std::stoi(data[3]);

                    faces.push_back(face);
                }
            }
        }

        f.close();
    }

    std::unique_ptr<octomap::ColorOcTree> tree = std::make_unique<octomap::ColorOcTree>(res);

    double sum_area = 0;
    double sum_area2 = 0;

    int nan_nan = 0;

    int counter = 0;

    for (const auto& face : faces)
    {
        Point p1 = points[face.first];
        Point p2 = points[face.second];
        Point p3 = points[face.third];

        double area = computeArea(p1, p2, p3);
        double area2 = computeArea2(p1, p2, p3);

        if (std::isnan(area))
        {
            ++nan_nan;
        }
        else
        {
            sum_area += area;
            sum_area2 += area2;
        }

        double x_min = std::min(p1.x(), std::min(p2.x(), p3.x()));
        double y_min = std::min(p1.y(), std::min(p2.y(), p3.y()));
        double z_min = std::min(p1.z(), std::min(p2.z(), p3.z()));
        double x_max = std::max(p1.x(), std::max(p2.x(), p3.x()));
        double y_max = std::max(p1.y(), std::max(p2.y(), p3.y()));
        double z_max = std::max(p1.z(), std::max(p2.z(), p3.z()));

        double x = x_min;
        double y = y_min;
        double z = z_min;

        while (x <= x_max)
        {
            while (y <= y_max)
            {
                while (z <= z_max)
                {
                    Point p(x, y, z);

                    double area_1 = computeArea(p, p2, p3);
                    double area_2 = computeArea(p1, p, p3);
                    double area_3 = computeArea(p1, p2, p);

                    if (std::abs((area_1 + area_2 + area_3) - area) < 0.1 * area)
                    {
                        if (octomap::ColorOcTreeNode* node { tree->search(p) }; node == nullptr)
                        {
                            octomap::OcTreeKey key = tree->coordToKey(p);

                            elements_of_faces[counter].insert(key);

                            node = tree->updateNode(p, true);
                        }
                    }

                    z += res;
                }

                z = z_min;

                y += res;
            }

            y = y_min;

            x += res;
        }

        ++counter;
    }

    for (const auto& a : elements_of_faces)
    {
        std::cout << a.first << '\t' << a.second.size() << '\n';
    }

    tree->expand();

    for (octomap::ColorOcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it)
    {
        octomap::ColorOcTreeNode* node = tree->search(it.getKey(), it.getDepth() - 1);
        tree->expandNode(node);
        tree->pruneNode(node);
    }

    tree->expand();

    std::cout << "Before: " << tree->getNumLeafNodes() << std::endl;

    // Smooth model ---

    /*
    KeySet points_to_add;

    const std::vector<octomap::point3d> steps { 
        { 2 * res, 0, 0 }, { -2 * res, 0, 0 }, 
        { 0, 2 * res, 0 }, { 0, -2 * res, 0 }, 
        { 0, 0, 2 * res }, { 0, 0, -2 * res } }; 

    for (octomap::ColorOcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it)
    {
        octomap::point3d point = tree->keyToCoord(it.getKey(), 15);

        for (int q = 0; q < steps.size(); ++q)
        {
            octomap::point3d neighbor_point = point + steps[q];

            octomap::ColorOcTreeNode* neighbor_node = tree->search(neighbor_point, 15);

            if (neighbor_node == nullptr)
            {
                octomap::point3d next_point = neighbor_point + steps[q];

                octomap::ColorOcTreeNode* next_node = tree->search(next_point, 15);

                if (next_node != nullptr)
                {
                    points_to_add.insert(tree->coordToKey(neighbor_point, 15));
                }
            }
        }
    }

    for (const auto& key : points_to_add)
    {
        tree->updateNode(key, true);

        octomap::ColorOcTreeNode* node = tree->search(key, 15);
        tree->expandNode(node);
        tree->pruneNode(node);
    }
    */


    // Smooth model ---

    tree->expand();

    std::cout << "After: " << tree->getNumLeafNodes() << std::endl;

    f.open(output_file, std::ios::out);

    if (f.is_open())
    {
        tree->write(f);

        f.close();
    }

    std::cout << "Points: "    << points.size() << "\n"
              << "Faces: "     << faces.size() << "\n"
              << "Sum area: "  << sum_area << "\n"
              << "Sum area2: " << sum_area2 << "\n"
              << "Tree size: " << tree->getNumLeafNodes() << "\n"
              << "Nan: "       << nan_nan << "\n";

    return 0;
}


double computeArea(const Point& A, const Point& B, const Point& C)
{
    Point AB = B - A;
    Point AC = C - A;
    Point BC = C - B;

    double cos_alpha = AB.dot(AC) / (AB.norm() * AC.norm());

    double area = 0.5 * AB.norm() * AC.norm() * std::sqrt(1 - std::pow(cos_alpha, 2));

    return area;
}

double computeArea2(const Point& A, const Point& B, const Point& C)
{
    double AB = (B - A).norm();
    double AC = (C - A).norm();
    double BC = (C - B).norm();

    double s = (AB + AC + BC) / 2;

    double area = std::sqrt(s * (s - AB) * (s - AC) * (s - BC));

    return area;
}


