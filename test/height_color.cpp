#include <iostream>
#include <string>
#include <cmath>
#include <memory>
#include <array>
#include <algorithm>

#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

using namespace octomap;

const std::array<std::array<float, 3>, 6> colormap = {{
    {0.0f, 0.0f, 1.0f}, // Blue
    {0.0f, 1.0f, 0.0f}, // Green
    {1.0f, 1.0f, 0.0f}, // Yellow
    {1.0f, 0.5f, 0.0f}, // Orange
    {1.0f, 0.0f, 0.0f}, // Red
    {0.5f, 0.0f, 0.5f}  // Purple
}};


void height2Color1(double z, double min_z, double max_z, uint8_t& r, uint8_t& g, uint8_t& b);
void height2Color2(double z, double min_z, double max_z, uint8_t& r, uint8_t& g, uint8_t& b);
void height2Color3(double z, double min_z, double max_z, uint8_t& r, uint8_t& g, uint8_t& b);
void height2Color4(double z, double min_z, double max_z, uint8_t& r, uint8_t& g, uint8_t& b);


int main (int argc, char** argv)
{
    const std::string input_file = "/home/barni/rpo_ws/src/rpo/models/base/cafe_color.ot";
    const std::string output_file = "/home/barni/rpo_ws/src/rpo/experiments/models_figs/models/base/cafe_height_g.ot";

    std::unique_ptr<ColorOcTree> color_tree = nullptr;

    color_tree.reset(dynamic_cast<ColorOcTree*>(AbstractOcTree::read(input_file)));

    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::min();

    for (ColorOcTree::leaf_iterator it = color_tree->begin_leafs(), end = color_tree->end_leafs(); it != end; ++it)
    {
        double z = it.getZ();
        if (z < min_z) min_z = z;
        if (z > max_z) max_z = z;
    }

    
    for (ColorOcTree::leaf_iterator it = color_tree->begin_leafs(), end = color_tree->end_leafs(); it != end; ++it) 
    {
        double z = it.getZ();
        uint8_t r, g, b;
        height2Color4(z, min_z, max_z, r, g, b);
        color_tree->setNodeColor(it.getKey(), r, g, b);
    }

    color_tree->write(output_file);

    ros::init(argc, argv, "h2c");

    ros::NodeHandle node_handle;
    ros::Publisher model_publisher = node_handle.advertise<octomap_msgs::Octomap>("/height_coloring", 10);

    octomap_msgs::Octomap message;

    std::stringstream data_stream;
    color_tree->writeData(data_stream);
    std::string datastring = data_stream.str();

    message.data = std::vector<int8_t>(datastring.begin(), datastring.end());
    message.resolution = color_tree->getResolution();
    message.id = color_tree->getTreeType();
    message.binary = false;
    message.header.frame_id = "map";
    message.header.stamp = ros::Time();

    while(ros::ok())
    {
        model_publisher.publish(message);
    }


    return 0;
}







void height2Color1(double z, double min_z, double max_z, uint8_t& r, uint8_t& g, uint8_t& b)
{
    double ratio = (z - min_z) / (max_z - min_z);

    if (ratio < 0.0) ratio = 0.0;
    if (ratio > 1.0) ratio = 1.0;

    if (ratio < 0.25)
    {
        r = 0;
        g = static_cast<uint8_t>(255 * (ratio / 0.25));
        b = 255;
    }
    else if (ratio < 0.5)
    {
        r = 0;
        g = 255;
        b = static_cast<uint8_t>(255 * (1 - (ratio - 0.25) / 0.25));
    }
    else if (ratio < 0.75)
    {
        r = static_cast<uint8_t>(255 * ((ratio - 0.5) / 0.25));
        g = 255;
        b = 0;
    }
    else
    {
        r = 255;
        g = static_cast<uint8_t>(255 * (1 - (ratio - 0.75) / 0.25));
        b = 0;
    }
}


void height2Color2(double z, double min_z, double max_z, uint8_t& r, uint8_t& g, uint8_t& b)
{
    double ratio = (z - min_z) / (max_z - min_z);

    if (ratio < 0.0) ratio = 0.0;
    if (ratio > 1.0) ratio = 1.0;

    r = static_cast<uint8_t>(255 * ratio);
    g = static_cast<uint8_t>(255 * (1.0f - fabs(0.5f - ratio) * 2.0f));
    b = static_cast<uint8_t>(255 * (1.0f - ratio));
}


void height2Color3(double z, double min_z, double max_z, uint8_t& r, uint8_t& g, uint8_t& b)
{
    double bin_size = (max_z - min_z) / 6.0;

    int bin_index = static_cast<int>((z - min_z) / bin_size);

    if (bin_index < 0) bin_index = 0;
    if (bin_index > 5) bin_index = 5;

    r = static_cast<uint8_t>(255 * colormap[bin_index][0]);
    g = static_cast<uint8_t>(255 * colormap[bin_index][1]);
    b = static_cast<uint8_t>(255 * colormap[bin_index][2]);
}


void height2Color4(double z, double min_z, double max_z, uint8_t& r, uint8_t& g, uint8_t& b)
{
    double ratio = (z - min_z) / (max_z - min_z);

    ratio = std::clamp(ratio, 0.0, 0.8);

    // if (ratio < 0.0) ratio = 0.0;
    // if (ratio > 1.0) ratio = 1.0;

    //r = 255;
    r = static_cast<uint8_t>(255 * (1.0 - ratio));
    g = static_cast<uint8_t>(255 * (1.0 - ratio));
    b = static_cast<uint8_t>(255 * (1.0 - ratio));
    //b = 255;
}

