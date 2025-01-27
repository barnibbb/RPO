#include <string>
#include <iostream>
#include <fstream>

#include <boost/algorithm/string.hpp>

#include "augmented_octree.h"

using namespace octomap;
using ExposureMap = std::unordered_map<OcTreeKey, float, OcTreeKey::KeyHash>;



ExposureMap loadBinary(const std::string& filename);
void writeBinary(const std::string& filename, const ExposureMap& map);
ExposureMap loadIrradianceMap(const std::string& input_file);

int main (int argc, char** argv)
{
    std::string input_file = "/home/barni/rpo_ws/src/rpo/experiments/test/irradiance_maps/32740_32784.txt";

    ExposureMap im = loadIrradianceMap(input_file);

    std::cout << im.size() << std::endl;

    std::string output_file = "/home/barni/rpo_ws/src/rpo/experiments/test/32740_32784.bin";

    writeBinary(output_file, im);

    im = loadBinary(output_file);

    OcTreeKey key(32841, 32832, 32817);

    std::cout << im[key] << std::endl;

    return 0;
}



ExposureMap loadBinary(const std::string& filename)
{
    std::ifstream in_file(filename, std::ios::binary);

    ExposureMap im;

    if (in_file.is_open())
    {
        OcTreeKey key;
        float value;

        while (in_file.read(reinterpret_cast<char*>(&key[0]), sizeof(key[0])))
        {
            in_file.read(reinterpret_cast<char*>(&key[1]), sizeof(key[1]));
            in_file.read(reinterpret_cast<char*>(&key[2]), sizeof(key[2]));
            in_file.read(reinterpret_cast<char*>(&value),  sizeof(value));

            im[key] = value;   
        }

        in_file.close();
    }

    return im;
}



void writeBinary(const std::string& filename, const ExposureMap& map)
{
    std::ofstream out_file(filename, std::ios::binary);

    int c = 0;

    if (out_file.is_open())
    {
        for (const auto& element : map)
        {
            if (element.second > 0)
            {   
                ++c;

                out_file.write(reinterpret_cast<const char*>(&element.first[0]), sizeof(element.first[0]));
                out_file.write(reinterpret_cast<const char*>(&element.first[1]), sizeof(element.first[1]));
                out_file.write(reinterpret_cast<const char*>(&element.first[2]), sizeof(element.first[2]));
                out_file.write(reinterpret_cast<const char*>(&element.second),   sizeof(element.second));
            }
        }

        out_file.close();
    }

    std::cout << c << std::endl;
}




ExposureMap loadIrradianceMap(const std::string& input_file)
{
    ExposureMap irradiance_map;

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

            irradiance_map[key] = std::stof(data[3]);
        }

        file.close();
    }
    else
    {
        std::cerr << "Could not open input file for loading irradiance map!" << std::endl;
    }

    return irradiance_map;
}
