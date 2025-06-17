#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include "extended_octree.h"
#include "dose_calculator.h"

namespace py = pybind11;

py::list get_voxels_from_tree(const std::string& filename)
{
    std::shared_ptr<rpo::ExtendedOcTree> extended_model = nullptr;

    extended_model.reset(dynamic_cast<rpo::ExtendedOcTree*>(AbstractOcTree::read(filename)));

    if (extended_model == nullptr) throw std::runtime_error("Failed to cast extended octree.");

    py::list result;

    for (rpo::ExtendedOcTree::leaf_iterator it = extended_model->begin_leafs(), end = extended_model->end_leafs(); it != end; ++it)
    {
        if (extended_model->isNodeOccupied(*it))
        {
            py::dict voxel;
            voxel["x"] = it.getX();
            voxel["y"] = it.getY();
            voxel["z"] = it.getZ();

            octomap::point3d color = it->getColor();

            voxel["r"] = color.x();
            voxel["g"] = color.y();
            voxel["b"] = color.z();

            voxel["type"] = it->getType();

            result.append(voxel);
        }
    }

    return result;
}


py::list get_irradiance_values(const std::string& model_file, const std::string& irradiance_file)
{
    std::shared_ptr<rpo::ExtendedOcTree> extended_model = nullptr;

    extended_model.reset(dynamic_cast<rpo::ExtendedOcTree*>(AbstractOcTree::read(model_file)));

    py::list result;

    std::ifstream file(irradiance_file, std::ios::binary);

    rpo::ExposureMap irradiance_map;

    if (file.is_open())
    {
        octomap::OcTreeKey key;
        float value;

        while (file.read(reinterpret_cast<char*>(&key[0]), sizeof(key[0])))
        {
            py::dict voxel;

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


    for (rpo::ExtendedOcTree::leaf_iterator it = extended_model->begin_leafs(), end = extended_model->end_leafs(); it != end; ++it)
    {
        if (extended_model->isNodeOccupied(*it))
        {
            py::dict voxel;
            voxel["x"] = it.getX();
            voxel["y"] = it.getY();
            voxel["z"] = it.getZ();

            if (irradiance_map.find(it.getKey()) != irradiance_map.end())
            {
                voxel["value"] = irradiance_map[it.getKey()];
            }
            else
            {
                voxel["value"] = 0;
            }

            result.append(voxel);
        }
    }


    return result;
}




PYBIND11_MODULE(extended_octree_module, m)
{
    m.doc() = "Pybind11 bindings for custom OctoMap";
    m.def("get_voxels", &get_voxels_from_tree, "Load voxels from extended octree file.");
    m.def("get_irradiance_values", &get_irradiance_values, "Load irradiance values from submap.");
}

