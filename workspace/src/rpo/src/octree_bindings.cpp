#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

#include "extended_octree.h"


// export PYTHONPATH=/home/appuser/workspace/devel/lib:$PYTHONPATH

namespace py = pybind11;

py::list get_voxels_from_tree(const std::string& filename)
{
    std::shared_ptr<rpo::ExtendedOcTree> extended_model = nullptr;

    extended_model.reset(dynamic_cast<rpo::ExtendedOcTree*>(AbstractOcTree::read(filename)));

    if (extended_model == nullptr) throw std::runtime_error("Failed to cast extended octree.");

    py::list result;

    return result;
}

PYBIND11_MODULE(extended_octree_module, m)
{
    m.doc() = "Pybind11 bindings for custom OctoMap";
    m.def("get_voxels", &get_voxels_from_tree, "Load voxels from extended octree file.");
}

