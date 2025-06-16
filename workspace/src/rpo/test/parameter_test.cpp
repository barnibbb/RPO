#include "parameters.h"

#include <iostream>

int main(int argc, char** argv)
{
    if (argc != 2)
    { 
        std::cerr << "Usage: rosrun rpo Parameters <parameter_file>" << std::endl;
        return -1;
    }

    const std::string parameters_file = argv[1];

    rpo::Parameters parameters = rpo::Parameters::loadParameters(parameters_file);

    std::cout 
        << "\nwork folder: "               << parameters.paths.workspace
        << "\nextended model file: "      << parameters.paths.extended_model
        << "\ncolor model file: "          << parameters.paths.color_model
        << "\nlamp voxel model file: "     << parameters.paths.lamp_model
        << "\nirradiance maps folder: "    << parameters.paths.irradiance_maps
        << "\nshort report file: "         << parameters.paths.short_report
        << "\nlong report file: "          << parameters.paths.long_report
        << "\ndepth: "                     << parameters.preprocessing.depth
        << "\nresolution: "                << parameters.preprocessing.resolution
        << "\nsafety radius: "             << parameters.preprocessing.safety_radius
        << "\nlamp height: "               << parameters.lamp.height
        << "\nlamp offset: "               << parameters.lamp.offset
        << "\nlamp power: "                << parameters.lamp.power
        << "\nlamp range: "                << parameters.lamp.range
        << "\ncomputation type: "          << parameters.computation.type
        << "\nexposure limit: "            << parameters.computation.exposure_limit
        << "\ntarget coverage: "           << parameters.computation.target_coverage
        << "\ngrid distance: "             << parameters.computation.grid_distance
        << "\nload maps: "                 << parameters.computation.load_maps
        << "\nsave maps: "                 << parameters.computation.save_maps
        << "\nstore maps: "                << parameters.computation.store_maps
        << "\nsemantic: "                  << parameters.computation.semantic
        << "\nfilter: "                    << parameters.computation.filter    
        << "\nstart number of positions: " << parameters.optimization.start_positions
        << "\nend number of positions: "   << parameters.optimization.end_positions
        << "\nplan element size: "         << parameters.optimization.element_size
        << "\nmaximum generations: "       << parameters.optimization.max_generations
        << "\npopulation size: "           << parameters.optimization.population_size
        << "\nnumber of crossovers: "      << parameters.optimization.num_crossovers
        << "\nnumber of mutations: "       << parameters.optimization.num_mutations
        << "\ncrossover type: "            << parameters.optimization.crossover_type
        << "\nmutation type: "             << parameters.optimization.mutation_type
        << "\ncrossover selection type: "  << parameters.optimization.crossover_selection_type
        << "\nmutation selection type: "   << parameters.optimization.mutation_selection_type
        << "\nsurvival selection type: "   << parameters.optimization.survival_selection_type
        << "\nmutation probability: "      << parameters.optimization.mutation_probability
        << "\nspace mutation parameter: "  << parameters.optimization.space_mutation
        << "\ntime mutation parameter: "   << parameters.optimization.time_mutation
        << "\ndisinfection time: "         << parameters.optimization.disinfection_time
        << "\nincrement: "                 << parameters.optimization.increment
        << "\ncondition: "                 << parameters.optimization.condition
        << "\nverify: "                    << parameters.optimization.verify
        << "\n";
    
    return 0;
}