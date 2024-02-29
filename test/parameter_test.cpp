#include <fstream>
#include <iostream>

#include "parameters.h"

int main(int argc, char** argv)
{
    const std::string parameters_file = "/home/barni/rpo_ws/src/rpo/test/test_params.txt";

    rpo::Parameters parameters;

    if (parameters.setValues(parameters_file))
    {
        std::cout 
            << "\nwork folder: "               << parameters.work_folder
            << "\naugmented model file: "      << parameters.augmented_model_file
            << "\ncolor model file: "          << parameters.color_model_file
            << "\nlamp voxel model file: "     << parameters.lamp_voxel_model_file
            << "\nirradiance maps folder: "    << parameters.irradiance_maps_folder
            << "\nshort report file: "         << parameters.short_report_file
            << "\nlong report file: "          << parameters.long_report_file
            << "\ndepth: "                     << parameters.depth
            << "\nresolution: "                << parameters.resolution
            << "\nsafety radius: "             << parameters.safety_radius
            << "\nlamp height: "               << parameters.lamp_height
            << "\nlamp offset: "               << parameters.lamp_offset
            << "\nlamp power: "                << parameters.lamp_power
            << "\nlamp range: "                << parameters.lamp_range
            << "\ncomputation type: "          << parameters.computation_type
            << "\nexposure limit: "            << parameters.exposure_limit
            << "\ntarget coverage: "           << parameters.target_coverage
            << "\ngrid distance: "             << parameters.grid_distance
            << "\nload maps: "                 << parameters.load_maps
            << "\nsave maps: "                 << parameters.save_maps
            << "\nstore maps: "                << parameters.store_maps
            << "\nsemantic: "                  << parameters.semantic
            << "\nstart number of positions: " << parameters.start_number_of_positions
            << "\nend number of positions: "   << parameters.end_number_of_positions
            << "\nplan element size: "         << parameters.plan_element_size
            << "\nmaximum generations: "       << parameters.maximum_generations
            << "\npopulation size: "           << parameters.population_size
            << "\nnumber of crossovers: "      << parameters.number_of_crossovers
            << "\nnumber of mutations: "       << parameters.number_of_mutations
            << "\ncrossover type: "            << parameters.crossover_type
            << "\nmutation type: "             << parameters.mutation_type
            << "\ncrossover selection type: "  << parameters.crossover_selection_type
            << "\nmutation selection type: "   << parameters.mutation_selection_type
            << "\nsurvival selection type: "   << parameters.survival_selection_type
            << "\nmutation probability: "      << parameters.mutation_probability
            << "\nspace mutation parameter: "  << parameters.space_mutation_parameter
            << "\ntime mutation parameter: "   << parameters.time_mutation_parameter
            << "\ndisinfection time: "         << parameters.disinfection_time
            << "\nincrement: "                 << parameters.increment
            << "\ncondition: "                 << parameters.condition
            << "\nverify: "                    << parameters.verify
            << "\n";
    }
    
    return 0;
}