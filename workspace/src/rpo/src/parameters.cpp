#include "parameters.h"

#include <yaml-cpp/yaml.h>

namespace rpo
{
    Parameters Parameters::loadParameters(const std::string& parameter_file)
    {
        YAML::Node params = YAML::LoadFile(parameter_file);

        Parameters parameters;

        // Paths
        parameters.paths.workspace       = params["paths"]["workspace"].as<std::string>();
        parameters.paths.extended_model = parameters.paths.workspace + params["paths"]["extended_model"].as<std::string>();
        parameters.paths.color_model     = parameters.paths.workspace + params["paths"]["color_model"].as<std::string>();
        parameters.paths.lamp_model      = parameters.paths.workspace + params["paths"]["lamp_model"].as<std::string>();
        parameters.paths.irradiance_maps = parameters.paths.workspace + params["paths"]["irradiance_maps"].as<std::string>();
        parameters.paths.short_report    = parameters.paths.workspace + "/short_report.txt";
        parameters.paths.long_report     = parameters.paths.workspace + "/long_report.txt";

        // Preprocessing
        parameters.preprocessing.depth         = params["preprocessing"]["depth"].as<int>();
        parameters.preprocessing.resolution    = params["preprocessing"]["resolution"].as<double>();
        parameters.preprocessing.safety_radius = params["preprocessing"]["safety_radius"].as<double>();

        // Lamp attributes
        parameters.lamp.height = params["lamp_attributes"]["height"].as<double>();
        parameters.lamp.offset = params["lamp_attributes"]["offset"].as<double>();
        parameters.lamp.power  = params["lamp_attributes"]["power"].as<double>();
        parameters.lamp.range  = params["lamp_attributes"]["range"].as<double>();

        // Dose computation
        parameters.computation.type            = params["dose_computation"]["type"].as<int>();
        parameters.computation.exposure_limit  = params["dose_computation"]["exposure_limit"].as<double>();
        parameters.computation.target_coverage = params["dose_computation"]["target_coverage"].as<double>();
        parameters.computation.grid_distance   = params["dose_computation"]["grid_distance"].as<int>();
        parameters.computation.load_maps       = params["dose_computation"]["load_maps"].as<bool>();
        parameters.computation.save_maps       = params["dose_computation"]["save_maps"].as<bool>();
        parameters.computation.store_maps      = params["dose_computation"]["store_maps"].as<bool>();
        parameters.computation.semantic        = params["dose_computation"]["semantic"].as<bool>();
        parameters.computation.filter          = params["dose_computation"]["filter"].as<bool>();

        // Optimization
        parameters.optimization.start_positions          = params["optimization"]["start_positions"].as<int>();
        parameters.optimization.end_positions            = params["optimization"]["end_positions"].as<int>();
        parameters.optimization.element_size             = params["optimization"]["element_size"].as<int>();
        parameters.optimization.max_generations          = params["optimization"]["max_generations"].as<int>();
        parameters.optimization.population_size          = params["optimization"]["population_size"].as<int>();
        parameters.optimization.num_crossovers           = params["optimization"]["num_crossovers"].as<int>();
        parameters.optimization.num_mutations            = params["optimization"]["num_mutations"].as<int>();
        parameters.optimization.crossover_type           = params["optimization"]["crossover_type"].as<int>();
        parameters.optimization.mutation_type            = params["optimization"]["mutation_type"].as<int>();
        parameters.optimization.crossover_selection_type = params["optimization"]["crossover_selection_type"].as<int>();
        parameters.optimization.mutation_selection_type  = params["optimization"]["mutation_selection_type"].as<int>();
        parameters.optimization.survival_selection_type  = params["optimization"]["survival_selection_type"].as<int>();
        parameters.optimization.mutation_probability     = params["optimization"]["mutation_probability"].as<double>();        
        parameters.optimization.space_mutation           = params["optimization"]["space_mutation"].as<double>();  
        parameters.optimization.time_mutation            = params["optimization"]["time_mutation"].as<double>();  
        parameters.optimization.disinfection_time        = params["optimization"]["disinfection_time"].as<double>();  
        parameters.optimization.increment                = params["optimization"]["increment"].as<double>();  
        parameters.optimization.verify                   = params["optimization"]["verify"].as<bool>();  
        parameters.optimization.condition                = params["optimization"]["condition"].as<bool>();  
        

        return parameters; 
    }
}

