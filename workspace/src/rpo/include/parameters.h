#pragma once
#include <string>

namespace rpo
{
    struct PathParams
    {
        std::string workspace;
        std::string extended_model;
        std::string color_model;
        std::string lamp_model;
        std::string irradiance_maps;
        std::string short_report;
        std::string long_report;
        std::string grid_indices;
    };

    struct PreprocessingParams
    {
        int depth = 16;
        double resolution = 0.05;
        double safety_radius = 0.3;
        double ground_level;
    };

    struct LampParams
    {
        double height = 1.2;
        double offset = 0.37;
        double power = 80;
        double range = 20;
        double center;
        double top;
        int lower_z = 0;
        int upper_z = 5;
    };

    struct DoseComputationParams
    {
        int type = 8;
        double exposure_limit = 280;
        double target_coverage = 0.8;
        int grid_distance = 10;
        bool load_maps = false;
        bool save_maps = true;
        bool store_maps = true;
        bool semantic = false;
        bool filter = false;
    };

    struct OptimizationParams
    {
        int start_positions = 1;
        int end_positions = 1000;
        int element_size = 3;
        int individual_size;
        int max_generations = 10;
        int population_size = 30;
        int num_crossovers = 15;
        int num_mutations = 15;
        int crossover_type = 1;
        int mutation_type = 2;
        int crossover_selection_type = 1;
        int mutation_selection_type = 1;
        int survival_selection_type = 4;
        double mutation_probability = 0.5;
        double space_mutation = 0.4;
        double time_mutation = 0.4;
        double disinfection_time = 1800;
        double increment = 0.00001;
        bool condition = true;
        bool verify = false;
        int num_positions;
    };

    struct Parameters
    {
        PathParams paths;
        PreprocessingParams preprocessing;
        LampParams lamp;
        DoseComputationParams computation;
        OptimizationParams optimization;

        static Parameters loadParameters(const std::string& parameter_file);
    };
}
