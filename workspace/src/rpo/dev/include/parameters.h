#pragma once
#include <string>
#include <unordered_map>
#include <fstream>
#include <limits>

namespace rpo
{
    using Params = std::unordered_map<std::string, std::string>;

    struct Parameters
    {
        bool setValues(const std::string& parameters_file);
        void readParameters(const std::string& parameters_file);
        std::string getValue(const std::string& parameter_name);

        Params params;

        // Paths -------------------------------------------------------------------
        std::string work_folder;
        std::string extended_model_file;
        std::string color_model_file;
        std::string lamp_voxel_model_file;
        std::string irradiance_maps_folder;
        std::string short_report_file;
        std::string long_report_file;

        // Preprocessing -----------------------------------------------------------
        int depth = 16;
        double resolution = 0.05;
        double safety_radius = 0.3;
        double ground_level;

        // Lamp attributes ---------------------------------------------------------
        double lamp_height = 1.2;
        double lamp_offset = 0.37;
        double lamp_power = 80;
        double lamp_range = 20;
        double lamp_center;
        double lamp_top;

        // Computation -------------------------------------------------------------
        int computation_type = 7;
        double exposure_limit = 280;
        double target_coverage = 0.8;
        int grid_distance = 10;
        bool load_maps = false;
        bool save_maps = false;
        bool store_maps = true;
        bool semantic = false;

        // Optimization ------------------------------------------------------------
        int start_number_of_positions = 1;
        int end_number_of_positions = 10000;
        int number_of_positions;
        int plan_element_size = 3;
        int individual_size;
        int maximum_generations = 10;
        int population_size = 30;
        int number_of_crossovers = 15;
        int number_of_mutations = 15;
        int crossover_type = 1;
        int mutation_type = 2;
        int crossover_selection_type = 1;
        int mutation_selection_type = 1;
        int survival_selection_type = 4;
        double mutation_probability = 0.5;
        double space_mutation_parameter = 0.4;
        double time_mutation_parameter = 0.4;
        double disinfection_time = 7200;
        double increment = 0.00001;
        bool condition = true;
        bool verify = false;
        bool filter = true;
    };
}
