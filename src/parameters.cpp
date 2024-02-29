#include "parameters.h"

#include <iostream>

namespace rpo
{
    bool Parameters::setValues(const std::string& parameters_file)
    {
        try
        {
            readParameters(parameters_file);

            // Paths -------------------------------------------------------------------
            work_folder = getValue("work folder");
            augmented_model_file   = work_folder + getValue("augmented model file");
            color_model_file       = work_folder + getValue("color model file");
            lamp_voxel_model_file  = work_folder + getValue("lamp voxel model file");
            irradiance_maps_folder = work_folder + getValue("irradiance maps folder");
            short_report_file      = work_folder + getValue("short report file");
            long_report_file       = work_folder + getValue("long report file");

            // Preprocessing -----------------------------------------------------------
            depth         = std::stoi(getValue("depth"));
            resolution    = std::stod(getValue("resolution"));
            safety_radius = std::stod(getValue("safety radius"));

            // Lamp attributes ---------------------------------------------------------
            lamp_height = std::stod(getValue("lamp height"));
            lamp_offset = std::stod(getValue("lamp offset"));
            lamp_power  = std::stod(getValue("lamp power"));
            lamp_range  = std::stod(getValue("lamp range"));

            // Computation -------------------------------------------------------------
            computation_type = std::stoi(getValue("computation type"));
            exposure_limit   = std::stod(getValue("exposure limit"));
            target_coverage  = std::stod(getValue("target coverage"));
            grid_distance    = std::stoi(getValue("grid distance"));;
            load_maps  = getValue("load maps")  == "true";
            save_maps  = getValue("save maps")  == "true";
            store_maps = getValue("store maps") == "true";
            semantic   = getValue("semantic")   == "true";

            // Optimization ------------------------------------------------------------
            start_number_of_positions = std::stoi(getValue("start number of positions"));
            end_number_of_positions   = std::stoi(getValue("end number of positions"));
            plan_element_size         = std::stoi(getValue("plan element size"));
            maximum_generations       = std::stoi(getValue("maximum generations"));
            population_size           = std::stoi(getValue("population size"));
            number_of_crossovers      = std::stoi(getValue("number of crossovers"));
            number_of_mutations       = std::stoi(getValue("number of mutations"));
            crossover_type            = std::stoi(getValue("crossover type"));
            mutation_type             = std::stoi(getValue("mutation type"));
            crossover_selection_type  = std::stoi(getValue("crossover selection type"));
            mutation_selection_type   = std::stoi(getValue("mutation selection type"));
            survival_selection_type   = std::stoi(getValue("survival selection type"));
            mutation_probability      = std::stod(getValue("mutation probability"));
            space_mutation_parameter  = std::stod(getValue("space mutation parameter"));
            time_mutation_parameter   = std::stod(getValue("time mutation parameter"));
            disinfection_time         = std::stod(getValue("disinfection time"));
            increment                 = std::stod(getValue("increment"));
            verify    = getValue("verify")    == "true";
            condition = getValue("condition") == "true";

            number_of_positions = start_number_of_positions;
            individual_size = number_of_positions * plan_element_size;
        }
        catch(const std::exception& error)
        {
            std::cerr << error.what() << std::endl;
            return false;
        }

        return true;
    }


    void Parameters::readParameters(const std::string& parameters_file)
    {
        std::ifstream file(parameters_file);

        if (file.is_open())
        {
            std::string line;

            while (getline(file, line))
            {
                if (size_t pos = line.find(':'); pos != std::string::npos)
                {
                    const std::string param_name = line.substr(0, pos);
                    const std::string param_value = line.substr(pos + 2, line.length() - pos - 2);

                    params[param_name] = param_value;
                }
            }
            
            file.close();
        }
        else
        {
            throw std::runtime_error { "Could not read parameter file!" };
        }
    }


    std::string Parameters::getValue(const std::string& parameter_name)
    {
        std::string parameter_value;

        if (params.find(parameter_name) != params.end())
        {
            parameter_value = params[parameter_name];
        }
        else
        {
            std::string error_message = "The " + parameter_name + " parameter must be set!";
            throw std::runtime_error { error_message };
        }

        return parameter_value;
    }
}
