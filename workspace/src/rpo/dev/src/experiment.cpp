#include <string>
#include <iostream>

int main (int argc, char** argv)
{
    const std::string param_files[] = {
        "lab/params_lab_z.yaml 1",
        "office/params_office_z.yaml",
        "infirmary/params_infirmary_z.yaml"
        // "cafe/params_cafe_z.yaml"
    };

    const std::string executable = "rosrun rpo Vertical /home/appuser/data/";           

    for (const auto& param_file : param_files)
    {
        for (int i = 0; i < 9; ++i)
        {
            const std::string command = executable + param_file; 

            system(command.c_str());
        }
    }

    return 0;
}
