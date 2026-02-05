#include <string>
#include <iostream>

int main (int argc, char** argv)
{
    const std::string param_files[] = {
        "lab/params_lab_xy.yaml 1",
        "office/params_office_xy.yaml",
        "infirmary/params_infirmary_xy.yaml",
        "cafe/params_cafe_xy.yaml"
    };

    const std::string executable = "rosrun rpo RPO /home/appuser/data/";           

    for (const auto& param_file : param_files)
    {
        for (int i = 0; i < 10; ++i)
        {
            const std::string command = executable + param_file; 

            system(command.c_str());
        }
    }

    return 0;
}
