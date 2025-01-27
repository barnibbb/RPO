#include <string>
#include <iostream>

int main (int argc, char** argv)
{
    const std::string param_files[] = {
        "/2d/params_8.txt"
    };

    const std::string executable = "rosrun rpo RPO /home/barni/rpo_ws/src/rpo/experiments/";           

    for (const auto& param_file : param_files)
    {
        for (int i = 0; i < 3; ++i)
        {
            const std::string command = executable + param_file; 

            system(command.c_str());
        }
    }

    return 0;
}
