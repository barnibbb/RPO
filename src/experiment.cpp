#include <string>
#include <iostream>

int main (int argc, char** argv)
{
    const std::string param_files[] = {
        // "/home/barni/rpo_ws/src/rpo/experiments/1a/params_7.txt",               // 1a
        // "/home/barni/rpo_ws/src/rpo/experiments/1a/params_1.txt",               // 1a
        // "/home/barni/rpo_ws/src/rpo/experiments/1a/params_2.txt",               // 1a
        // "/home/barni/rpo_ws/src/rpo/experiments/1a/params_3.txt",               // 1a
        // "/home/barni/rpo_ws/src/rpo/experiments/1a/params_4.txt",               // 1a
        // "/home/barni/rpo_ws/src/rpo/experiments/1a/params_5.txt",               // 1a
        // "/home/barni/rpo_ws/src/rpo/experiments/1a/params_6.txt",               // 1a -- end
        // "/home/barni/rpo_ws/src/rpo/experiments/1b/params_7.txt",               // 1b
        // "/home/barni/rpo_ws/src/rpo/experiments/1b/params_1.txt",               // 1b
        // "/home/barni/rpo_ws/src/rpo/experiments/1b/params_2.txt",               // 1b
        // "/home/barni/rpo_ws/src/rpo/experiments/1b/params_3.txt",               // 1b
        // "/home/barni/rpo_ws/src/rpo/experiments/1b/params_4.txt",               // 1b
        // "/home/barni/rpo_ws/src/rpo/experiments/1b/params_5.txt",               // 1b
        // "/home/barni/rpo_ws/src/rpo/experiments/1b/params_6.txt",               // 1b -- end
        "/home/barni/rpo_ws/src/rpo/experiments/2a/params_30.txt"              // 2a
        // "/home/barni/rpo_ws/src/rpo/experiments/2a/params_100.txt",             // 2a -- end
        // "/home/barni/rpo_ws/src/rpo/experiments/2b/params_infirmary.txt",       // 2b
        // "/home/barni/rpo_ws/src/rpo/experiments/2b/params_office.txt"           // 2b
        // "/home/barni/rpo_ws/src/rpo/experiments/2b/params_cafe.txt",            // 2b -- end
        // "/home/barni/rpo_ws/src/rpo/experiments/2c/params_0.txt",               // 2c
        // "/home/barni/rpo_ws/src/rpo/experiments/2c/params_1.txt"               // 2c
        // "/home/barni/rpo_ws/src/rpo/experiments/2c/params_2.txt",               // 2c*
        // "/home/barni/rpo_ws/src/rpo/experiments/2c/params_3.txt",               // 2c
        // "/home/barni/rpo_ws/src/rpo/experiments/2c/params_4.txt",               // 2c*
        // "/home/barni/rpo_ws/src/rpo/experiments/2c/params_5.txt",               // 2c
        // "/home/barni/rpo_ws/src/rpo/experiments/2c/params_6.txt",               // 2c*
        // "/home/barni/rpo_ws/src/rpo/experiments/2c/params_7.txt",               // 2c
        // "/home/barni/rpo_ws/src/rpo/experiments/2c/params_8.txt",               // 2c*
        // "/home/barni/rpo_ws/src/rpo/experiments/2c/params_9.txt",               // 2c
        // "/home/barni/rpo_ws/src/rpo/experiments/2c/params_10.txt",              // 2c* -- end
        // "/home/barni/rpo_ws/src/rpo/experiments/3/params_general_full.txt",     // 3
        // "/home/barni/rpo_ws/src/rpo/experiments/3/params_general_surface.txt",  // 3
        // "/home/barni/rpo_ws/src/rpo/experiments/3/params_full_general.txt",     // 3
        // "/home/barni/rpo_ws/src/rpo/experiments/3/params_surface_general.txt"   // 3  -- end
    };

    const std::string executable = "rosrun rpo RPO ";           

    for (const auto& param_file : param_files)
    {
        for (int i = 0; i < 1; ++i)
        {
            const std::string command = executable + param_file; 

            system(command.c_str());
        }
    }

    return 0;
}