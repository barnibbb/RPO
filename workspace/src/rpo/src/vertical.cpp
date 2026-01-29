#include <chrono>
#include <filesystem>

#include "ros_visualizer.h"
#include "plan_generator.h"

int main (int argc, char** argv)
{
    auto start_overall = std::chrono::high_resolution_clock::now();

    // Read input parameters -------------------------------------------------------
    if (argc != 2)
    {
        std::cerr << "Usage: rosrun rpo RPO <parameter_file>" << std::endl;
        return -1;
    }

    const std::string parameters_file = argv[1];
    rpo::Parameters parameters = rpo::Parameters::loadParameters(parameters_file);


    std::filesystem::create_directory(parameters.paths.irradiance_maps);


    // Read 3D models --------------------------------------------------------------                                                                                  
    std::shared_ptr<ColorOcTree> color_model = nullptr;
    std::shared_ptr<rpo::ExtendedOcTree> extended_model = nullptr;

    std::ifstream file(parameters.paths.color_model);

    if (file.is_open())
    {
        color_model.reset(dynamic_cast<ColorOcTree*>(AbstractOcTree::read(file)));
        std::cout << "Color octree num leaf nodes: " << color_model->getNumLeafNodes() << std::endl;
        file.close();
    }
    else
    {
        std::cerr << "Could not open color octree file!" << std::endl;
        return -1;
    }

    file.open(parameters.paths.extended_model);

    if (file.is_open())
    {
        extended_model.reset(dynamic_cast<rpo::ExtendedOcTree*>(AbstractOcTree::read(file)));
        std::cout << "Extended octree num leaf nodes: " << extended_model->getNumLeafNodes() << std::endl;
        file.close();
    }        
    else
    {
        std::cerr << "Could not open extended octree file!" << std::endl;
        return -1;
    }


    assert(color_model->getNumLeafNodes() < 1'000);
    assert(autmented_model->getNumLeafNodes() < 1'000);



    // Visualizer initialization ---------------------------------------------------
    ros::init(argc, argv, "rpo");

    rpo::ROSVisualizer visualizer(extended_model, color_model, parameters);
    
    

    // Preprocessing ---------------------------------------------------------------
    visualizer.cutUnderGround();
    visualizer.computeGroundZone();
    visualizer.computeGridElements();
    visualizer.computeRayTargets();

    auto precomp_time = std::chrono::seconds::zero();


    // Cut random elements on lab model
    if (argc == 3 && argv[2] == "1") visualizer.filter();


    // Precomputation
    if (!parameters.computation.load_maps)
    {
        auto start_precomputation = std::chrono::high_resolution_clock::now();
        visualizer.computeIrradianceMaps3();
        auto stop_precomputation = std::chrono::high_resolution_clock::now();
        precomp_time = std::chrono::duration_cast<std::chrono::seconds>(stop_precomputation - start_precomputation);
        std::cout << "Precomputation time: " << precomp_time.count() << std::endl;
    }
    else
    {
        visualizer.loadIrradianceMaps2();
    }
    


    // Set optimization elements ---------------------------------------------------
    visualizer.setOptimizationElements();
    visualizer.showOptimizationElements();


    // Initialize plan generator ---------------------------------------------------
    rpo::PlanGenerator generator(visualizer.getParameters());

    generator.setModelBoundaries(extended_model->getBoundaries());
    generator.setGroundZone(visualizer.getGroundZone());

    visualizer.showElementTypes();
    
    std::cout << "Elements shown!" << std::endl;



    // Iterative optimization ------------------------------------------------------
    rpo::RadiationPlan previous_best_plan;
    rpo::RadiationPlan best_plan;

    while (generator.getNumPos() <= parameters.optimization.end_positions)
    {
        auto start_iteration = std::chrono::high_resolution_clock::now();

        generator.createInitialPopulation();

        for (unsigned int i = 0; i < parameters.optimization.max_generations; ++i)
        {
            visualizer.compute(generator.getPopulation(), generator.getIndexOfUnevaluated());

            generator.selectSurvivals();

            if (generator.getBestPlan().second >= parameters.computation.target_coverage) break;

            generator.recombine();

            visualizer.compute(generator.getPopulation(), generator.getIndexOfUnevaluated());

            generator.mutate();
        }

        visualizer.compute(generator.getPopulation(), generator.getIndexOfUnevaluated());

        best_plan = generator.getBestPlan();

        auto stop_iteration = std::chrono::high_resolution_clock::now();

        auto duration_iteration = std::chrono::duration_cast<std::chrono::seconds>(stop_iteration - start_iteration);



        // Verification ------------------------------------------------------------
        double general_coverage = best_plan.second;
        double object_coverage = -1;
        
        if (parameters.optimization.verify || parameters.computation.semantic)
        {
            rpo::Score score = visualizer.showResult(visualizer.getGrid(best_plan), true);

            general_coverage = score.general_coverage;
            object_coverage = score.object_coverage;
        }

        

        // Reports -----------------------------------------------------------------
        std::cout << std::setprecision(5) << generator.getNumPos() << '\t' << general_coverage << '\t' << object_coverage << '\t' << duration_iteration.count() << '\n';

        // Short report
        std::fstream fs(parameters.paths.short_report, std::ios::out | std::ios::app);
    
        if (fs.is_open())
        {
            fs << generator.getNumPos() << '\t' 
               << general_coverage << '\t' 
               << object_coverage << '\t' 
               << duration_iteration.count() << '\n';

            fs.close();
        }

        // Long report
        std::fstream fl(parameters.paths.long_report, std::ios::out | std::ios::app);
        
        if (fl.is_open())
        {
            fl << generator.getNumPos() << '\t'
               << general_coverage << '\t' 
               << object_coverage << '\t' 
               << duration_iteration.count() << '\t';

            for (const auto& gene : visualizer.getGrid(best_plan).first)
            {
                fl << gene << '\t';
            }

            fl << '\n';

            fl.close();
        }



        // Exit criteria -----------------------------------------------------------
        if (best_plan.second < parameters.computation.target_coverage)
        {
            if (parameters.optimization.condition && generator.getNumPos() > parameters.optimization.start_positions && best_plan.second - previous_best_plan.second <= parameters.optimization.increment)
            {
                best_plan = previous_best_plan;
                std::cout << "Inferior result!" << std::endl;
                break;
            }
            else
            {
                generator.incrementPlans();
                previous_best_plan = best_plan;
            }
        }
        else
        {
            std::cout << "Coverage achieved!" << std::endl;
            break;
        }
    }

    auto stop_overall = std::chrono::high_resolution_clock::now();

    auto duration_overall = std::chrono::duration_cast<std::chrono::seconds>(stop_overall - start_overall);





    return 0;
}
