#include <chrono>
#include <filesystem>

#include "ros_visualizer.h"
#include "plan_generator.h"



int main (int argc, char** argv)
{
    auto start_overall = std::chrono::high_resolution_clock::now();

    const std::string parameters_file = argv[1];
    rpo::Parameters parameters = rpo::Parameters::loadParameters(parameters_file);

    std::filesystem::create_directory(parameters.paths.irradiance_maps);

    std::shared_ptr<ColorOcTree> color_model = nullptr;
    std::shared_ptr<rpo::ExtendedOcTree> extended_model = nullptr;

    std::ifstream file(parameters.paths.color_model);
    color_model.reset(dynamic_cast<ColorOcTree*>(AbstractOcTree::read(file)));
    file.close();

    file.open(parameters.paths.extended_model);
    extended_model.reset(dynamic_cast<rpo::ExtendedOcTree*>(AbstractOcTree::read(file)));
    file.close();

    ros::init(argc, argv, "vertical");
    rpo::ROSVisualizer visualizer(extended_model, color_model, parameters);


    visualizer.cutUnderGround();
    visualizer.computeGroundZone();
    visualizer.computeGridElements();
    visualizer.computeRayTargets();

    auto precomp_time = std::chrono::seconds::zero();

    if (argc == 3) visualizer.filter();

    if (!parameters.computation.load_maps)
    {
        auto start_precomputation = std::chrono::high_resolution_clock::now();
        visualizer.computeIrradianceMaps4();
        auto stop_precomputation = std::chrono::high_resolution_clock::now();
        precomp_time = std::chrono::duration_cast<std::chrono::seconds>(stop_precomputation - start_precomputation);
        std::cout << "Precomputation time: " << precomp_time.count() << std::endl;
    }
    else
    {
        visualizer.loadIrradianceMaps3();
    }



    visualizer.setOptimizationElements2();
    visualizer.showOptimizationElements();


    return 0;

    rpo::PlanGenerator generator(visualizer.getParameters());

    generator.setModelBoundaries(extended_model->getBoundaries());
    generator.setGroundZone(visualizer.getGroundZone());

    visualizer.showElementTypes();


        // Iterative optimization ------------------------------------------------------
    rpo::RadiationPlan previous_best_plan;
    rpo::RadiationPlan best_plan;

    while (generator.getNumPos() <= parameters.optimization.end_positions)
    {
        auto start_iteration = std::chrono::high_resolution_clock::now();

        generator.createInitialPopulation();

        for (unsigned int i = 0; i < parameters.optimization.max_generations; ++i)
        {
            visualizer.compute2(generator.getPopulation(), generator.getIndexOfUnevaluated());

            generator.selectSurvivals();

            if (generator.getBestPlan().second >= parameters.computation.target_coverage) break;

            generator.recombine();

            visualizer.compute2(generator.getPopulation(), generator.getIndexOfUnevaluated());

            generator.mutate();
        }

        visualizer.compute2(generator.getPopulation(), generator.getIndexOfUnevaluated());

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


    // Active indices
    std::vector<double> active_indices = visualizer.getGridIndices(best_plan);

    std::fstream fi(parameters.paths.active_indices, std::ios::out);

    if (fi.is_open())
    {
        for (int i = 0; i < active_indices.size() - 1; ++i) 
        {
            fi << active_indices[i] << " ";
        }
        fi << active_indices[active_indices.size() - 1] << std::endl;

        fi.close();
    }


    // Final verification ----------------------------------------------------------
    double optimized_coverage = best_plan.second;
    double verified_coverage = best_plan.second;

    int final_number_of_positions = best_plan.first.size() / parameters.optimization.element_size;

    if (!parameters.optimization.verify && !parameters.computation.semantic)
    {
        rpo::Score score = visualizer.showResult3(visualizer.getGrid(best_plan));

        verified_coverage = score.general_coverage;
    }



    // Final report ----------------------------------------------------------------
    std::fstream fs(parameters.paths.brief_report, std::ios::out | std::ios::app);

    if (fs.is_open())
    {
        fs << parameters.computation.type << " ";
        fs << optimized_coverage          << " ";
        fs << verified_coverage           << " ";
        fs << final_number_of_positions   << " ";
        fs << precomp_time.count()        << " ";
        fs << duration_overall.count()    << std::endl;
        
        fs.close();
    }

    std::cout << "Optimized coverage: "        << optimized_coverage        << std::endl;
    std::cout << "Verified coverage: "         << verified_coverage         << std::endl;
    std::cout << "Final number of positions: " << final_number_of_positions << std::endl;
    std::cout << "Precomputation time: "       << precomp_time.count()      << std::endl;
    std::cout << "Overall duration: "          << duration_overall.count()  << "\n\n\n";




    return 0;
}


