#include "plan_generator.h"

namespace rpo
{
    PlanGenerator::PlanGenerator(const Parameters& parameters)
    {   
        m_parameters = parameters;
    
        std::random_device seeder;

        const auto seed = seeder.entropy() ? seeder() : std::time(nullptr);

        m_engine = std::make_unique<std::mt19937>(static_cast<std::mt19937::result_type>(seed));
    }



    void PlanGenerator::setModelBoundaries(const std::array<double, 6>& model_boundaries)
    {
        m_model_boundaries = model_boundaries;
    }



    void PlanGenerator::setGroundZone(const std::multimap<double, double>& ground_zone)
    {
        m_ground_zone = ground_zone;
    }



    // TODO: understand
    bool PlanGenerator::isElementOfGroundZone(double x, double y) const
    {
        const double factor = 2 / m_parameters.resolution;

        const double x_factor = factor * x;
        const double y_factor = factor * y;

        double x_discrete = std::round(x_factor);
        double y_discrete = std::round(y_factor);

        if (static_cast<int>(x_discrete) % 2 == 0)
        {
            if (x_factor < x_discrete)
            {
                x_discrete -= 1;
            }
            else
            {
                x_discrete += 1;
            }
        }

        if (static_cast<int>(y_discrete) % 2 == 0)
        {
            if (y_factor < y_discrete)
            {
                y_discrete -= 1;
            }
            else
            {
                y_discrete += 1;
            }
        }

        double x_search = x_discrete / factor;
        double y_search = y_discrete / factor;

        auto it = m_ground_zone.equal_range(x_search);

        if (it.first->first == x_search)
        {
            for (auto itr = it.first; itr != it.second; ++itr)
            {
                if (itr->second == y_search)
                {
                    return true;
                }
            }
        }

        return false;
    }



    int PlanGenerator::getNumPos() const
    {
        return m_parameters.number_of_positions;
    }



    std::vector<RadiationPlan>& PlanGenerator::getPopulation()
    {
        return m_population;
    }



    IndexVector& PlanGenerator::getIndexOfUnevaluated()
    {
        return m_unevaluated_individuals;
    }



    RadiationPlan PlanGenerator::getBestPlan()
    {
        sort(m_population, false);

        return m_population[0];
    }



    void PlanGenerator::createInitialPopulation()
    {
        if (m_population.size() == 0)
        {
            for (size_t i = 0; i < 2 * m_parameters.population_size; ++i)
            {
                std::vector<double> individual(m_parameters.individual_size);

                for (size_t j = 0; j < m_parameters.individual_size; j += m_parameters.plan_element_size)
                {
                    addNewPosition(individual, j, false);
                }

                normalizeRadiationTime(individual, false);

                addNewElement(individual);
            }
        }
    }



    void PlanGenerator::addNewPosition(std::vector<double>& individual, int index, bool extend) const
    {
        // Insert new position if neccessary
        if (extend)
        {
            normalizeRadiationTime(individual, true);

            const int size = individual.size();

            individual.resize(size + m_parameters.plan_element_size);

            if (index < size)
            {
                for (int i = size - 1; i >= index; --i)
                {
                    individual[i + m_parameters.plan_element_size] = individual[i];
                }
            }
        }

        static std::uniform_real_distribution distribution_x(m_model_boundaries[0], m_model_boundaries[1]);
        static std::uniform_real_distribution distribution_y(m_model_boundaries[2], m_model_boundaries[3]);

        double base_time = m_parameters.disinfection_time / static_cast<double>(m_parameters.number_of_positions);

        std::normal_distribution<double> distribution_t(base_time, 0.1 * base_time);

        bool good_position = false;

        int counter = 0;

        while(!good_position)
        {
            const double x = distribution_x(*m_engine);
            const double y = distribution_y(*m_engine);

            if(isElementOfGroundZone(x, y))
            {
                good_position = true;

                if (extend)
                {
                    for (int i = 0; i < individual.size(); i += 3)
                    {
                        if (i != index)
                        {
                            if (std::abs(x - individual[i]) < 0.15 || std::abs(y - individual[i + 1]) < 0.15)
                            {
                                good_position = false;
                            }
                        }
                    }
                }
                else
                {
                    for (int i = 0; i < index; i += 3)
                    {
                        if (std::abs(x - individual[i]) < 0.15 || std::abs(y - individual[i + 1]) < 0.15)
                        {
                            good_position = false;
                        }
                    }
                }

                if (good_position || counter > 20)
                {
                    individual[index] = x;
                    individual[index + 1] = y;
                    break;
                }

                ++counter;
            }
        }

        individual[index + 2] = distribution_t(*m_engine);
    }



    void PlanGenerator::normalizeRadiationTime(std::vector<double>& individual, bool extend) const
    {
        const int element_size = m_parameters.plan_element_size;

        double full_time = 0;

        for (size_t i = 0; i < individual.size(); i += element_size)
        {
            full_time += individual[i + 2];
        }

        for (size_t i = 0; i < individual.size(); i += element_size)
        {
            if (extend)
            {
                individual[i + 2] *= m_parameters.disinfection_time / (full_time * (individual.size() + element_size) / individual.size());
            }
            else
            {
                individual[i + 2] *= m_parameters.disinfection_time / full_time;
            }
        }
    }



    void PlanGenerator::addNewElement(const std::vector<double>& individual)
    {
        m_population.push_back(std::make_pair<std::vector<double>, double>(static_cast<std::vector<double>>(individual), 0));

        m_unevaluated_individuals.push_back(m_population.size() - 1);
    }



    void PlanGenerator::incrementPlans()
    {
        // TODO: might not make any difference
        m_unevaluated_individuals.erase(m_unevaluated_individuals.begin(), m_unevaluated_individuals.end());

        m_parameters.number_of_positions += 1;
        m_parameters.individual_size += m_parameters.plan_element_size;

        for (size_t i = 0; i < m_population.size(); ++i)
        {
            std::uniform_int_distribution uniform_distribution(0, static_cast<int>(m_population[i].first.size() / m_parameters.plan_element_size));

            int index = m_parameters.plan_element_size * uniform_distribution(*m_engine);

            addNewPosition(m_population[i].first, index, true);

            normalizeRadiationTime(m_population[i].first, false);

            m_unevaluated_individuals.push_back(i);
        }
    }



    void PlanGenerator::selectSurvivals()
    {
        const IndexVector survivals = select(m_parameters.survival_selection_type, m_parameters.population_size);

        std::vector<RadiationPlan> old_population = m_population;

        if (m_population.size() > 0)
        {
            m_population.erase(m_population.begin(), m_population.end());
        }

        for (const auto index : survivals)
        {
            m_population.push_back(old_population[index]);   
        }

        old_population.erase(old_population.begin(), old_population.end());
    }



    IndexVector PlanGenerator::select(int selection_type, int number_of_selections)
    {
        sort(m_population, false);

        switch(selection_type)
        {
        case 1:
            return applyFitnessProportionateSelection(number_of_selections);
        case 2:
            return applyRandomSelection(number_of_selections);
        case 3:
            return applyTournamentSelection(number_of_selections);
        case 4:
            return applyTruncationSelection(number_of_selections);
        default:
            return applyFitnessProportionateSelection(number_of_selections);
        }
    }



    IndexVector PlanGenerator::applyFitnessProportionateSelection(int number_of_selections) const
    {
        IndexVector index_vector(number_of_selections);

        std::vector<double> roulette_wheel(m_population.size());

        roulette_wheel[0] = m_population[0].second;

        for (size_t i = 1; i < roulette_wheel.size(); ++i)
        {
            roulette_wheel[i] = roulette_wheel[i - 1] + m_population[i].second;
        }

        std::uniform_real_distribution uniform_distribution(0.0, roulette_wheel[roulette_wheel.size() - 1]);

        for (size_t i = 0; i < number_of_selections; ++i)
        {
            double random_value = uniform_distribution(*m_engine);

            for (size_t j = 0; j < roulette_wheel.size(); ++j)
            {
                if (random_value <= roulette_wheel[j])
                {
                    index_vector[i] = j;
                    break;
                }
            }
        }

        return index_vector;
    }



    IndexVector PlanGenerator::applyRandomSelection(int number_of_selections) const
    {
        IndexVector index_vector(number_of_selections);

        std::uniform_int_distribution uniform_distribution(0, static_cast<int>(m_population.size()));

        for (size_t i = 0; i < number_of_selections; ++i)
        {
            index_vector[i] = uniform_distribution(*m_engine);
        }

        return index_vector;
    }  



    IndexVector PlanGenerator::applyTournamentSelection(int number_of_selections) const
    {
        IndexVector index_vector(number_of_selections);

        const int group_size = m_population.size() / 4;

        std::uniform_int_distribution uniform_distribution(0, static_cast<int>(m_population.size()));

        for (size_t i = 0; i < number_of_selections; ++i)
        {
            int index_1 = uniform_distribution(*m_engine);

            for (size_t j = 0; j < group_size - 1; ++j)
            {
                int index_2 = uniform_distribution(*m_engine);

                if (index_2 < index_1)
                {
                    index_1 = index_2;
                }
            }

            index_vector[i] = index_1;
        }

        return index_vector;
    }



    IndexVector PlanGenerator::applyTruncationSelection(int number_of_selections) const
    {
        IndexVector index_vector(number_of_selections);

        for (size_t i = 0; i < number_of_selections; ++i)
        {
            index_vector[i] = i;
        }

        return index_vector;
    }



    void PlanGenerator::recombine()
    {
        m_parents = select(m_parameters.crossover_selection_type, 2 * m_parameters.number_of_crossovers);

        switch(m_parameters.crossover_type)
        {
        case 1:
            applyUniformCrossover();
            break;
        case 2:
            applyTwoPointCrossover();
            break;
        default:
            applyUniformCrossover();
            break;
        }

        m_parents.erase(m_parents.begin(), m_parents.end());
    }



    void PlanGenerator::applyUniformCrossover()
    {
        std::uniform_real_distribution distribution(0.0, 1.0);

        for (size_t i = 0; i < m_parents.size(); i += 2)
        {
            std::vector<double> offspring_1(m_parameters.individual_size);
            std::vector<double> offspring_2(m_parameters.individual_size);

            for (size_t j = 0; j < m_parameters.individual_size; ++j)
            {
                if (distribution(*m_engine) < 0.5)
                {
                    offspring_1[j] = m_population[i].first[j];
                    offspring_2[j] = m_population[i + 1].first[j];
                }
                else
                {
                    offspring_1[j] = m_population[i + 1].first[j];
                    offspring_2[j] = m_population[i].first[j];
                }
            }

            normalizeRadiationTime(offspring_1, false);
            normalizeRadiationTime(offspring_2, false);

            addNewElement(offspring_1);
            addNewElement(offspring_2);
        }
    }



    void PlanGenerator::applyTwoPointCrossover()
    {
        for (size_t i = 0; i < m_parents.size(); i += 2)
        {
            std::vector<double> offspring_1(m_parameters.individual_size);
            std::vector<double> offspring_2(m_parameters.individual_size);

            std::uniform_int_distribution distribution_first(0, m_parameters.individual_size - 1);

            const int first = distribution_first(*m_engine);

            std::uniform_int_distribution distribution_last(first, m_parameters.individual_size - 1);

            const int last = distribution_last(*m_engine);

            for (size_t j = 0; j < m_parameters.individual_size; ++j)
            {
                if (j < first || j > last)
                {
                    offspring_1[j] = m_population[i].first[j];
                    offspring_2[j] = m_population[i + 1].first[j];
                }
                else
                {
                    offspring_1[j] = m_population[i + 1].first[j];
                    offspring_2[j] = m_population[i].first[j];
                }
            }

            normalizeRadiationTime(offspring_1, false);
            normalizeRadiationTime(offspring_2, false);

            addNewElement(offspring_1);
            addNewElement(offspring_2);
        }
    }



    void PlanGenerator::mutate()
    {
        m_premutants = select(m_parameters.mutation_selection_type, m_parameters.number_of_mutations);

        std::uniform_real_distribution distribution(0.0, 1.0);

        const double gene_mutation_probability = m_parameters.mutation_probability;

        double deviation_x = m_parameters.space_mutation_parameter * (m_model_boundaries[1] - m_model_boundaries[0]);

        double deviation_y = m_parameters.space_mutation_parameter * (m_model_boundaries[3] - m_model_boundaries[2]);

        double deviation_t = m_parameters.time_mutation_parameter * m_parameters.disinfection_time / static_cast<double>(m_parameters.number_of_positions);

        for (size_t i = 0; i < m_premutants.size(); ++i)
        {
            std::vector<double> mutant = m_population[i].first;

            for (size_t j = 0; j < m_parameters.individual_size; j += m_parameters.plan_element_size)
            {
                if (distribution(*m_engine) < gene_mutation_probability)
                {
                    double old_gene = mutant[j];
                    mutateGene(mutant[j], deviation_x);
                    if (std::abs(mutant[j] - old_gene) < m_parameters.resolution)
                    {
                        mutant[j] = old_gene;
                    }
                }
                
                if (distribution(*m_engine) < gene_mutation_probability)
                {
                    double old_gene = mutant[j + 1];
                    mutateGene(mutant[j + 1], deviation_y);
                    if (std::abs(mutant[j + 1] - old_gene) < m_parameters.resolution)
                    {
                        mutant[j + 1] = old_gene;
                    } 
                }

                if (distribution(*m_engine) < gene_mutation_probability)
                {
                    double old_gene = mutant[j + 2];
                    mutateGene(mutant[j + 2], deviation_t);
                    if (std::abs(mutant[j + 2] - old_gene) < 10)
                    {
                        mutant[j + 2] = old_gene;
                    }
                }
            }

            normalizeRadiationTime(mutant, false);

            addNewElement(mutant);
        }

        if (m_premutants.size() > 0)
        {
            m_premutants.erase(m_premutants.begin(), m_premutants.end());
        }
    }



    void PlanGenerator::mutateGene(double& gene, double deviation) const
    {
        if (m_parameters.mutation_type == 1)
        {
            std::normal_distribution<double> normal_distribution(gene, deviation);
            gene = normal_distribution(*m_engine);
        }
        else if (m_parameters.mutation_type == 2)
        {
            std::uniform_real_distribution uniform_distribution(gene - deviation / 2.0, gene + deviation / 2.0);
            gene = uniform_distribution(*m_engine);
        }
    }



    void PlanGenerator::sort(std::vector<RadiationPlan>& radiation_plans, bool normalize)
    {
        std::sort(radiation_plans.begin(), radiation_plans.end(), compareFitness);

        radiation_plans.erase(std::unique(radiation_plans.begin(), radiation_plans.end()), radiation_plans.end());

        if (normalize)
        {
            double sum_fitness = 0;

            for (const RadiationPlan& radiation_plan : radiation_plans) 
            {
                sum_fitness += radiation_plan.second;
            }

            for (RadiationPlan& radiation_plan : radiation_plans)
            {
                radiation_plan.second /= sum_fitness;
            }
        }
    }



    bool PlanGenerator::compareFitness(const RadiationPlan& plan_1, const RadiationPlan& plan_2)
    {
        return (plan_1.second > plan_2.second);
    }


}

