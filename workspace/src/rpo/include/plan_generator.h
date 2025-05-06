#include <vector>
#include <memory>
#include <random>
#include <map>
#include <iostream>
#include <ctime>
#include <algorithm>
#include <iomanip>

#include "parameters.h"

namespace rpo
{
    using RadiationPlan = std::pair<std::vector<double>, double>;
    using IndexVector = std::vector<int>;

    class PlanGenerator
    {
    public:
        PlanGenerator(const Parameters& parameters);
        
        void setModelBoundaries(const std::array<double, 6>& model_boundaries);
        void setGroundZone(const std::multimap<double, double>& ground_zone);
        bool isElementOfGroundZone(double x, double y) const;

        // Getters
        int getNumPos() const;
        std::vector<RadiationPlan>& getPopulation();
        IndexVector& getIndexOfUnevaluated();
        RadiationPlan getBestPlan();

        // Create and modify population
        void createInitialPopulation();
        void addNewPosition(std::vector<double>& individual, int index, bool extend) const;
        void normalizeRadiationTime(std::vector<double>& individual, bool extend) const;
        void addNewElement(const std::vector<double>& individual);
        void incrementPlans();

        // Selection functions
        void selectSurvivals();
        IndexVector select(int selection_type, int number_of_selections);
        IndexVector applyFitnessProportionateSelection(int number_of_selections) const;
        IndexVector applyRandomSelection(int number_of_selections) const;
        IndexVector applyTournamentSelection(int number_of_selections) const;
        IndexVector applyTruncationSelection(int number_of_selections) const;

        // Recombination
        void recombine();
        void applyUniformCrossover();
        void applyTwoPointCrossover();

        // Mutation
        void mutate();
        void mutateGene(double& gene, double deviation) const;

        // Evaluation
        static void sort(std::vector<RadiationPlan>& radiation_plans, bool normalize);
        static bool compareFitness(const RadiationPlan& plan_1, const RadiationPlan& plan_2);
        

    private:   
        Parameters m_parameters;

        std::unique_ptr<std::mt19937> m_engine = nullptr;

        std::vector<RadiationPlan> m_population;
        IndexVector m_unevaluated_individuals;
        IndexVector m_parents;
        IndexVector m_premutants;

        std::array<double, 6> m_model_boundaries;
        std::multimap<double, double> m_ground_zone;
    };
}