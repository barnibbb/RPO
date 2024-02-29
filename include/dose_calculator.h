#include <unordered_map>
#include <memory>
#include <vector>
#include <optional>

#include <boost/algorithm/string.hpp>

#include <signal.h>

#include "augmented_octree.h"

namespace rpo
{
    using RadiationPlan = std::pair<std::vector<double>, double>;
    using PlanElement = std::pair<point3d, double>;
    using ExposureMap = std::unordered_map<OcTreeKey, double, OcTreeKey::KeyHash>;

    using IndexVector = std::vector<int>;

    struct Score { double general_coverage, object_coverage; };

    class DoseCalculator
    {
    public:
        DoseCalculator(const std::shared_ptr<AugmentedOcTree> augmented_model, const Parameters& parameters);
        Parameters getParameters() const;

        // Preprocessing
        void computeGroundZone();
        void computeGridElements();
        void computeRayTargets();
        std::multimap<double, double> getGroundZone() const;
        RadiationPlan getGrid(const RadiationPlan& plan) const;

        // Irradiance maps
        void computeIrradianceMaps();
        void loadIrradianceMaps();
        void saveIrradianceMap(const OcTreeKey& plan_element_key, const ExposureMap& irradiance_map);
        ExposureMap loadIrradianceMap(const OcTreeKey& plan_element_key) const;

        // Optimization elements
        void setOptimizationElements();
        void create2DModel();
        void compute2DNormalVectors();
        void computeLeastEfficientElements();
        bool isHiddenElement(const point3d& point) const;

        // Dose computation
        void compute(std::vector<RadiationPlan>& radiation_plans, IndexVector& index_vector);
        double computeCoverageForPlan(RadiationPlan& radiation_plan);
        ExposureMap computeDoseForPlanElement(const PlanElement& plan_element, bool verify);
        ExposureMap computeIrradianceForPosition(const point3d& lamp_position, bool verify);
        double computeIrradianceForElement(const point3d& lamp_position, const OcTreeKey& key) const;
        double computeIrradianceType1(const point3d& lamp_position, const OcTreeKey& key) const;
        double computeIrradianceType2(const point3d& lamp_position, const OcTreeKey& key) const;
        double computeIrradianceType3(const point3d& lamp_position, const OcTreeKey& key) const;
        double computeIrradianceType4(const point3d& lamp_position, const OcTreeKey& key) const;
        double computeIrradianceIntegral(const point3d& distance, const point3d& normal, double L) const;
        bool compute2DVisibility(const point3d& lamp_position, const point3d& element) const;
        bool compute3DVisibility(const point3d& lamp_position, const point3d& element) const;

    protected:
        Parameters m_parameters;

        std::shared_ptr<AugmentedOcTree> m_augmented_model = nullptr;

        KeySet m_ground_zone_elements;
        KeySet m_optimization_elements;
        KeySet m_verification_elements;

        std::vector<OcTreeKey> m_grid_elements;

        std::vector<double> m_ray_targets;

        std::vector<ExposureMap> m_irradiance_maps;

    };
}
