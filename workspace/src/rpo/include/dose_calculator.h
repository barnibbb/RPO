#include <unordered_map>
#include <memory>
#include <vector>

#include "augmented_octree.h"
#include "parameters.h"

namespace rpo
{
    using RadiationPlan = std::pair<std::vector<double>, double>;
    using PlanElement = std::pair<point3d, double>;
    using ExposureMap = std::unordered_map<OcTreeKey, float, OcTreeKey::KeyHash>; // Previously double was used

    using IndexVector = std::vector<int>;

    using BreakPoints = std::unordered_map<OcTreeKey, std::vector<OcTreeKey>, OcTreeKey::KeyHash>;

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
        std::vector<double> getRayTargets() const;
        std::multimap<double, double> getGroundZone() const;
        RadiationPlan getGrid(const RadiationPlan& plan) const;
        std::vector<OcTreeKey> getGridElements() const;
        double getGroundLevel() const;

        // Irradiance maps
        void computeIrradianceMaps();
        void loadIrradianceMaps();
        void saveIrradianceMap(const OcTreeKey& plan_element_key, const ExposureMap& irradiance_map);
        void saveBinaryMap(const OcTreeKey& plan_element_key, const ExposureMap& irradiance_map);
        void saveBinaryMap2(const OcTreeKey& plan_element_key, const ExposureMap& irradiance_map);
        ExposureMap loadIrradianceMap(const OcTreeKey& plan_element_key) const;
        ExposureMap loadBinaryMap(const OcTreeKey& plan_element_key) const;
        ExposureMap loadBinaryMap2(const OcTreeKey& plan_element_key) const;

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
        ExposureMap computeIrradianceForPosition(const point3d& lamp_position, bool verify, int index = -1);
        double computeIrradianceForElement(const point3d& lamp_position, const OcTreeKey& key, int index = -1);
        double computeIrradianceType1(const point3d& lamp_position, const OcTreeKey& key) const;
        double computeIrradianceType2(const point3d& lamp_position, const OcTreeKey& key) const;
        double computeIrradianceType3(const point3d& lamp_position, const OcTreeKey& key) const;
        double computeIrradianceType4(const point3d& lamp_position, const OcTreeKey& key, int index = -1);
        double computeIrradianceIntegral(const point3d& distance, const point3d& normal, double L) const;
        bool compute2DVisibility(const point3d& lamp_position, const point3d& element) const;
        bool compute3DVisibility(const point3d& lamp_position, const point3d& element) const;
        
        // New functions
        void computeGeneralVisibility();

        KeySet getBaseReachableElements() const;

        void computeBreakPoints();

        inline double getOrigin(const double base, const double direction) const;

        bool compute3DVisibility2(const point3d& lamp_position, const point3d& element, int index);


    protected:
        Parameters m_parameters;
        
        int m_depth;
        double m_resolution;
        double m_ground_level;

        std::shared_ptr<AugmentedOcTree> m_augmented_model = nullptr;

        KeySet m_ground_zone_elements;
        KeySet m_optimization_elements;
        KeySet m_verification_elements;
        KeySet m_base_reachable_elements;
        KeySet m_object_elements;

        KeySet m_obstacle_2d;

        std::vector<OcTreeKey> m_grid_elements;

        std::vector<double> m_ray_targets;

        std::vector<ExposureMap> m_irradiance_maps;

        std::vector<BreakPoints> m_break_points_x, m_break_points_y, m_break_points_z;
        std::vector<BreakPoints> m_break_points_xn, m_break_points_yn, m_break_points_zn;
    };
}
