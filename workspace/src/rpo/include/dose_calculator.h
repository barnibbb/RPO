#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <set>
#include <vector>

#include "extended_octree.h"
#include "parameters.h"

namespace rpo
{
    using RadiationPlan = std::pair<std::vector<double>, double>;
    using PlanElement = std::pair<point3d, double>;
    using ExposureMap = std::unordered_map<OcTreeKey, float, OcTreeKey::KeyHash>; // Previously double was used

    using IndexVector = std::vector<int>;

    using BreakPoints = std::unordered_map<OcTreeKey, std::vector<OcTreeKey>, OcTreeKey::KeyHash>;

    struct Score { double general_coverage, object_coverage; };

    // New structure
    using VoxelID = uint64_t;
    struct IrradianceEntry { VoxelID voxel; float value; };
    using IrradianceMap = std::vector<IrradianceEntry>;
    using AccumMap = std::unordered_map<VoxelID, float>;

    class DoseCalculator
    {
    public:
        DoseCalculator(const std::shared_ptr<ExtendedOcTree> extended_model, const Parameters& parameters);
        Parameters getParameters() const;

        // Preprocessing
        void computeGroundZone();
        void computeGridElements();
        void computeRayTargets();
        std::vector<double> getRayTargets() const;
        std::multimap<double, double> getGroundZone() const;
        RadiationPlan getGrid(const RadiationPlan& plan) const;
        std::vector<double> getGridIndices(const RadiationPlan& plan) const;
        std::vector<OcTreeKey> getGridElements() const;
        double getGroundLevel() const;

        // Irradiance maps
        void computeIrradianceMaps();
        void computeIrradianceMaps2();
        void computeIrradianceMaps3();
        void computeIrradianceMaps4();
        void loadIrradianceMaps();
        void loadIrradianceMaps2();
        void loadIrradianceMaps3();
        void saveIrradianceMap(const OcTreeKey& plan_element_key, const ExposureMap& irradiance_map);
        void saveBinaryMap(const OcTreeKey& plan_element_key, const ExposureMap& irradiance_map);
        void saveBinaryMap2(const OcTreeKey& plan_element_key, const ExposureMap& irradiance_map);
        void saveBinaryMap3(const OcTreeKey& plan_element_key, const IrradianceMap& irradiance_map);
        ExposureMap loadIrradianceMap(const OcTreeKey& plan_element_key) const;
        ExposureMap loadBinaryMap(const OcTreeKey& plan_element_key) const;
        ExposureMap loadBinaryMap2(const OcTreeKey& plan_element_key) const;
        IrradianceMap loadBinaryMap3(const OcTreeKey& plan_element_key) const;

        // Optimization elements
        void setOptimizationElements();
        void setOptimizationElements2();
        void create2DModel();
        void compute2DNormalVectors();
        void compute2DNormalVectors2();
        void computeLeastEfficientElements();
        bool isHiddenElement(const point3d& point) const;

        // Dose computation
        void compute(std::vector<RadiationPlan>& radiation_plans, IndexVector& index_vector);
        double computeCoverageForPlan(RadiationPlan& radiation_plan);
        ExposureMap computeDoseForPlanElement(const PlanElement& plan_element, bool verify, int z_step = 0);
        ExposureMap computeIrradianceForPosition(const point3d& lamp_position, bool verify, int index = -1, int z_step = 0);
        double computeIrradianceForElement(const point3d& lamp_position, const OcTreeKey& key, int index = -1, int z_step = 0);
        double computeIrradianceType1(const point3d& lamp_position, const OcTreeKey& key) const;
        double computeIrradianceType2(const point3d& lamp_position, const OcTreeKey& key) const;
        double computeIrradianceType3(const point3d& lamp_position, const OcTreeKey& key) const;
        double computeIrradianceType4(const point3d& lamp_position, const OcTreeKey& key, int index = -1, int z_step = 0);
        double computeIrradianceType5(const point3d& lamp_position, const OcTreeKey& key) const;
        double computeIrradianceType6(const point3d& lamp_position, const OcTreeKey& key, int index = -1);
        double computeIrradianceIntegral(const point3d& distance, const point3d& normal, double L) const;
        bool compute2DVisibility(const point3d& lamp_position, const point3d& element) const;
        bool compute3DVisibility(const point3d& lamp_position, const point3d& element) const;
        
        // New functions
        void computeGeneralVisibility();

        KeySet getBaseReachableElements() const;

        void computeBreakPoints();
        void computeBreakPoint(const int i);
        void deleteBreakPoint(const int i);

        inline double getOrigin(const double base, const double direction) const;

        bool compute3DVisibility2(const point3d& lamp_position, const point3d& element, int index);


        // New structure
        inline uint64_t packKey(const OcTreeKey& key)
        {
            return (uint64_t(key[2]) << 32) | (uint64_t(key[1]) << 16) | uint64_t(key[0]);
        }



        inline OcTreeKey unpackKey(uint64_t voxel)
        {
            OcTreeKey key;
            key[0] = static_cast<uint16_t>(voxel & 0xFFFF);
            key[1] = static_cast<uint16_t>((voxel >> 16) & 0xFFFF);
            key[2] = static_cast<uint16_t>((voxel >> 32) & 0xFFFF);
            return key;
        }

        void compute2(std::vector<RadiationPlan>& radiation_plans, IndexVector& index_vector);
        double computeCoverageForPlan2(RadiationPlan& radiation_plan);
        AccumMap computeDoseForPlanElement2(const PlanElement& plan_element, bool verify, int z_step = 0);

    protected:
        Parameters m_parameters;
        
        int m_depth;
        double m_resolution;
        double m_ground_level;

        std::shared_ptr<ExtendedOcTree> m_extended_model = nullptr;

        KeySet m_ground_zone_elements;
        KeySet m_optimization_elements;
        KeySet m_verification_elements;
        KeySet m_base_reachable_elements;
        KeySet m_object_elements;

        KeySet m_obstacle_2d;

        std::vector<OcTreeKey> m_grid_elements;

        std::vector<double> m_ray_targets;

        std::vector<ExposureMap> m_irradiance_maps;

        // New structure
        std::vector<IrradianceMap> m_irradiance_maps_2;
        std::unordered_set<VoxelID> m_verification_elements_2;


        std::vector<BreakPoints> m_break_points_x, m_break_points_y, m_break_points_z;
        std::vector<BreakPoints> m_break_points_xn, m_break_points_yn, m_break_points_zn;

        std::set<std::pair<int, int>> m_traversable;
    };
}
