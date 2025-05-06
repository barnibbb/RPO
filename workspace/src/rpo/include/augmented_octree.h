#pragma once

#include <string>
#include <fstream>
#include <iostream>
#include <memory>
#include <unordered_set>
#include <map>
#include <functional>

#include <octomap/ColorOcTree.h>
#include <pcl/features/normal_3d.h>

#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

using namespace octomap;

namespace rpo
{
    using Color = octomap::ColorOcTreeNode::Color;

    const Color ORANGE(239, 178, 97); // ORANGE(255, 120, 0);
    const Color RED(222, 23, 56);
    const Color BLACK(0, 0, 0);
    const Color BLUE(38, 79, 155);
    const Color GREEN(51, 178, 51);
    const Color PURPLE(160, 0, 160);
    const Color P2(200, 0, 200);
    const Color LIGHTBLUE(0, 255, 255);
    const Color ROYAL_BLUE(65, 105, 225);
    const Color ROYAL_BLUE2(75, 115, 235);
    const Color RUBY_RED2(254, 47, 125);


    // Colors 4
    // Earthy Tones
    const Color WARM_BROWN(139, 69, 19);
    const Color OLIVE_GREEN(85, 107, 47);
    const Color CREAM(255, 253, 208);
    const Color DEEP_BURGUNDY(128, 0, 32);

    // Ocean Breeze
    const Color AQUA_BLUE(0, 206, 209);
    const Color SEAFOAM_GREEN(46, 139, 87);
    const Color CORAL(255, 127, 80);
    const Color SAND(245, 222, 179);

    // Soft Pastels
    const Color LAVENDER(230, 230, 250);
    const Color SOFT_PINK(255, 192, 203);
    const Color MINT_GREEN(152, 251, 152);
    const Color LIGHT_GRAY(211, 211, 211);

    // Sunset Glow
    const Color DEEP_PURPLE(102, 51, 153);
    const Color BRIGHT_ORANGE(255, 165, 0);
    const Color MAGENTA(255, 0, 255);
    const Color GOLDEN_YELLOW(255, 215, 0);

    // Elegant Neutrals
    const Color CHARCOAL(54, 69, 79);
    const Color SLATE_BLUE(106, 90, 205);
    const Color PALE_SAGE(178, 172, 136);
    const Color IVORY(255, 255, 240);

    // Vintage Blues and Golds
    const Color NAVY_BLUE(0, 0, 128);
    const Color DUSTY_ROSE(192, 128, 129);
    const Color GOLD(255, 215, 0);
    const Color LIGHT_BLUE(173, 216, 230);

    // ---

    // Electric Pop
    const Color NEON_PINK(255, 20, 147);
    const Color ELECTRIC_BLUE(0, 191, 255);
    const Color JET_BLACK(0, 0, 0);
    const Color BRIGHT_WHITE(255, 255, 255);

    // Bold Sunrise
    const Color DEEP_NAVY(0, 51, 102);
    // const Color BRIGHT_ORANGE(255, 102, 0);
    const Color VIBRANT_YELLOW(255, 215, 0);
    const Color PURE_WHITE(255, 255, 255);

    // Striking Monochrome with Red
    const Color CRIMSON_RED(220, 20, 60);
    const Color DARK_CHARCOAL(51, 51, 51);
    const Color COOL_GRAY(128, 128, 128);
    const Color SNOW_WHITE(250, 250, 250);

    // Vivid tropics
    const Color BRIGHT_TEAL(0, 128, 128);
    const Color BOLD_MAGENTA(255, 0, 144);
    const Color SUNSHINE_YELLOW(255, 204, 0);
    const Color ROYAL_PURPLE(75, 0, 130);

    // High Contrast Nature
    const Color FOREST_GREEN(34, 139, 34);
    const Color BURNT_ORANGE(205, 92, 92);
    const Color DEEP_PLUM(102, 0, 102);
    // const Color SNOW_WHITE(255, 255, 255);

    // Modern Minimal
    // const Color JET_BLACK(0, 0, 0);
    const Color CRISP_WHITE(255, 255, 255);
    const Color VIVID_AQUA(0, 255, 255);
    const Color HOT_PINK(255, 105, 180);

    // ---
    // Jewel Tones
    const Color EMERALD_GREEN(80, 200, 120);
    const Color SAPPHIRE_BLUE(15, 82, 186);
    const Color RUBY_RED(224, 17, 95);
    // const Color GOLDEN_YELLOW(255, 204, 0);

    // Retro Brights
    const Color BRIGHT_TURQUOISE(0, 255, 239);
    const Color DEEP_ORANGE(255, 85, 0);
    const Color BOLD_LAVENDER(150, 80, 180);
    const Color CHARTREUSE_GREEN(127, 255, 0);

    // Spicy Sunset
    const Color FIERY_RED(255, 69, 0);
    const Color SUNLIT_YELLOW(255, 223, 0);
    const Color COBALT_BLUE(0, 71, 171);
    const Color RICH_PLUM(142, 36, 170);




    using KeySet = std::unordered_set<OcTreeKey, OcTreeKey::KeyHash>;

    const double PRECISION = 1e-4;

    class AugmentedOcTree;

    class AugmentedOcTreeNode : public OcTreeNode
    {
    public:
        friend class AugmentedOcTree;
            
        AugmentedOcTreeNode() : OcTreeNode(), m_normal(point3d(0, 0, 0)), m_dose(0), m_type(0) {}
        AugmentedOcTreeNode(const AugmentedOcTreeNode& node) : OcTreeNode(node), 
            m_normal(node.m_normal), m_dose(node.m_dose), m_type(node.m_type) {}

        bool operator==(const AugmentedOcTreeNode& node) const
        {
            return (node.value == value && node.m_normal == m_normal && node.m_dose == m_dose && node.m_type == m_type);
        }

        bool hasSameAugmentations(const AugmentedOcTreeNode& node) const
        {
            return (node.m_normal == m_normal && node.m_dose == m_dose && node.m_type == m_type);
        }

        void copyData(const AugmentedOcTreeNode& from)
        {
            OcTreeNode::copyData(from);
            this->m_normal      = from.getNormal();
            this->m_dose        = from.getDose();
            this->m_type        = from.getType();
        }

        inline point3d getNormal() const { return m_normal; }
        inline void setNormal(const point3d& normal) { this->m_normal = normal; }

        inline double getDose() const { return m_dose; }
        inline void setDose(const double dose) { this->m_dose = dose; }

        inline int getType() const { return m_type; }
        inline void setType(const int type) { this->m_type = type; }

        std::istream& readData(std::istream& s);
        std::ostream& writeData(std::ostream& s) const;

    protected:
        point3d m_normal = point3d(0, 0, 0);

        double m_dose = 0;

        int m_type = 1;
    };

    using NodePtr = AugmentedOcTreeNode*;

    class AugmentedOcTree : public OccupancyOcTreeBase<AugmentedOcTreeNode>
    {
    public:

        // Octomap related methods -------------------------------------------------------------

        AugmentedOcTree(double resolution);

        AugmentedOcTree* create() const { return new AugmentedOcTree(resolution); }

        std::string getTreeType() const { return "AugmentedOcTree"; }

        virtual bool pruneNode(AugmentedOcTreeNode* node);

        virtual void expandNode(AugmentedOcTreeNode* node);

        virtual bool isNodeCollapsible(const AugmentedOcTreeNode* node) const;

        void updateInnerOccupancy();

        // RPO related methods -----------------------------------------------------------------

        static std::shared_ptr<AugmentedOcTree> convertToAugmentedOcTree(const ColorOcTree& color_octree);

        void pruneTree();

        // Ray cast related methods ------------------------------------------------------------

        // Improved ray cast to consider arbitrary depth and resolution
        bool castRay(const point3d& origin, const point3d& direction, const point3d& target, point3d& end, 
            bool ignore_unknown, double max_range, int depth, double resolution) const;

        bool castRay2(const point3d& origin, const point3d& direction, const point3d& target, point3d& end, 
            bool ignore_unknown, double max_range, int depth, double resolution, std::vector<OcTreeKey>& break_keys1,
            std::vector<OcTreeKey>& break_keys2, bool skip = false) const;

        bool castRay3(const point3d& origin, const point3d& direction, const point3d& target, point3d& end,
            bool ignore_unknown, double max_range, int depth, double resolution, double t0, double t1, double t2, bool show = false) const;

        bool castRay4(const point3d& origin, const point3d& directionP, point3d& end, bool ignoreUnknown, double maxRange, OcTreeKey target, bool show = false) const;

        bool checkBreakPoints(const std::vector<OcTreeKey>& break_keys, const std::vector<OcTreeKey>& break_keys_n,
            const point3d& p_target_3d, const point3d& p_origin_3d, const double floor_plan_level, bool show = false);


        // Ray cast to consider hitting a target without marking it occupied
        bool evaluateRayCast(const point3d& target_point, const point3d& end_point, int depth) const;

        bool checkRayCast(bool good, const point3d& target_point, const point3d& origin, const point3d& direction, 
            point3d& end_point, double max_range, int depth, double resolution, bool ignore_unknown) const;



        void compute3DNormalVectors();

        void computeGroundLevel();

        void findObjects(bool surface);

        void visualize();

        std::array<double, 6> getBoundaries() const;
        

    protected:
        void updateInnerOccupancyRecurs(AugmentedOcTreeNode* node, unsigned int depth);

        class StaticMemberInitializer
        {
        public:
            StaticMemberInitializer() 
            {
                AugmentedOcTree* tree = new AugmentedOcTree(0.1);
                tree->clearKeyRays();
                AbstractOcTree::registerTreeType(tree);
            }

            void ensureLinking() {};
        };

        static StaticMemberInitializer m_augmented_octree_member_init;
    };
}

