#include <string>
#include <fstream>
#include <iostream>
#include <memory>
#include <unordered_set>
#include <map>

#include <octomap/ColorOcTree.h>
#include <pcl/features/normal_3d.h>

#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include "parameters.h"

using namespace octomap;

namespace rpo
{
    using KeySet = std::unordered_set<OcTreeKey, OcTreeKey::KeyHash>;

    class AugmentedOcTree;

    class AugmentedOcTreeNode : public OcTreeNode
    {
    public:
        friend class AugmentedOcTree;
            
        AugmentedOcTreeNode() : OcTreeNode(), m_normal(point3d(0, 0, 0)), m_type(0) {}
        AugmentedOcTreeNode(const AugmentedOcTreeNode& node) : OcTreeNode(node), 
            m_normal(node.m_normal), m_type(node.m_type) {}

        bool operator==(const AugmentedOcTreeNode& node) const
        {
            return (node.value == value && node.m_normal == m_normal && node.m_type == m_type);
        }

        bool hasSameAugmentations(const AugmentedOcTreeNode& node) const
        {
            return (node.m_normal == m_normal && node.m_type   == m_type);
        }

        void copyData(const AugmentedOcTreeNode& from)
        {
            OcTreeNode::copyData(from);
            this->m_normal      = from.getNormal();
            this->m_type        = from.getType();
        }

        inline point3d getNormal() const { return m_normal; }
        inline void setNormal(const point3d& normal) { this->m_normal = normal; }

        inline int getType() const { return m_type; }
        inline void setType(const int type) { this->m_type = type; }

        std::istream& readData(std::istream& s);
        std::ostream& writeData(std::ostream& s) const;

    protected:
        point3d m_normal = point3d(0, 0, 0);

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

