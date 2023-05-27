#pragma once


#include <nori/mesh.h>

NORI_NAMESPACE_BEGIN

// KdTreeAccel Declarations
struct KdAccelNode;
struct BoundEdge;
class KdTree{
  public:
    // KdTreeAccel Public Methods
    KdTree(Mesh *mesh, int isectCost = 80, int traversalCost = 1,
                float emptyBonus = 0.5, int maxPrims = 1, int maxDepth = -1);
    Bounds3f WorldBound() const { return bounds; }
    ~KdTree();
    bool Intersect(const Ray3f &ray_, Intersection &its, bool shadowRay) const;
    bool IntersectP(const Ray &ray) const;

  private:
    // KdTreeAccel Private Methods
    void buildTree(int nodeNum, const BoundingBox3f &bounds,
                   const std::vector<BoundingBox3f> &primBounds, int *primNums,
                   int nprims, int depth,
                   const std::unique_ptr<BoundEdge[]> edges[3], int *prims0,
                   int *prims1, int badRefines = 0);

    // KdTreeAccel Private Data
    const int isectCost, traversalCost, maxPrims;
    const float emptyBonus;
    Mesh *mesh;
    std::vector<int> primitiveIndices;
    KdAccelNode *nodes;
    int nAllocedNodes, nextFreeNode;
    BoundingBox3f bounds;
};

struct KdToDo {
    const KdAccelNode *node;
    float tMin, tMax;
};

NORI_NAMESPACE_END