#pragma once
#include <nori/mesh.h>

NORI_NAMESPACE_BEGIN
struct treeNode;
class Octree{
    public:
    Octree(Mesh* mesh);
    BoundingBox3f WorldBound() const { return bounds;}
    bool IntersectOctree(Ray3f &ray_, Intersection &its, bool shadowRay,int &triIndex) const;

    private:
    void buildTree(std::vector<int> meshIndex, BoundingBox3f bound, treeNode *parent, int nTri);
    bool recursive(const treeNode &node, Ray3f &ray_, Intersection &its, bool shadowRay, BoundingBox3f bbox,int &triIndex) const;
    Mesh* mesh;
    std::vector<int> meshIndices;
    BoundingBox3f bounds;
    std::vector<BoundingBox3f> tribounds;
    treeNode *root;
};
NORI_NAMESPACE_END
