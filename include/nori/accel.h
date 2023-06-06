/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/

#pragma once

#include <nori/mesh.h>
#include <queue>
#include <utility>

NORI_NAMESPACE_BEGIN

/**
 * \brief Acceleration data structure for ray intersection queries
 *
 * The current implementation falls back to a brute force loop
 * through the geometry.
 */
struct AccelNode
{
    uint32_t child = 0;
    BoundingBox3f bbox;
    std::vector<std::pair<uint32_t, uint32_t>> indices;

    AccelNode() : bbox() {}

    explicit AccelNode(BoundingBox3f box)
        : bbox(std::move(box)) {}

    AccelNode(BoundingBox3f box, uint32_t size)
        : bbox(std::move(box)), indices(size) {}
};

class Accel {
public:
    /**
     * \brief Register a triangle mesh for inclusion in the acceleration
     * data structure
     *
     * This function can only be used before \ref build() is called
     */
    void addMesh(Mesh *mesh);

    /// Build the acceleration data structure (currently a no-op)
    void build();

    /// Return an axis-aligned box that bounds the scene
    const BoundingBox3f &getBoundingBox() const { return m_bbox; }

    /**
     * \brief Intersect a ray against all triangles stored in the scene and
     * return detailed intersection information
     *
     * \param ray
     *    A 3-dimensional ray data structure with minimum/maximum extent
     *    information
     *
     * \param its
     *    A detailed intersection record, which will be filled by the
     *    intersection query
     *
     * \param shadowRay
     *    \c true if this is a shadow ray query, i.e. a query that only aims to
     *    find out whether the ray is blocked or not without returning detailed
     *    intersection information.
     *
     * \return \c true if an intersection was found
     */

    void divide(uint32_t n, std::vector<AccelNode>* children);

    bool traverse(uint32_t n, Ray3f& ray, Intersection& its, uint32_t& f, bool shadowRay) const;

    std::pair<uint32_t, uint32_t> getLimits() const { return { 16, 12 };}

    bool rayIntersect(const Ray3f &ray, Intersection &its, bool shadowRay) const;

    uint32_t getTotalTriangleCount() const { return m_indexes.size(); }

protected:
    std::vector<Mesh*> m_meshes;
    BoundingBox3f m_bbox;
    std::vector<AccelNode> m_tree;
    std::vector<std::pair<uint32_t, uint32_t>> m_indexes;

    uint32_t depth_curr_ = 1;
    uint32_t count_leaf_ = 1;
    uint32_t count_node_ = 1;

private:
    Mesh         *m_mesh = nullptr; ///< Mesh (only a single one for now)
};

NORI_NAMESPACE_END
