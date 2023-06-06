/*
    This file is part of Nori, a simple educational ray tracer

    Copyright (c) 2015 by Wenzel Jakob
*/
#include <chrono>
#include <nori/accel.h>
#include <Eigen/Geometry>

NORI_NAMESPACE_BEGIN

void Accel::addMesh(Mesh *mesh) {
    /*if (m_mesh)
        throw NoriException("Accel: only a single mesh is supported!");
    m_mesh = mesh;
    m_bbox = m_mesh->getBoundingBox();*/

    if (m_meshes.size() > 10)
        throw NoriException("Accel: only 10 meshes are supported!");
    m_meshes.push_back(mesh);
    m_bbox.expandBy(mesh->getBoundingBox());
    for (int i = 0; i < mesh->getTriangleCount(); i++) {
        m_indexes.emplace_back(i, m_meshes.size() - 1);
    }
}

void Accel::build() {
    if (m_meshes.empty()) return;

    auto start = std::chrono::high_resolution_clock::now();

    //��ʼ����
    m_tree.clear();
    auto root = AccelNode(m_bbox, getTotalTriangleCount());
    root.indices = m_indexes;
    m_tree.emplace_back(root);

    //��ʼ����������
    std::queue<uint32_t> q;
    q.push(0);

    //��ʼ���ӽڵ��б�
    auto children = std::vector<AccelNode>();

    //��ȡ��������
    auto [LIMIT_COUNT, LIMIT_DEPTH] = getLimits();
    //������
    while (!q.empty()) {
        uint32_t n = q.size();//��α���
        for (uint32_t k = 0; k < n; k++) {
            //����֮�ڶԽڵ�����ֲ�
            if (m_tree[q.front()].indices.size() > LIMIT_COUNT && depth_curr_ < LIMIT_DEPTH) {
                //�����ӽڵ���ʼ����
                m_tree[q.front()].child = m_tree.size();
                //�ָ��ӽڵ�
                divide(q.front(), &children);
                //�ӽڵ���������������
                --count_leaf_;
                for (auto& child : children) {
                    q.push(m_tree.size());
                    m_tree.emplace_back(child);
                    ++count_node_;
                    ++count_leaf_;
                }
            }
            //������������
            q.pop();
            children.clear();
            children.shrink_to_fit();
        }
        depth_curr_++;
    }

    auto over = std::chrono::high_resolution_clock::now();
    auto time = std::chrono::duration_cast<std::chrono::milliseconds>(over - start).count();

    std::cout << "[build time]: " << time << "ms" << std::endl;
    std::cout << "[max depth]: " << depth_curr_ << std::endl;
    std::cout << "[node count]: " << count_node_ << std::endl;
    std::cout << "[leaf count]: " << count_leaf_ << std::endl;
}

void Accel::divide(uint32_t n, std::vector<AccelNode>* children) {
    auto& node = m_tree[n];
    //��ȡ��Χ�����ĵ�
    Vector3f center = node.bbox.getCenter();
    //��ȡ��Χ�а˸��սǵ�
    for (size_t i = 0; i < 8; i++) {
        //�����Ӱ�Χ��
        Vector3f corner = node.bbox.getCorner(i);
        BoundingBox3f bbox_sub;
        for (uint32_t j = 0; j < 3; j++) {
            bbox_sub.min[j] = std::min(center[j], corner[j]);
            bbox_sub.max[j] = std::max(center[j], corner[j]);
        }

        //�����ӽڵ�
        AccelNode node_sub(bbox_sub);
        for (auto [faceIndex, meshIdx] : node.indices) {
            //���ڵ���е�ͼԪ���Ӱ�Χ���Ƿ��ص�
            if (bbox_sub.overlaps(m_meshes[meshIdx]->getBoundingBox(faceIndex))) {
                node_sub.indices.emplace_back(faceIndex, meshIdx);
            }
        }
        children->emplace_back(node_sub);
    }
}

bool Accel::traverse(uint32_t n, Ray3f& ray, Intersection& its, uint32_t& f, bool shadowRay) const {
    auto& node = m_tree[n];

    //��ǰ�ڵ��Χ����������ײ
    if (!node.bbox.rayIntersect(ray))
        return false;

    bool isHit = false;

    //�ڵ�ΪҶ�ӽڵ�
    if (node.child == 0) {
        float u, v, t;
        //�����ڵ��ڵ�ͼԪ
        for (auto [faceIndex, meshIndex] : node.indices) {
            //���������ཻ�������ͼԪ
            if (m_meshes[meshIndex]->rayIntersect(faceIndex, ray, u, v, t) && t < ray.maxt) {
                if (shadowRay) {
                    return true;
                }
                ray.maxt = t;
                its.t = t;
                its.uv = Point2f(u, v);
                its.mesh = m_meshes[meshIndex];
                f = faceIndex;
                isHit = true;
            }
        }
    }
    else {
        std::pair<uint32_t, float> children[8] = {};
        for (uint32_t i = 0; i < 8; i++) {
            auto ptr = node.child + i;
            //����ڵ㵽����ԭ��ľ���
            children[i] = { ptr, m_tree[ptr].bbox.distanceTo(ray.o) };
        }
        //���ӽڵ��������
        std::sort(
            children,
            children + 8,
            [](const auto& lhs, const auto& rhs) {
                return lhs.second < rhs.second;
            }
        );
        //�ݹ��ӽڵ����
        for (auto& child : children) {
            isHit |= traverse(child.first, ray, its, f, shadowRay);
            if (shadowRay && isHit) {
                return true;
            }
        }
    }
    return isHit;
}

bool Accel::rayIntersect(const Ray3f& ray_, Intersection& its, bool shadowRay) const {
    bool foundIntersection = false;  // Was an intersection found so far?
    auto f = (uint32_t)-1;      // Triangle index of the closest intersection

    Ray3f ray(ray_); /// Make a copy of the ray (we will need to update its '.maxt' value)


    //Brute force search through all triangles
  /*  for (uint32_t idx = 0; idx < m_mesh->getTriangleCount(); ++idx) {
      float u, v, t;
      if (m_mesh->rayIntersect(idx, ray, u, v, t)) {
        //An intersection was found! Can terminate
        //immediately if this is a shadow ray query
        if (shadowRay)
          return true;
        ray.maxt = its.t = t;
        its.uv = Point2f(u, v);
        its.mesh = m_mesh;
        f = idx;
        foundIntersection = true;
      }
    }*/

    foundIntersection = traverse(0, ray, its, f, shadowRay);

    if (shadowRay)
        return foundIntersection;

    if (foundIntersection) {
        /* At this point, we now know that there is an intersection,
           and we know the triangle index of the closest such intersection.

           The following computes a number of additional properties which
           characterize the intersection (normals, texture coordinates, etc..)
        */

        /* Find the barycentric coordinates */
        Vector3f bary;
        bary << 1 - its.uv.sum(), its.uv;

        /* References to all relevant mesh buffers */
        const Mesh* mesh = its.mesh;
        const MatrixXf& V = mesh->getVertexPositions();
        const MatrixXf& N = mesh->getVertexNormals();
        const MatrixXf& UV = mesh->getVertexTexCoords();
        const MatrixXu& F = mesh->getIndices();

        /* Vertex indices of the triangle */
        uint32_t idx0 = F(0, f), idx1 = F(1, f), idx2 = F(2, f);

        Point3f p0 = V.col(idx0), p1 = V.col(idx1), p2 = V.col(idx2);

        /* Compute the intersection positon accurately
           using barycentric coordinates */
        its.p = bary.x() * p0 + bary.y() * p1 + bary.z() * p2;

        /* Compute proper texture coordinates if provided by the mesh */
        if (UV.size() > 0)
            its.uv = bary.x() * UV.col(idx0) +
            bary.y() * UV.col(idx1) +
            bary.z() * UV.col(idx2);

        /* Compute the geometry frame */
        its.geoFrame = Frame((p1 - p0).cross(p2 - p0).normalized());

        if (N.size() > 0) {
            /* Compute the shading frame. Note that for simplicity,
               the current implementation doesn't attempt to provide
               tangents that are continuous across the surface. That
               means that this code will need to be modified to be able
               use anisotropic BRDFs, which need tangent continuity */

            its.shFrame = Frame(
                (bary.x() * N.col(idx0) +
                    bary.y() * N.col(idx1) +
                    bary.z() * N.col(idx2)).normalized());
        }
        else {
            its.shFrame = its.geoFrame;
        }
    }

    return foundIntersection;
}

NORI_NAMESPACE_END

