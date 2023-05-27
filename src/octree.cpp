#include<nori/octree.h>

NORI_NAMESPACE_BEGIN
struct treeNode
    {
        void Init(int meshIndex_, int flag_, int nTri_, treeNode* child_){
            meshIndex = meshIndex_;
            flag = flag_;
            nTri = nTri_;
            child = child_;
        };
        int meshIndex;
        int flag;
        int nTri;
        treeNode* child;
    };
Octree::Octree(Mesh* mesh){
    this->mesh = mesh;
    bounds = mesh->getBoundingBox();
    int nums = mesh->getTriangleCount();
    std::vector<int> triNums(nums);
    for (int i = 0; i < nums; ++i){
        triNums[i] = i;
        tribounds.push_back(mesh->getBoundingBox(i));
    } 
    root = new treeNode();
    root->Init(0,0,nums,nullptr);
    buildTree(triNums, bounds, root, nums);

}
void Octree::buildTree(std::vector<int> meshIndex, BoundingBox3f bound, treeNode *cur, int nTri){
    // leaf condition
    if(nTri<5 || bound.getVolume()<0.00001f){
        cur->nTri = nTri;
        cur->meshIndex = meshIndices.size();
        for(int i = 0; i < nTri; i++){
            meshIndices.push_back(meshIndex[i]);
        }
        cur->flag = 1;
    }else{
        std::vector<BoundingBox3f> subbounds;
        Point3f halfsize =0.5*(bound.max-bound.min);
        cur->child = new treeNode[8];
        std::vector<std::vector<int>> indexArray(8);
        // get all bounding boxes for all 8 subnodes
        for (int i = 0; i < 2; i++){
            for (int j = 0; j < 2; j++){
                for (int k = 0; k < 2; k++){
                    Point3f add(i*halfsize.x(),j*halfsize.y(),k*halfsize.z());
                    subbounds.push_back(BoundingBox3f(bound.min+add,bound.min+add+halfsize));
                }
            }
        }
        // search through all mesh triangles
        for(int index = 0; index < nTri; index++){
            for(int i = 0; i< 8; i++){
                if(tribounds[meshIndex[index]].overlaps(subbounds[i])){
                    indexArray[i].push_back(meshIndex[index]);
                }
            }
        }
        for(int i = 0; i < 8; i++){
            (cur->child)[i].Init(0,0,indexArray[i].size(),nullptr);
            buildTree(indexArray[i],subbounds[i],&(cur->child)[i],indexArray[i].size());
        }
    }
}
bool Octree::IntersectOctree(Ray3f &ray, Intersection &its, bool shadowRay, int &triIndex) const{
    return Octree::recursive(*root,ray,its,shadowRay,bounds,triIndex);
}

bool Octree::recursive(const treeNode &node, Ray3f &ray_, Intersection &its, bool shadowRay, BoundingBox3f bbox, int &triIndex) const{
    bool foundIntersection = false;

    // only check triangles of node and its children if ray intersects with node bbox
    if (!bbox.rayIntersect(ray_)) {
        return false;
    }
    std::vector<BoundingBox3f> subbounds;
    Point3f halfsize =0.5*(bbox.max-bbox.min);
    for (int i = 0; i < 2; i++){
            for (int j = 0; j < 2; j++){
                for (int k = 0; k < 2; k++){
                    Point3f add(i*halfsize.x(),j*halfsize.y(),k*halfsize.z());
                    subbounds.push_back(BoundingBox3f(bbox.min+add,bbox.min+add+halfsize));
                }
            }
        }
    // search through all triangles in node
    if (node.flag==1)
    for (uint32_t i = 0; i < node.nTri; ++i) {
        float u, v, t;
        uint32_t triangle_idx = meshIndices[node.meshIndex+i];
        if (mesh->rayIntersect(triangle_idx, ray_, u, v, t) && t < ray_.maxt) {
            /* An intersection was found! Can terminate
               immediately if this is a shadow ray query */
            if (shadowRay)
                return true;
            ray_.maxt = its.t = t;
            its.uv = Point2f(u, v);
            its.mesh = mesh;
            triIndex = triangle_idx;
            foundIntersection = true;
        }
    }
    else {
        std::vector<std::pair<int, float>> children(8);
        for (int i = 0; i < 8; i++){
            children[i].first = i;
            children[i].second = subbounds[i].distanceTo(ray_.o);
        }

        std::sort(children.begin(), children.end(), [](const std::pair<int, float>& l, const std::pair<int, float>& r) {
            return l.second < r.second;
        });

        for (int i = 0; i<8; i++) {
            int index = children[i].first;
            foundIntersection = Octree::recursive(node.child[index], ray_, its, shadowRay, subbounds[index], triIndex) || foundIntersection;
            if (shadowRay && foundIntersection)
                return true;
        }
    }
    return foundIntersection;
}

NORI_NAMESPACE_END