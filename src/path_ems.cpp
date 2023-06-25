#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class PathMats : public Integrator {
public:
    PathMats(const PropertyList& props) {}

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& _ray) const override {
        Color3f color = 0;// final color
        Color3f t = 1;// contribution of this interaction
        Ray3f rayRecursive = _ray;
        float probability;// probability to continue
        int depth = 1;// depth to bounce light
        while (true) {
            Intersection its;
            if (!scene->rayIntersect(rayRecursive, its))
                break;
            // it is a light source
            if (its.mesh->isEmitter()) {
                EmitterQueryRecord lRecE(rayRecursive.o, its.p, its.shFrame.n);
                color += t * its.mesh->getEmitter()->eval(lRecE);
            }
            // Russian Roulette
            if (depth >= 3) {
                probability = std::min(t.maxCoeff(), 0.99f);// pick the largest contribution
                if (sampler->next1D() > probability)
                    break;
                t /= probability;
            }
            BSDFQueryRecord bRec(its.shFrame.toLocal(-rayRecursive.d));
            Color3f f = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
            t *= f;// add the contribution
            // continue recursion
            rayRecursive = Ray3f(its.p, its.toWorld(bRec.wo));
            depth++;
        }
        return color;
    }

    std::string toString() const {
        return "PathMats[]";
    }
};

NORI_REGISTER_CLASS(PathMats, "path_mats");
NORI_NAMESPACE_END