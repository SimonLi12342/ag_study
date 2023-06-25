#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/sampler.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class PathEms : public Integrator {
public:
    PathEms(const PropertyList& props) {}

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& _ray) const override {
        Color3f color = 0;
        Color3f t = 1;// final contribution of this interaction
        Ray3f rayRecursive = _ray;
        float probability;
        int depth = 1;
        int isDelta = 1;// is diffuse
        while (true) {
            Intersection its;
            if (!scene->rayIntersect(rayRecursive, its))
                break;
            if (its.mesh->isEmitter()) {// light source
                EmitterQueryRecord lRecE(rayRecursive.o, its.p, its.shFrame.n);
                color += t * its.mesh->getEmitter()->eval(lRecE) * isDelta;
            }
            if (its.mesh->getBSDF()->isDiffuse()) {
                auto light = scene->getRandomEmitter(sampler->next1D());//uniform sample light source
                EmitterQueryRecord lRec(its.p);
                Color3f Li = light->getEmitter()->sample(light, lRec, sampler);//sample outgoing direction
                if (scene->rayIntersect(lRec.shadowRay)) {//outgoing direction no intersection
                    Li = 0;
                }
                float cosTheta = Frame::cosTheta(its.shFrame.toLocal(lRec.wi));
                BSDFQueryRecord bRec(its.toLocal(-rayRecursive.d), its.toLocal(lRec.wi), ESolidAngle);
                Color3f f = its.mesh->getBSDF()->eval(bRec);//calculate bsdf
                // accumulate to final result
                color += Li * f * cosTheta * t / (1.0f / (float)scene->getEmitters().size());
                isDelta = 0;
            }
            else {
                isDelta = 1;// not diffuse then next time 
            }
            //Russian Roulette 
            if (depth >= 3) {
                probability = std::min(t.maxCoeff(), 0.99f);
                if (sampler->next1D() > probability)
                    break;
                t /= probability;
            }
            //
            BSDFQueryRecord bRec(its.shFrame.toLocal(-rayRecursive.d));
            Color3f f = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
            t *= f;// accumulate contribution
            rayRecursive = Ray3f(its.p, its.toWorld(bRec.wo)); // update recursive ray
            depth++;
        }
        return color;
    }

    std::string toString() const {
        return "PathEms[]";
    }
};

NORI_REGISTER_CLASS(PathEms, "path_ems");
NORI_NAMESPACE_END