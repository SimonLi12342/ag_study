#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/emitter.h>
#include <nori/sampler.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN

class PathMisIntegrator : public Integrator {
public:
    PathMisIntegrator(const PropertyList& props) {}

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray) const override {
        Color3f color = 0;
        Color3f t = 1;
        Ray3f rayRecursive = ray;
        float probability;
        float w_mats = 1.0f;//BRDF weights for next iter
        int depth = 1;
        Intersection its;
        if (!scene->rayIntersect(rayRecursive, its)) {
            return color;
        }
        while (true) {
            if (its.mesh->isEmitter()) {
                EmitterQueryRecord lRec(rayRecursive.o, its.p, its.shFrame.n);
                //lRec.uv = its.uv;
                color += t * w_mats * its.mesh->getEmitter()->eval(lRec);
            }
            //sample light
            const Mesh* mesh = scene->getRandomEmitter(sampler->next1D());
            const Emitter* light = mesh->getEmitter();
            EmitterQueryRecord lRec(its.p);
            //lRec.uv = its.uv;
            Color3f Li = light->sample(mesh, lRec, sampler) * scene->getEmitters().size();
            float pdf_em = light->pdf(mesh, lRec);//light source emitter
            if (!scene->rayIntersect(lRec.shadowRay)) {// not blocked
                float cosTheta = std::max(0.f, Frame::cosTheta(its.shFrame.toLocal(lRec.wi)));
                BSDFQueryRecord bRec(its.toLocal(-rayRecursive.d), its.toLocal(lRec.wi), ESolidAngle);
                Color3f f = its.mesh->getBSDF()->eval(bRec);
                float pdf_mat = its.mesh->getBSDF()->pdf(bRec);//BRDF pdf
                
                // calculate the balance heuristic
                float w_ems = pdf_mat + pdf_em > 0.0f ? pdf_em / (pdf_mat + pdf_em) : pdf_em;
                color += Li * f * cosTheta * w_ems * t;// add to the result
            }
            if (depth >= 3) { // Russian Roulette
                probability = std::min(t.maxCoeff(), 0.99f);
                if (sampler->next1D() > probability) {
                    return color;
                }
                t /= probability;
            }
            //BRDF sampling
            BSDFQueryRecord bRec(its.shFrame.toLocal(-rayRecursive.d));
            Color3f f = its.mesh->getBSDF()->sample(bRec, sampler->next2D());
            t *= f;
            rayRecursive = Ray3f(its.p, its.toWorld(bRec.wo));
            float pdf_mat = its.mesh->getBSDF()->pdf(bRec);//BRDF pdf
            Point3f origin = its.p;
            if (!scene->rayIntersect(rayRecursive, its)) {
                return color;
            }
            if (its.mesh->isEmitter()) {// if emitter update brdf weight
                EmitterQueryRecord newLRec = EmitterQueryRecord(origin, its.p, its.shFrame.n);
                //lRec.uv = its.uv;
                float new_pdf_em = its.mesh->getEmitter()->pdf(its.mesh, newLRec);
                w_mats = pdf_mat + new_pdf_em > 0.f ? pdf_mat / (pdf_mat + new_pdf_em) : pdf_mat;
            }
            if (bRec.measure == EDiscrete) { // if not solid angle
                w_mats = 1.0f;
            }
            depth++;
        }
        return color;
    }

    std::string toString() const {
        return "PathMisIntegrator[]";
    }
};

NORI_REGISTER_CLASS(PathMisIntegrator, "path_mis");
NORI_NAMESPACE_END