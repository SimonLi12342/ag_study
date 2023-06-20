#include <nori/integrator.h>
#include <nori/scene.h>
#include <nori/sampler.h>
#include <nori/emitter.h>
#include <nori/bsdf.h>

NORI_NAMESPACE_BEGIN
class WhittedIntegrator : public Integrator
{
private:
    int maxDepth = 10;
public:
    WhittedIntegrator(const PropertyList& props) {}

    Color3f Li(const Scene* scene, Sampler* sampler, const Ray3f& ray_) const
    {
        int depth = 0;
        Color3f radiance(0.0f);
        float coff = 1.0f;
        Ray3f ray = ray_;
        while (depth < maxDepth)
        {
            depth++;
            Intersection its;
            //no intersection
            if (!scene->rayIntersect(ray, its))
            {
                continue;
            }
            Color3f Le(0.0f);
            // hit light source
            if (its.mesh->isEmitter())
            {
                EmitterQueryRecord record(ray.o, its.p, its.shFrame.n);
                Le = its.mesh->getEmitter()->eval(record);
            }
            //diffuse
            if (its.mesh->getBSDF()->isDiffuse())
            {
                //pick point on light
                Mesh* light = scene->getRandomEmitter(sampler->next1D());
                EmitterQueryRecord directRecord(its.p);
                //sample the light source
                Color3f Li = light->getEmitter()->sample(light, directRecord, sampler);

                //shadow test
                if (scene->rayIntersect(directRecord.shadowRay))
                {
                    Li = 0;
                }

                // bsdf to calc color
                float cosTheta = Frame::cosTheta(its.shFrame.toLocal(directRecord.wi));
                BSDFQueryRecord bsdf(its.toLocal(-ray.d), its.toLocal(directRecord.wi), ESolidAngle);
                Color3f f = its.mesh->getBSDF()->eval(bsdf);
                if (cosTheta < 0)
                {
                    cosTheta = 0;
                }
                radiance += Le + Li * f * cosTheta / (1.0f / (float)scene->getEmitters().size());
                break;
            }
            else // not diffuse
            {
                // bsdf to calc color
                BSDFQueryRecord bRec(its.toLocal(-ray.d));
                Color3f refColor = its.mesh->getBSDF()->sample(bRec, sampler->next2D());//采样一个出射方向

                //importance
                if (sampler->next1D() > 0.95 && refColor.x() <= 0.f)
                {
                    break;
                }

                ray = Ray3f(its.p, its.toWorld(bRec.wo));
                coff /= 0.95f;
            }
        }
        return radiance *= coff;
    }

    std::string toString() const
    {
        return "WhittedIntegrator[]";
    }
};

NORI_REGISTER_CLASS(WhittedIntegrator, "whitted");
NORI_NAMESPACE_END