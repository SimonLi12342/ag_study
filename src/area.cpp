
#include <nori/emitter.h>
#include <nori/mesh.h>
#include <nori/sampler.h>

NORI_NAMESPACE_BEGIN

class AreaLight : public Emitter
{
private:
    Color3f m_radiance;
public:
    AreaLight(const PropertyList& propList)
    {
        m_radiance = propList.getColor("radiance");
    }

    Color3f eval(const EmitterQueryRecord& emit_record) const override
    {
        return (emit_record.n.dot(emit_record.wi) < 0.0f) ? m_radiance : 0.0f;
    }

    Color3f getRadiance() const override
    {
        return m_radiance;
    }

    Color3f sample(const Mesh* mesh, EmitterQueryRecord& record, Sampler* sampler) const override
    {
        // uniform sample the light source
        auto result = mesh->sampleSurfaceUniform(sampler);
        record.p = result.p;
        record.n = result.n;
        record.wi = (record.p - record.ref).normalized();
        record.shadowRay = Ray3f(record.ref, record.wi, Epsilon, (record.p - record.ref).norm() - Epsilon);
        record.pdf = pdf(mesh, record);
        if (record.pdf > 0.0f && !std::isnan(record.pdf) && !std::isinf(record.pdf))
        {
            return eval(record) / record.pdf;
        }
        // sample unsuccessful
        return Color3f(0.0f);
    }

    float pdf(const Mesh* mesh, const EmitterQueryRecord& record) const override
    {
        float costTheta = record.n.dot(-record.wi);
        if (costTheta <= 0.0f)
        {
            return 0.0f;
        }
        //light source
        return mesh->getPdf().getNormalization() * (record.p - record.ref).squaredNorm() / costTheta;
    }

    std::string toString() const override
    {
        return "Emitter[]";
    }
};

NORI_REGISTER_CLASS(AreaLight, "area")
NORI_NAMESPACE_END