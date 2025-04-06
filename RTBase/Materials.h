#pragma once

#include "Core.h"
#include "Imaging.h"
#include "Sampling.h"
// #include<utility>
#define mathMax(a, b) ((a) > (b) ? (a) : (b))
#pragma warning( disable : 4244)

class BSDF;

class ShadingData
{
public:
	Vec3 x;
	Vec3 wo;
	Vec3 sNormal;
	Vec3 gNormal;
	float tu;
	float tv;
	Frame frame;
	BSDF* bsdf;
	float t;
	ShadingData() {}
	ShadingData(Vec3 _x, Vec3 n)
	{
		x = _x;
		gNormal = n;
		sNormal = n;
		bsdf = NULL;
	}
};

class ShadingHelper
{
public:
	static float fresnelDielectric(float cosTheta, float iorInt, float iorExt)
	{
		// costheta: cos between incident ray and normal
		// iorInt: refractive index of incident ray
		// Add code here
		int etaI, etaE;
		// check direction of light
		if (cosTheta > 0.f) {
			// ray shoot from internal to external 
			etaI = iorExt;
			etaE = iorInt;
		}
		else {
			etaI = iorInt;
			etaE = iorExt;
		}
		// calculate refreaction angle: thetat
		float refractionRate = etaE / etaI;
		float sinTheta = mathMax(sqrt(1 - cosTheta * cosTheta),0); // sin thetai, incident angle
		float sinThetat = refractionRate * sinTheta;

		if (sinThetat >= 1.f) {
			return 1.f;
		}
		float cosThetat = sqrt(1 - sinThetat * sinThetat);
		float fParallel = (cosTheta - refractionRate * cosThetat) / (cosTheta + refractionRate * cosThetat);
		float fPerp = (refractionRate * cosTheta - cosThetat) / (refractionRate * cosTheta + cosThetat);

		return (fParallel * fParallel + fPerp * fPerp)/2;

	}
	static Colour fresnelConductor(float cosTheta, Colour ior, Colour k)
	{
		float sinTheta = sqrt(1 - (cosTheta * cosTheta));
		float csinTheta = sinTheta * sinTheta; // avoid repeat calculation
		float cosThetaSq = cosTheta * cosTheta;
		float sinThetaSq = sinTheta * sinTheta;
		Colour fParallel = ((ior * ior + k * k) * cosTheta * cosTheta - (ior * 2.f*cosTheta) + Colour(sinThetaSq, sinThetaSq, sinThetaSq)) / ((ior * ior + k * k) * cosTheta * cosTheta + ior * cosTheta * 2 + Colour(sinThetaSq, sinThetaSq, sinThetaSq));
		Colour fPerp = ((ior * ior + k * k) - ior * cosTheta * 2 +Colour(cosThetaSq,cosThetaSq,cosThetaSq))/ ((ior * ior + k * k) + ior * cosTheta * 2 + Colour(cosThetaSq, cosThetaSq, cosThetaSq));
		return (fParallel *fParallel + fPerp * fPerp) / 2;
	}
	static float lambdaGGX(Vec3 wi, float alpha)
	{
		// Add code here
		// for use with G(WO,WI)
		// compute support function
		float cosTheta = wi.z;
		if (cosTheta <= 0.f) { return 0.f; }

		float tanThetaSq = (1.f - cosTheta * cosTheta) / (cosTheta * cosTheta);
		return 0.5f * (std::sqrt(1.f + alpha * alpha * tanThetaSq) - 1.f);
	}

	static float Gggx(Vec3 wi, Vec3 wo, float alpha)
	{
		// Add code here
		float lambdawi = lambdaGGX(wi, alpha);
		float lambdawo = lambdaGGX(wo, alpha);
		// gv = 1/1+lambda(V)
		return 1.f / ((1.f + lambdawi) * (1.f + lambdawo));
	}
	static float Dggx(Vec3 h, float alpha)
	{
		// Add code here
		// distribution of ggx
		float cosTheta = h.z;

		float temp = (cosTheta * cosTheta) * (alpha * alpha - 1.f) + 1.f;
		return (alpha * alpha) / (M_PI * temp * temp);
	}
};


class BSDF
{
public:
	Colour emission;
	virtual Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf) = 0;
	virtual Colour evaluate(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual float PDF(const ShadingData& shadingData, const Vec3& wi) = 0;
	virtual bool isPureSpecular() = 0;
	virtual bool isTwoSided() = 0;

	bool isLight()
	{
		return emission.Lum() > 0 ? true : false;
	}
	void addLight(Colour _emission)
	{
		emission = _emission;
	}
	Colour emit(const ShadingData& shadingData, const Vec3& wi)
	{
		return emission;
	}
	virtual float mask(const ShadingData& shadingData) = 0;
};


class DiffuseBSDF : public BSDF
{
public:
	Texture* albedo;
	DiffuseBSDF() = default;
	DiffuseBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Need to tanslate to local first.
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next()); // use sample, which is local coordinate
		pdf = SamplingDistributions::cosineHemispherePDF(wi); // from formula : p = cos theta /pi
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		// translate to world
		Vec3 wiWorld = shadingData.frame.toWorld(wi); // ensure energy 
		return wiWorld;
	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// wi is local
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		if (wiLocal.z <= 0) return Colour(0.f,0.f,0.f); // back clip
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		Vec3 wiLocal = shadingData.frame.toLocal(wi); // transform to local
		if (wiLocal.z <= 0) return 0.0f; // back clip
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class MirrorBSDF : public BSDF
{
public:
	Texture* albedo;
	MirrorBSDF() = default;
	MirrorBSDF(Texture* _albedo)
	{
		albedo = _albedo;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Mirror sampling code
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		
		Vec3 normal(0, 0, 1);
		Vec3 wr = Vec3(-woLocal.x, -woLocal.y, woLocal.z);
		
		Vec3 wi = shadingData.frame.toWorld(wr); // w

		pdf = 1.f; // mirror will reflect all light, Delta distribution
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv)/ wr.z;

		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		return Colour(0.f,0.f,0.f);


	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Mirror PDF
		return 0;
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData) override
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

static Vec3 reflect(const Vec3& incident, const Vec3& normal) {
	return incident - normal * (2.0f * incident.dot(normal));
}

class ConductorBSDF : public BSDF
{
public:
	Texture* albedo;
	Colour eta;
	Colour k;
	float alpha;
	ShadingHelper support;
	ConductorBSDF() = default;
	ConductorBSDF(Texture* _albedo, Colour _eta, Colour _k, float roughness)
	{
		albedo = _albedo;
		eta = _eta;
		k = _k;
		// alpha = 1.62142f * sqrtf(roughness);
		alpha = roughness * roughness;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{


		if (alpha < 0.001f)
		{
			// mirror
			Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo).normalize();
			Vec3 normal(0, 0, (woLocal.z > 0 ? 1 : -1));
			Vec3 wiLocal = reflect(-woLocal, normal).normalize();

			float cosTheta = std::abs(woLocal.dot(normal));
			reflectedColour = ShadingHelper::fresnelConductor(cosTheta, eta, k);
			pdf = 1.0f;
			return shadingData.frame.toWorld(wiLocal);
		}
		else
		{
			
			Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo).normalize();
			if (woLocal.z <= 0.0f)
			{
				pdf = 0.0f;
				return Vec3(0.f, 0.f, 0.f);
			}

			float u1 = sampler->next();
			float u2 = sampler->next();

			// get half vector
			float phi = 2.0f * M_PI * u1;
			float cosTheta = sqrt((1.0f - u2) / (1.0f + (alpha * alpha - 1.0f) * u2));
			float sinTheta = sqrt(std::max(0.0f, 1.0f - cosTheta * cosTheta));
			Vec3 hLocal(sinTheta * cosf(phi), sinTheta * sinf(phi), cosTheta);
			hLocal = hLocal.normalize();

			// check half vector and incoming direction
			if (woLocal.dot(hLocal) < 0.0f)
				hLocal = -hLocal;

			// reflection direction
			Vec3 wiLocal = reflect(-woLocal, hLocal).normalize();
			if (wiLocal.z <= 0.0f)
			{
				pdf = 0.0f;
				return Vec3(0.f, 0.f, 0.f);
			}

			// microface param
			float D = ShadingHelper::Dggx(hLocal, alpha);
			float G = ShadingHelper::Gggx(woLocal, wiLocal, alpha);
			Colour F = ShadingHelper::fresnelConductor(std::max(0.0f, woLocal.dot(hLocal)), eta, k);

			
			float dotHW = mathMax(woLocal.dot(hLocal), EPSILON);
			pdf = (D * hLocal.z) / (4.0f * dotHW + EPSILON);

			float denominator = 4.0f * woLocal.z * wiLocal.z + EPSILON;
			reflectedColour = (F * D * G) / denominator;

			return shadingData.frame.toWorld(wiLocal);
		}

	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo).normalize();
		Vec3 wiLocal = shadingData.frame.toLocal(wi).normalize();
		const float epsilon = 1e-6f;

		if (alpha < 0.001f)
		{
			// mirror case
			Vec3 normal(0, 0, (woLocal.z >= 0 ? 1 : -1));
			Vec3 reflectDir = reflect(woLocal, normal).normalize();
			if ((wiLocal - reflectDir).lengthSq() > epsilon)
				return Colour(0.f, 0.f, 0.f);

			float cosTheta = woLocal.dot(normal);
			return ShadingHelper::fresnelConductor(cosTheta, eta, k);
		}
		else
		{
			if (woLocal.z * wiLocal.z <= 0.0f)
				return Colour(0.f, 0.f, 0.f);

			Vec3 hLocal = (woLocal + wiLocal).normalize();
			if (hLocal.dot(woLocal) < 0.0f)
				hLocal = -hLocal;

			float cosThetaH = std::max(woLocal.dot(hLocal), 0.0f);
			Colour F = ShadingHelper::fresnelConductor(cosThetaH, eta, k);
			float D = ShadingHelper::Dggx(hLocal, alpha);
			float G = ShadingHelper::Gggx(woLocal, wiLocal, alpha);

			return (F * D * G) / (4.0f * abs(woLocal.z) * abs(wiLocal.z) + epsilon);
		}
	}

	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
	

		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo).normalize();
		Vec3 wiLocal = shadingData.frame.toLocal(wi).normalize();
		const float epsilon = 1e-6f;

		if (alpha < 0.001f)
		{
		
			Vec3 normal(0, 0, (woLocal.z >= 0 ? 1 : -1));
			Vec3 reflectDir = reflect(woLocal, normal).normalize();
			return (wiLocal - reflectDir).lengthSq() < epsilon ? 1.0f : 0.0f;
		}
		else
		{
			if (woLocal.z * wiLocal.z <= 0.0f)
				return 0.0f;

			Vec3 hLocal = (woLocal + wiLocal).normalize();
			if (hLocal.dot(woLocal) < 0.0f)
				hLocal = -hLocal;

			float D = ShadingHelper::Dggx(hLocal, alpha);
			float dotHW = std::max(woLocal.dot(hLocal), epsilon);
			return (D * hLocal.z) / (4.0f * dotHW + epsilon);
		}
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};



class GlassBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	GlassBSDF() = default;
	GlassBSDF(Texture* _albedo, float _intIOR, float _extIOR)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}

	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Glass sampling code
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		float eta = 0;
		if (woLocal.z > 0) {
			eta = extIOR / intIOR;
		}
		else {
			eta = intIOR / extIOR;
		}

		Vec3 normal(0, 0, 0);
		// refraction case
		if (woLocal.z > 0) {
			normal.z = 1;
		}
		else {
			normal.z = -1;
		}
		
		// Fresnel-Schlick Appro
		float cosTheta = std::abs(woLocal.dot(normal)); // cos of incoming light
		float f0 = std::pow((intIOR - extIOR) / (intIOR + extIOR), 2);
		float f = f0 + (1 - f0) * std::pow(1 - cosTheta, 5);
		float u = sampler->next();

		// choose refreaction or reflection through probability
		Vec3 wiLocal;
		if (sampler->next() < f) { // check if reflect 
			Vec3 temp(0,0,1); // must use left value
			wiLocal = -woLocal - 2 * (woLocal.dot(Vec3(0, 0, 1))) * temp;
			reflectedColour = albedo->sample(shadingData.tu, shadingData.tv);
			pdf = f;
		}
		else { // refract

			float cosTheta = -woLocal.dot(normal);
			float k = 1 - eta * eta * (1 - cosTheta * cosTheta);
			if (k < 0) {
				// full refection
				
				wiLocal = -woLocal - 2 * (woLocal.dot(Vec3(0, 0, 1))) * normal;
				reflectedColour = albedo->sample(shadingData.tu, shadingData.tv)*f;
				pdf = 1.f;
			}
			else { // refraction
				float sqrtK = std::sqrt(k);
				Vec3 refractionDir = eta * woLocal + (eta * cosTheta - sqrtK) * normal;
				wiLocal = refractionDir.normalize();
				reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) * (1 - f) / (eta * eta);
				pdf = 1 - f;
			}
			
		}
		return shadingData.frame.toWorld(wiLocal);

	}

	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Glass evaluation code
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo).normalize();
		float eta = woLocal.z >0 ? extIOR / intIOR : intIOR / extIOR;
		Vec3 wiLocal = shadingData.frame.toLocal(wi).normalize();

		// change normal dirctions
		Vec3 normal(0, 0, (woLocal.z >= 0) ? 1 : -1);

		// reflection direction
		Vec3 perfectReflectDir = -woLocal + 2 * woLocal.dot(normal) * normal;
		perfectReflectDir = perfectReflectDir.normalize();
		// wiLocal = wiLocal.normalize();

		// refraction direction
		// 
		float cosTheta = std::abs(woLocal.dot(normal));
		float sinThetaSq = 1 - cosTheta * cosTheta;
		float sinThetatSq = eta * eta * sinThetaSq;

		Vec3 perfectRefractDir;
		// refraction or not
		if (sinThetatSq <= 1) {
			float cosThetat = std::sqrt(1 - sinThetatSq);
			perfectRefractDir = ((-woLocal) * eta) + (eta * cosTheta - cosThetat)* normal;
			perfectRefractDir = perfectRefractDir.normalize();
		}

		// check if wi matches refraction/reflection direction
		bool isReflect = wiLocal.lengthSq(perfectReflectDir) < EPSILON;
		bool isRefract = sinThetatSq <= 1 && (wiLocal.lengthSq(perfectRefractDir) < EPSILON);
		if (!isReflect && !isRefract) {
			return Colour(0.f,0.f,0.f);
		}

		// calculate frenel
		float f0 = std::pow((extIOR - intIOR) / (extIOR + intIOR), 2);
		float f = f0 + (1 - f0) * std::pow(1 - cosTheta, 5);

		if (isReflect) {
			return albedo->sample(shadingData.tu, shadingData.tv) * f;
		}
		if(isRefract) {
			return albedo->sample(shadingData.tu, shadingData.tv) * (1 - f) / (eta * eta);
		}
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with GlassPDF
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo).normalize();
		float eta = woLocal.z > 0 ? extIOR / intIOR : intIOR / extIOR;
		Vec3 wiLocal = shadingData.frame.toLocal(wi).normalize();
		Vec3 normal(0, 0, woLocal.z > 0 ? 1 : -1);

		Vec3 perfectReflectDir = -woLocal + 2 * woLocal.dot(normal) * normal;
		perfectReflectDir = perfectReflectDir.normalize();
		//f
		float cosTheta = std::abs(woLocal.dot(normal));
		float sinThetat = eta* std::sqrt(1 - cosTheta * cosTheta);
		Vec3 perfectRefractDir;

		if (sinThetat <= 1.f) {
			float cosThetaT = std::sqrt(1 - sinThetat * sinThetat);
			perfectRefractDir = ((- woLocal) * eta) + (eta * cosTheta - cosThetaT) * normal;
			perfectRefractDir.normalize();
		}

		bool isReflect = wiLocal.lengthSq(perfectReflectDir) < EPSILON;
		bool isRefract = sinThetat <= 1.f && (wiLocal.lengthSq(perfectRefractDir) < EPSILON);

		float f0 = std::pow((extIOR - intIOR) / (extIOR + intIOR), 2.0f);
		float f = f0 + (1 - f0) * std::pow(1 - cosTheta, 5.0f);

		// probability return
		if (isReflect) {
			return f; // reflect
		}
		else if (isRefract) {
			return sinThetat > 1.0f ? 0.0f : (1 - f); // refract
		}
		else {
			return 0.0f; 
		}
	}
	bool isPureSpecular()
	{
		return true;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};


// Materials currently not implement
class DielectricBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	DielectricBSDF() = default;
	DielectricBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Dielectric sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Dielectric PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return false;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class OrenNayarBSDF : public BSDF
{
public:
	Texture* albedo;
	float sigma;
	OrenNayarBSDF() = default;
	OrenNayarBSDF(Texture* _albedo, float _sigma)
	{
		albedo = _albedo;
		sigma = _sigma;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with OrenNayar sampling code
		Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
		pdf = wi.z / M_PI;
		reflectedColour = albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
		wi = shadingData.frame.toWorld(wi);
		return wi;
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar evaluation code
		return albedo->sample(shadingData.tu, shadingData.tv) / M_PI;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with OrenNayar PDF
		Vec3 wiLocal = shadingData.frame.toLocal(wi);
		return SamplingDistributions::cosineHemispherePDF(wiLocal);
	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class PlasticBSDF : public BSDF
{
public:
	Texture* albedo;
	float intIOR;
	float extIOR;
	float alpha;
	PlasticBSDF() = default;
	PlasticBSDF(Texture* _albedo, float _intIOR, float _extIOR, float roughness)
	{
		albedo = _albedo;
		intIOR = _intIOR;
		extIOR = _extIOR;
		alpha = 1.62142f * sqrtf(roughness);
	}
	float alphaToPhongExponent()
	{
		return (2.0f / SQ(std::max(alpha, 0.001f))) - 2.0f;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Replace this with Plastic sampling code
		// diffuse part
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);
		Vec3 normal(0, 0, 1);

		// f
		float cosTheta = std::abs(woLocal.z);
		float f0 = std::pow((extIOR - intIOR) / (extIOR + intIOR), 2.f);
		float f = f0 + (1 - f0) * std::pow(1 - cosTheta, 5);

		float u = sampler->next(); // use random number to decide refraction or reflection
		Vec3 wiLocal;

		if (u < f) {
			// mirror relection
			float exp = alphaToPhongExponent(); // transform from alpha to phone
			
			float phi = 2*M_PI * sampler->next();
			float cosAlpha = std::pow(sampler->next(), 1 / (exp + 1));
			float sinAlpha = std::sqrt(1 - cosAlpha * cosAlpha);

			Vec3 h(sinAlpha * cos(phi), sinAlpha*sin(phi), cosAlpha);
			h = h.normalize();

			// check validiation of dirction (i.e, is it in same side with normal
			if (wiLocal.z * wiLocal.z <= 0) {
				reflectedColour = Colour(0.f, 0.f, 0.f);
				pdf = 0.f;
				return Vec3(0.f, 0.f, 0.f);
			}
			// mirror pdf
			float dist = (exp + 1) * std::pow(cosAlpha, exp) / (2 * M_PI);
			pdf = f * dist / (4 * woLocal.dot(h));

			// mirror colour
			reflectedColour = Colour(1.f, 1.f, 1.f) * f;

		}
		else {
			// diffuse reflection
			wiLocal = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next()).normalize();
			// diffuse pdf
			pdf = (1 - f) * wiLocal.z / M_PI; 
			// diffuse colour
			Colour kd = albedo->sample(shadingData.tu, shadingData.tv);
			reflectedColour = kd * (1 - f);
			
		}
		return shadingData.frame.toWorld(wiLocal);

	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic evaluation code
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo).normalize();
		Vec3 wiLocal = shadingData.frame.toLocal(wi).normalize();
		if (woLocal.z * wiLocal.z <= 0) {
			return Colour(0.f, 0.f, 0.f);
		}

		Vec3 normal(0, 0, 1); //do not consider case when normal.z = -1

		// f
		float cosTheta = std::abs(woLocal.z);
		float f0 = std::pow((extIOR - intIOR) / (extIOR + intIOR), 2.f);
		float f = f0 + (1 - f0) * std::pow(1 - cosTheta, 5);

		// half vector
		Vec3 h = (wiLocal + woLocal).normalize();

		float exp = alphaToPhongExponent(); // transform from alpha to phone
		float cosAlpha = std::max(h.dot(normal), 0.f); // difference between half vec and normal
		float phong = (exp + 2) / (2 * M_PI) * std::pow(cosAlpha, exp);

		// contribution from mirror reflection
		float cosThetai = std::abs(wiLocal.z);
		float cosThetao = cosThetai; // because we are disscussing mirror reflection here
		Colour specular = Colour(1.f,1.f,1.f) * (f * phong) / (4 * cosThetai * cosThetao);
		
		// diffuse contribution
		Colour diffuse = albedo->sample(shadingData.tu, shadingData.tv) * (1 - f) / M_PI;

		return specular + diffuse;
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Replace this with Plastic PDF
		// need to mix pdf of diffuse and mirror
		Vec3 wiLocal = shadingData.frame.toLocal(wi).normalize();
		Vec3 woLocal = shadingData.frame.toLocal(shadingData.wo);

			// f
		float cosTheta = std::abs(woLocal.z);
		float f0 = std::pow((extIOR - intIOR) / (extIOR + intIOR), 2.f);
		float f = f0 + (1 - f0) * std::pow(1 - cosTheta, 5);
		
		// mirror pdf
		float exp = alphaToPhongExponent(); // transform from alpha to phone
		Vec3 h(woLocal + wiLocal);
		h = h.normalize();
		float cosAlpha = h.z;
		float dist = (exp + 1) * std::pow(cosAlpha, exp) / (2 * M_PI);
		float pdfS = f * dist / (4 * woLocal.dot(h));

		// diffuse pdf
		float pdfD = wiLocal.z / M_PI;

		return f * pdfS + (1 - f) * pdfD; // return mix pdf

	}
	bool isPureSpecular()
	{
		return false;
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return albedo->sampleAlpha(shadingData.tu, shadingData.tv);
	}
};

class LayeredBSDF : public BSDF
{
public:
	BSDF* base;
	Colour sigmaa;
	float thickness;
	float intIOR;
	float extIOR;
	LayeredBSDF() = default;
	LayeredBSDF(BSDF* _base, Colour _sigmaa, float _thickness, float _intIOR, float _extIOR)
	{
		base = _base;
		sigmaa = _sigmaa;
		thickness = _thickness;
		intIOR = _intIOR;
		extIOR = _extIOR;
	}
	Vec3 sample(const ShadingData& shadingData, Sampler* sampler, Colour& reflectedColour, float& pdf)
	{
		// Add code to include layered sampling
		return base->sample(shadingData, sampler, reflectedColour, pdf);
	}
	Colour evaluate(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code for evaluation of layer
		return base->evaluate(shadingData, wi);
	}
	float PDF(const ShadingData& shadingData, const Vec3& wi)
	{
		// Add code to include PDF for sampling layered BSDF
		return base->PDF(shadingData, wi);
	}
	bool isPureSpecular()
	{
		return base->isPureSpecular();
	}
	bool isTwoSided()
	{
		return true;
	}
	float mask(const ShadingData& shadingData)
	{
		return base->mask(shadingData);
	}
};