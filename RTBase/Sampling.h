#pragma once

#include "Core.h"
#include <random>
#include <algorithm>

class Sampler
{
public:
	virtual float next() = 0;
};

class MTRandom : public Sampler
{
public:
	std::mt19937 generator;
	std::uniform_real_distribution<float> dist;
	MTRandom(unsigned int seed = 1) : dist(0.0f, 1.0f)
	{
		generator.seed(seed);
	}
	float next()
	{
		return dist(generator);
	}
};

// Note all of these distributions assume z-up coordinate system
class SamplingDistributions
{
public:
	static Vec3 uniformSampleHemisphere(float r1, float r2)
	{
		// theta
		float phi = 2.0f * M_PI * r2;
		float theta = acos(r1);
		//float PDF = 1 / (4*M_PI);
		//float CDFtheta = (1 - cos(theta))/2;
		//float CDFphi = phi / (2 * M_PI);
		return SphericalCoordinates::sphericalToWorld(theta, phi);

	}
	static float uniformHemispherePDF(const Vec3 wi)
	{

		// Add code here
		return 1/(2*M_PI);
	}
	static Vec3 cosineSampleHemisphere(float r1, float r2)
	{
		float theta = acosf(sqrtf(r1));
		float phi = 2.0f * M_PI * r2;
		return Vec3(0, 0, 1);
	}
	//TODO WATCH THIS PARAMETER
	static float cosineHemispherePDF(const Vec3 wi,float r1)
	{
		float theta = acosf(sqrtf(r1));
		return cosf(theta)/M_PI;
	}

	static Vec3 uniformSampleSphere(float r1, float r2)
	{
		// theta
		float phi = 2.0f * M_PI * r2;
		float theta = acos(1 - 2 * r1);
		return SphericalCoordinates::sphericalToWorld(theta,phi);
	}
	static float uniformSpherePDF(const Vec3& wi)
	{
		// Add code here
		return 1/(4*M_PI);
	}
};