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
		float r = sqrt(r1);
		float phi = 2.0f * M_PI * r2;
		float x = r * cos(phi);
		float y = r * sin(phi);
		float z = sqrt(1.0f - x * x - y * y);
		return Vec3(x, y, z);
	}
	//TODO WATCH THIS PARAMETER
	//static float cosineHemispherePDF(const Vec3 wi,float r1)
	//{
	//	return std::max(0.0f, wi.z) / M_PI;
	//}
	static float cosineHemispherePDF(const Vec3 wi)
	{
		return std::max(0.0f, wi.z) / M_PI;
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

//class SamplingDistributions
//{
//public:
//    // 余弦加权半球采样
//    static Vec3 cosineSampleHemisphere(float r1, float r2)
//    {
//        // 使用Malley方法进行余弦加权半球采样
//        float phi = 2.0f * M_PI * r1;       // 方位角[0,2π]
//        float sinTheta = std::sqrt(r2);     // 极角的正弦
//        float cosTheta = std::sqrt(1.0f - r2); // 极角的余弦
//        // 转换为笛卡尔坐标
//        float x = sinTheta * std::cos(phi);
//        float y = sinTheta * std::sin(phi);
//        float z = cosTheta;
//        return Vec3(x, y, z);  // 返回采样方向
//    }
//    // 余弦加权半球采样的概率密度函数
//    static float cosineHemispherePDF(const Vec3& wi)
//    {
//        // wi必须归一化且在半球内(z > 0)
//        return wi.z / M_PI;  // PDF与cosθ成正比，除以π归一化
//    }
//    // 均匀球体采样
//    static Vec3 uniformSampleSphere(float r1, float r2)
//    {
//        float phi = 2.0f * M_PI * r1;       // 方位角[0,2π]
//        float cosTheta = 1.0f - 2.0f * r2;  // 极角的余弦[-1,1]
//        float sinTheta = std::sqrt(1.0f - cosTheta * cosTheta); // 极角的正弦
//        // 转换为笛卡尔坐标
//        float x = sinTheta * std::cos(phi);
//        float y = sinTheta * std::sin(phi);
//        float z = cosTheta;
//        return Vec3(x, y, z);  // 返回采样方向
//    }
//    // 均匀球体采样的概率密度函数
//    static float uniformSpherePDF(const Vec3& wi)
//    {
//        return 1.0f / (4.0f * M_PI);  // 单位球表面积为4π，PDF为1/4π
//    }
//};