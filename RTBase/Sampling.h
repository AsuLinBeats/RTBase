#pragma once

#include "Core.h"
#include <random>
#include <algorithm>
#include"Imaging.h"
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

class TabulatedDistributions {

	std::vector<float> marginalPDF;
	std::vector<float> marginalCDF;
	std::vector<std::vector<float>> conditionalPDF;
	std::vector<std::vector<float>> conditionalCDF;
	int width, height;
public:
	// This is for environment map
	// 
	// TabulatedDistributions(EnvironmentMap* env) { // Try to avoid using included environment map
	TabulatedDistributions(Texture * tex) {
		// Texture* tex = env->env;
		width = tex->width;
		height = tex->height;
		//marginalPDF.resize(height, 0.0f);
		// luminance matrix construction
		std::vector<std::vector<float>> luminanceM(height, std::vector<float>(width, 0.f));// filled with 0 for now
		float totalLuminance = 0.f;

		for (int i = 0; i < height; ++i) {
			float v = (i + 0.5f) / height;
			float theta = v * M_PI;
			float sinTheta = sin(theta);

			for (int j = 0; j < width; ++j) {
				float u = (j + 0.5f) / width;
				Colour c = tex->sample(u, v);

				float lum = c.r * 0.2126f + c.g * 0.7152f + c.b * 0.0722f;
				luminanceM[i][j] = lum * sinTheta; //! unhandled bug position
				totalLuminance += luminanceM[i][j];
			}
		}

		marginalPDF.resize(height, 0.0f);
		float temp = 1.0f / totalLuminance;
		for (auto& row : luminanceM) {
			for (float& val : row) {
				val *= temp;
			}
		}
		// build conditional distribution
		conditionalPDF.resize(height);
		conditionalCDF.resize(height);
		for (int i = 0; i < height; ++i) {
			conditionalPDF[i].resize(width);
			if (i < 0 || i >= marginalPDF.size()) { std::cout << "error"; }
			float rowSum = marginalPDF[i];

			if (rowSum > 0) {
				for (int j = 0; j < width; ++j) {
					conditionalPDF[i][j] = luminanceM[i][j] / rowSum;
				}
			}
			else {
				// use uniform dist for 0 rows
				std::fill(conditionalPDF[i].begin(), conditionalPDF[i].end(), 1.0f / width);
			}

			conditionalCDF[i].resize(width + 1);
			conditionalCDF[i][0] = 0.0f;
			for (int x = 0; x < width; ++x) {
				conditionalCDF[i][x + 1] = conditionalCDF[i][x] + conditionalPDF[i][x];
			}
		}

		marginalCDF.resize(height + 1);
		marginalCDF[0] = 0.0f;
		for (int i = 0; i < height; ++i) {
			marginalCDF[i + 1] = marginalCDF[i] + marginalPDF[i];
		}

	}
	Vec3 sample(float u1, float u2) {
		u1 = std::clamp(u1, 0.0f, 1.0f);
		u2 = std::clamp(u2, 0.0f, 1.0f);

		//// choose row
		//auto y_it = std::lower_bound(marginalCDF.begin(), marginalCDF.end(), u1);
		//int y = std::distance(marginalCDF.begin(), y_it) - 1;
		//// y = std::clamp(y, 0, height - 1);

		//// choose column
		//const auto& cdf = conditionalCDF[y];
		//auto x_it = std::lower_bound(cdf.begin(), cdf.end(), u2);
		//int x = std::distance(cdf.begin(), x_it) - 1;

		auto y_it = std::upper_bound(marginalCDF.begin(), marginalCDF.end(), u1);
		int y = std::clamp(
			static_cast<int>(std::distance(marginalCDF.begin(), y_it)) - 1,
			0, height - 1
		);

		// Step 3: 条件采样
		const auto& cdf = conditionalCDF[y];
		auto x_it = std::upper_bound(cdf.begin(), cdf.end(), u2);
		int x = std::clamp(
			static_cast<int>(std::distance(cdf.begin(), x_it)) - 1,
			0, width - 1
		);

		// change to direction vector
		float phi = (x + 0.5f) / width * 2 * M_PI;
		float theta = (y + 0.5f) / height * M_PI;
		return Vec3{
			sin(theta) * cos(phi),
			cos(theta),
			sin(theta) * sin(phi)
		};

	}

	float MarginalPDF(int y) {
		if (y < 0 || y >= height) return 0.0f;
		return marginalPDF[y];
	}

	float ConditionalPDF(int y, int x) {
		if (y < 0 || y >= height || x < 0 || x >= width) return 0.0f;
		return conditionalPDF[y][x];
	}

	float getPDF(int x, int y)  {
		return conditionalPDF[y][x] * marginalPDF[y];
	}
};
