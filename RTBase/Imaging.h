#pragma once

#include "Core.h"
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define __STDC_LIB_EXT1__
#include "stb_image_write.h"
#include<string>
#include<OpenImageDenoise/oidn.hpp>
// Stop warnings about buffer overruns if size is zero. Size should never be zero and if it is the code handles it.
#pragma warning( disable : 6386)

constexpr float texelScale = 1.0f / 255.0f;

class Texture
{
public:
	Colour* texels;
	float* alpha;
	int width;
	int height;
	int channels;
	void loadDefault()
	{
		width = 1;
		height = 1;
		channels = 3;
		texels = new Colour[1];
		texels[0] = Colour(1.0f, 1.0f, 1.0f);
	}
	void load(std::string filename)
	{
		alpha = NULL;
		if (filename.find(".hdr") != std::string::npos)
		{
			float* textureData = stbi_loadf(filename.c_str(), &width, &height, &channels, 0);
			if (width == 0 || height == 0)
			{
				loadDefault();
				return;
			}
			texels = new Colour[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				texels[i] = Colour(textureData[i * channels], textureData[(i * channels) + 1], textureData[(i * channels) + 2]);
			}
			stbi_image_free(textureData);
			return;
		}
		unsigned char* textureData = stbi_load(filename.c_str(), &width, &height, &channels, 0);
		if (width == 0 || height == 0)
		{
			loadDefault();
			return;
		}
		texels = new Colour[width * height];
		for (int i = 0; i < (width * height); i++)
		{
			texels[i] = Colour(textureData[i * channels] / 255.0f, textureData[(i * channels) + 1] / 255.0f, textureData[(i * channels) + 2] / 255.0f);
		}
		if (channels == 4)
		{
			alpha = new float[width * height];
			for (int i = 0; i < (width * height); i++)
			{
				alpha[i] = textureData[(i * channels) + 3] / 255.0f;
			}
		}
		stbi_image_free(textureData);
	}
	Colour sample(const float tu, const float tv) const
	{
		Colour tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		Colour s[4];
		s[0] = texels[y * width + x];
		s[1] = texels[y * width + ((x + 1) % width)];
		s[2] = texels[((y + 1) % height) * width + x];
		s[3] = texels[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	float sampleAlpha(const float tu, const float tv) const
	{
		if (alpha == NULL)
		{
			return 1.0f;
		}
		float tex;
		float u = std::max(0.0f, fabsf(tu)) * width;
		float v = std::max(0.0f, fabsf(tv)) * height;
		int x = (int)floorf(u);
		int y = (int)floorf(v);
		float frac_u = u - x;
		float frac_v = v - y;
		float w0 = (1.0f - frac_u) * (1.0f - frac_v);
		float w1 = frac_u * (1.0f - frac_v);
		float w2 = (1.0f - frac_u) * frac_v;
		float w3 = frac_u * frac_v;
		x = x % width;
		y = y % height;
		float s[4];
		s[0] = alpha[y * width + x];
		s[1] = alpha[y * width + ((x + 1) % width)];
		s[2] = alpha[((y + 1) % height) * width + x];
		s[3] = alpha[((y + 1) % height) * width + ((x + 1) % width)];
		tex = (s[0] * w0) + (s[1] * w1) + (s[2] * w2) + (s[3] * w3);
		return tex;
	}
	~Texture()
	{
		delete[] texels;
		if (alpha != NULL)
		{
			delete alpha;
		}
	}
};

class ImageFilter
{
public:
	virtual float filter(const float x, const float y) const = 0;
	virtual int size() const = 0;
};

float Gaussians(float alpha, float x) {
	return std::expf(-alpha * x * x);
}



class BoxFilter : public ImageFilter
{
public:

	float filter(float x, float y) const
	{
		if (fabsf(x) <= 0.5f && fabs(y) <= 0.5f)
		{
			return 1;
		}
		return 0;
	}

	int size() const
	{
		return 0;
	}
};



class GaussianFilter : public ImageFilter
{
public:
	float radius;
	float alpha;
	GaussianFilter(float r, float a) {
		radius = r;
		alpha = a;
	}

	float filter(float x, float y) const
	{
		if (fabsf(x) <= 0.5f && fabs(y) <= 0.5f)
		{

			float gx = Gaussians(alpha, x);
			float gy = Gaussians(alpha, y);

			float filter = gx * gy;
			return filter;
		}
		return 0;
	}

	int size() const
	{
		return std::ceil(radius) * 2;
	}
};

class Mitchell_Netravali : public ImageFilter {
	// changable parameters
	const float B = 1 / 3;
	const float C = 1 / 3;

	float h(float x) const {
		if (fabsf(x) >= 0 && fabsf(x) < 1) {
			return ((1 / 6) * ((12 - 9 * B - 6 * C) * std::pow(fabsf(x), 3) + (-18 + 12 * B + 6 * C) * std::pow(fabsf(x), 2) + (6 - 2 * B)));
		}
		if (fabsf(x) >= 1 && fabsf(x) < 2) {
			return ((1 / 6) * ((-B - 6 * C) * std::pow(fabsf(x), 3) + (6 * B + 30 * C) * std::pow(fabsf(x), 2) + (-12 * B - 48 * C) * fabsf(x) + (8 * B + 24 * C)));
		}
		if (fabsf(x) >= 2) {
			return 0;
		}
	}

	float filter(float x, float y) const {
		if (fabsf(x) <= 0.5f && fabs(y) <= 0.5f) {
			float hx = h(x);
			float hy = h(y);

			float filter = hx * hy;
			return filter;
		}
		return 0;
	}
};

class Film
{
public:
	Colour* film;
	unsigned int width;
	unsigned int height;
	int SPP;
	ImageFilter* filter;

	// For intel denoise
	Colour* albedoBuffer;
	Colour* normalBuffer;

	// buffers 
	oidn::BufferRef colourBuffer;
	//////////////////////std::vector<float> colourBuffer;
	oidn::BufferRef outputBuffer;
	
	// for adaptive sampling
	std::vector<Colour> tileColourSum;    // colour sum
	std::vector<float> tileVariance;    // colour variance
	std::vector<int> tileSampleCount;   // sample count 
	std::vector<bool> tileNeedsMoreSamples; // more sample?
	int baseSPP = 1;                    
	int maxSPP = 64;                    
	float varianceThreshold = 0.01f;    // thresold for variance

	void tonemap(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b, float exposure = 1.0f)
	{
		
		// Filmic Tone Mapping 
		const float A = 0.22f;
		const float B = 0.30f;
		const float C = 0.10f;
		const float D = 0.20f;
		const float E = 0.01f;
		const float F = 0.30f;
		// ¼ÆËãË÷Òý
		int index = y * width + x;

		Colour hdrColor = film[index] / (float)SPP; 

		float r_hdr = hdrColor.r * exposure;
		float g_hdr = hdrColor.g * exposure;
		float b_hdr = hdrColor.b * exposure;

		auto filmic = [&](float x) -> float {
			float numerator = (x * (A * x + C * B) + D * E);
			float denominator = (x * (A * x + B) + D * F);
			return (numerator / denominator) - (E / F);
			};
		float r_mapped = filmic(r_hdr);
		float g_mapped = filmic(g_hdr);
		float b_mapped = filmic(b_hdr);

		float gamma = 1.0f / 2.2f;
		r_mapped = powf(r_mapped, gamma);
		g_mapped = powf(g_mapped, gamma);
		b_mapped = powf(b_mapped, gamma);
		r = static_cast<unsigned char>((r_mapped * 255.0f > 255.0f) ? 255.0f : ((r_mapped * 255.0f < 0.0f) ? 0.0f : r_mapped * 255.0f));
		g = static_cast<unsigned char>((g_mapped * 255.0f > 255.0f) ? 255.0f : ((g_mapped * 255.0f < 0.0f) ? 0.0f : g_mapped * 255.0f));
		b = static_cast<unsigned char>((b_mapped * 255.0f > 255.0f) ? 255.0f : ((b_mapped * 255.0f < 0.0f) ? 0.0f : b_mapped * 255.0f));
	}

	float Clamp(float& x, float& A, float& B, float& C, float& D, float& E, float& F, float& W) {
		return (x * (A * x + C * B) + D * E) / (x * (A * x + B) + D * F) - (E / F);
	}

	void filmicToneMap(int x, int y, unsigned char& r, unsigned char& g, unsigned char& b, float exposure = 1.0f,
		float A = 0.15f, float B = 0.5f, float C = 0.1f, float D = 0.2f, float E = 0.02f, float F = 0.3f,float W = 11.2f) {
		Colour pixel = film[(y * width) + x] * exposure / (float)SPP; // film is an 1d array, so we need to calculate the index
		// filmic tonemapping
		float L = 0.2126f * pixel.r + 0.7152f * pixel.g + 0.0722f * pixel.b;
		// filmic tonemapping
		float CLuminance = Clamp(L,A,B,C,D,E,F,W);
		float CW = Clamp(W, A, B, C, D, E, F, W);
		double temp = 1 / 2.2f;
		float Lout = (float) std::pow((CLuminance / CW), temp);

		r *= Lout;
		g *= Lout;
		b *= Lout;

		// gamma correction
		//r = std::min(powf(std::max(pixel.r, 0.0f), 1.f / 2.2f) * 255, 255.f);
		//g = std::min(powf(std::max(pixel.g, 0.0f), 1.f / 2.2f) * 255, 255.f);
		//b = std::min(powf(std::max(pixel.b, 0.0f), 1.f / 2.2f) * 255, 255.f);
		// Return a tonemapped pixel at coordinates x, y

	}
	// Do not change any code below this line
	void init(int _width, int _height, ImageFilter* _filter)
	{
		width = _width;
		height = _height;
		film = new Colour[width * height];
		clear();
		filter = _filter;
	}
	void initBuffer(oidn::DeviceRef& device) {
		const size_t bufferSize = width * height * 3 * sizeof(float);
		colourBuffer = device.newBuffer(bufferSize);
		outputBuffer = device.newBuffer(bufferSize);
		albedoBuffer = new Colour[width * height];   
		normalBuffer = new Colour[width * height];

	}

	Colour getColour(int x, int y) const
	{
		// return black when beyond range
		if (x < 0 || x >= (int)width || y < 0 || y >= (int)height)
			return Colour{ 0.0f, 0.0f, 0.0f };

		const int index = y * width + x;

		return film[index] / static_cast<float>(SPP);
	}

	void clear()
	{
		memset(film, 0, width * height * sizeof(Colour));
		SPP = 0;
	}
	void incrementSPP()
	{
		SPP++;
	}
	void save(std::string filename)
	{
		Colour* hdrpixels = new Colour[width * height];
		for (unsigned int i = 0; i < (width * height); i++)
		{
			hdrpixels[i] = film[i] / (float)SPP;
		}
		stbi_write_hdr(filename.c_str(), width, height, 3, (float*)hdrpixels);
		delete[] hdrpixels;
	}

	void splat(const float x, const float y, const Colour& L) {
		float filterWeights[25]; // Storage to cache weights
		unsigned int indices[25]; // Store indices to minimize computations
		unsigned int used = 0;
		float total = 0;
		int size = filter->size();
		for (int i = -size; i <= size; i++) {
			for (int j = -size; j <= size; j++) {
				int px = (int)x + j;
				int py = (int)y + i;
				if (px >= 0 && px < width && py >= 0 && py < height) {
					indices[used] = (py * width) + px;
					filterWeights[used] = filter->filter(j, i);
					total += filterWeights[used];
					used++;
				}
			}
		}
		for (int i = 0; i < used; i++) {
			film[indices[i]] = film[indices[i]] + (L * filterWeights[i] / total);
		}
	}
	

	void splat1(const float x, const float y, const Colour& L) {
		float filterWeights[25];
		unsigned int indices[25];
		unsigned int used = 0;
		float total = 0;
		int size = filter->size();
		for (int i = -size; i <= size; i++) {
			for (int j = -size; j <= size; j++) {
				int px = (int)x + j;
				int py = (int)y + i;
				if (px >= 0 && px < width && py >= 0 && py < height) {
					indices[used] = (py * width) + px;
					filterWeights[used] = filter->filter(j, i);
					total += filterWeights[used];
					used++;
				}
			}
		}
		for (int i = 0; i < used; i++) {
			film[indices[i]] = film[indices[i]] + (L * filterWeights[i] / total);
		}
	}
};