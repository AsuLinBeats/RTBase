#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "GamesEngineeringBase.h"
#include <thread>
#include <functional>

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom* samplers;
	std::thread** threads;
	int numProcs;
	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new GaussianFilter(1,2));
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		threads = new std::thread * [numProcs];
		samplers = new MTRandom[numProcs];
		clear();
	}
	void clear()
	{
		film->clear();
	}
	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		// Sample a light
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);
		// Sample a point on the light
		float pdf;
		Colour emitted;
		Vec3 p = light->sample(shadingData, sampler, emitted, pdf);
		if (light->isArea())
		{
			// Calculate GTerm
			Vec3 wi = p - shadingData.x;
			float l = wi.lengthSq();
			wi = wi.normalize();
			float GTerm = (max(Dot(wi, shadingData.sNormal), 0.0f) * max(-Dot(wi, light->normal(shadingData, wi)), 0.0f)) / l;
			if (GTerm > 0)
			{
				// Trace
				if (scene->visible(shadingData.x, p))
				{
					// Shade
					return shadingData.bsdf->evaluate(shadingData, wi) * emitted * GTerm / (pmf * pdf);
				}
			}
		}
		else
		{
			// Calculate GTerm
			Vec3 wi = p;
			float GTerm = max(Dot(wi, shadingData.sNormal), 0.0f);
			if (GTerm > 0)
			{
				// Trace
				if (scene->visible(shadingData.x, shadingData.x + (p * 10000.0f)))
				{
					// Shade
					return shadingData.bsdf->evaluate(shadingData, wi) * emitted * GTerm / (pmf * pdf);
				}
			}
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	const int MAX_DEPTH = 3; // generally need 3-5 bounces
	Colour pathTrace(Ray& r, Colour pathThroughput, int depth, Sampler* sampler, bool canHitLight = true)
	{
		// depth: initial depth is 0 or 1
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				if (canHitLight == true)
				{
					return pathThroughput * shadingData.bsdf->emit(shadingData, shadingData.wo);
				}
				else
				{
					return Colour(0.0f, 0.0f, 0.0f);
				}
			}
			Colour direct = pathThroughput * computeDirect(shadingData, sampler);
			 if (depth > MAX_DEPTH)
			// if (depth > 4)
			{
				return direct;
			}
			//float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);

			//if (sampler->next() < russianRouletteProbability)
			//{
			//	pathThroughput = pathThroughput / russianRouletteProbability;
			//}
			//else
			//{
			//	return direct;
			//}

			 if (depth > 3) {  // start RR after 3 bounces
				 float surviveProb = 0.8f;
				 if (sampler->next() > surviveProb) {
					 return direct;
				 }
				 pathThroughput = pathThroughput / surviveProb;
			 }
			Colour bsdf;
			float pdf;
			Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
			 //Colour indirect;
			 //float pdf;
			 //Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, indirect, pdf);
			 
			 pdf = SamplingDistributions::cosineHemispherePDF(wi);
			wi = shadingData.frame.toWorld(wi);
			bsdf = shadingData.bsdf->evaluate(shadingData, wi);
			pathThroughput = pathThroughput * bsdf * fabsf(Dot(wi, shadingData.sNormal)) / pdf;
			// r.init(shadingData.x + (wi * EPSILON), wi);
			Vec3 offset = shadingData.sNormal * (Dot(wi, shadingData.sNormal) > 0 ? EPSILON : -EPSILON);
			r.init(shadingData.x + offset, wi);
			return (direct + pathTrace(r, pathThroughput, depth + 1, sampler, shadingData.bsdf->isPureSpecular()));
		}
		// return scene->background->evaluate(shadingData, r.dir);
		return scene->background->evaluate(r.dir);
	}

	Colour direct(Ray& r, Sampler* sampler)
	{

		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return computeDirect(shadingData, sampler);
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}

	Colour albedo(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			//return shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
			return computeDirect(shadingData, samplers);
		}
		return scene->background->evaluate(r.dir);
	}
	Colour viewNormals(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}


	// light tracing
	void connectToCamera(Vec3 p, Vec3 n, Colour col) {
		// handle connections to camera
		Camera& camera = scene->camera;
		Vec3 cameraPos = camera.origin;
		Vec3 cameraDir = camera.viewDirection;

		// check if p is on camera
		float u, v;
		bool pOnCamera = scene->camera.projectOntoCamera(p, u, v);
		// starting a light path
		Vec3 wi = (p - cameraPos).normalize(); // direction
		float cosTheta = max(Dot(n, wi), 0.f);
		float cosThetaCam = max(Dot(cameraDir, -wi), 0.f);
		float distanceSq = (cameraPos - p).lengthSq();
		float g = (cosTheta * cosThetaCam) / distanceSq;

		Colour contribution = col * g; //we currently do not conside the sensitivity of cam

		//! Today's process.

		if (pOnCamera) {
			// compute geometry term between p and camera
			Vec3 normal = scene->camera.viewDirection; // camera normal
			Vec3 pos = scene->camera.origin; // camera position
			Vec3 v = (pos - p).normalize(); // vector from p to camera
			float Theta = acos(Dot(n, v));
			float wi = 1 / scene->camera.Afilm * std::pow(cos(Theta),4);
		}
	}

	void lightTrace(Sampler* sampler) {
		// handles starting a light path

		// sample a light source

		// check if it is a area light

		//create a ray from p in direction wi

		// lightTracePath(r, Colour(1.f, 1.f, 1.f), Le, sampler);
	}

	void lightTracePath(Ray& r, Colour pathThroughput, Colour Le, Sampler* sampler) {

	}


	void render() {

		film->incrementSPP();
		int tileSize = 16; // try 16*16 first
		int numTileX = (film->width + tileSize - 1) / tileSize; // calculate number of tiles needed for rendering
		int numTileY = (film->height + tileSize - 1) / tileSize; // calculate number of tiles needed for rendering
		int totalTiles = numTileX * numTileY; // total number of tiles	

		auto renderTiles = [this, tileSize, numTileX, numTileY](int tileIndex, int threadIdx) {
			// locate pixels based on tileSize
			int tileY = tileIndex / numTileX; // get row of tile position
			int tileX = tileIndex % numTileX; // get column of tile position


			unsigned int xStart = tileX * tileSize; // start of each tile
			unsigned int xEnd = min(xStart + tileSize, film->width); // end of each tile(consider the last tile)
			unsigned int yStart = tileY * tileSize;
			unsigned int yEnd = min(yStart + tileSize, film->height);
			//! Test code
			//std::cout << "Processing tile " << tileIndex
			//	<< " [" << xStart << "," << yStart << "]-["
			//	<< xEnd << "," << yEnd << "]\n";


			// »ñÈ¡µ±Ç°Ïß³Ì¶ÔÓ¦µÄËæ»úÊýÉú³ÉÆ÷
			MTRandom* sampler = &samplers[threadIdx];
			// render tiles
			for (unsigned int y = yStart; y < yEnd; ++y) {
				for (unsigned int x = xStart; x < xEnd; ++x) {
					float px = x + 0.5f;
					float py = y + 0.5f;
					Ray ray = scene->camera.generateRay(px, py);

					 
					// Colour col = pathTrace(ray, albedo,depth,sampler);
					Colour col = pathTrace(ray, Colour(1.f,1.f,1.f), 0, sampler);
					//Colour col = computeDirect()
					// Colour col = direct(ray, sampler);
					// Colour col = viewNormals(ray);
					film->splat(px, py, col);

					unsigned char r = (unsigned char)(col.r * 255);
					unsigned char g = (unsigned char)(col.g * 255);
					unsigned char b = (unsigned char)(col.b * 255);
					film->tonemap(x, y, r, g, b);
					canvas->draw(x, y, r, g, b);
				}
			}
		};

		// create and launch threads
		for (int i = 0; i < numProcs; ++i) {
			threads[i] = new std::thread([=]() {
				// we use = to ensure the exclusive resource for each thread
				// Assign task for each thread, i: current thread, 
				for (unsigned int tileIndex = i; tileIndex < totalTiles; tileIndex += numProcs) {
					
					renderTiles(tileIndex, i);
				}
				});
		}

		
		for (int i = 0; i < numProcs; ++i) {
			// check if threads exist and whether thread has already joined
			if (threads[i] && threads[i]->joinable()) {
				threads[i]->join();
				// delete all threads after mission accomplished
				delete threads[i];
				// set thread pointer to nullptr to avoid wild pointer
				threads[i] = nullptr;
			}
		}
	}

	//void render()
	//{
	//	film->incrementSPP();
	//	for (unsigned int y = 0; y < film->height; y++)
	//	{
	//		for (unsigned int x = 0; x < film->width; x++)
	//		{
	//			// raytracing happens here
	//			// pick a point in pixel
	//			float px = x + 0.5f;
	//			float py = y + 0.5f;
	//			Ray ray = scene->camera.generateRay(px, py);
	//			Colour col = viewNormals(ray);
	// Colour col = pathTrace(ray, Colour(1.f,1.f,1.f), 0, sampler);
	//			//Colour col = albedo(ray);
	//			film->splat(px, py, col);
	//			unsigned char r = (unsigned char)(col.r * 255);
	//			unsigned char g = (unsigned char)(col.g * 255);
	//			unsigned char b = (unsigned char)(col.b * 255);
	//			film->tonemap(x, y, r, g, b);
	//			canvas->draw(x, y, r, g, b);
	//		}
	//	}
	//}



	int getSPP()
	{
		return film->SPP;
	}
	void saveHDR(std::string filename)
	{
		film->save(filename);
	}
	void savePNG(std::string filename)
	{
		stbi_write_png(filename.c_str(), canvas->getWidth(), canvas->getHeight(), 3, canvas->getBackBuffer(), canvas->getWidth() * 3);
	}

};

