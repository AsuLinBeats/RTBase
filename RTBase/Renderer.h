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
#include<OpenImageDenoise/oidn.hpp>

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom* samplers;
	std::thread** threads;
	int numProcs;
	std::vector<VPL> VPLs; // store in a vector
	
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
	const int MAX_DEPTH = 12; // generally need 3-5 bounces


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


			float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);

			if (sampler->next() < russianRouletteProbability)
			{
				pathThroughput = pathThroughput / russianRouletteProbability;
			}
			else
			{
				return direct;
			}


			Colour bsdf;
			float pdf;
			// Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
			
			 //float pdf;
			 Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, bsdf, pdf);
			 
		
			pathThroughput = pathThroughput * bsdf * fabsf(Dot(wi, shadingData.sNormal)) / pdf;
			// r.init(shadingData.x + (wi * EPSILON), wi);
			Vec3 offset = shadingData.sNormal * (Dot(wi, shadingData.sNormal) > 0 ? EPSILON : -EPSILON);
			// TODO TO FIX SHADOW ACNE, TRY A FIXED OFFSET
			//Vec3 offset = wi ; // not work....
			
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

	// VPL
	void traceVPLs(Sampler* sampler, int N_VPLs) {
		for (VPL& vpl : VPLs) { // 使用引用以修改原VPL对象
			// 获取VPL的发射光强度vpl、概率质量函数和概率密度函数
			float pmf;
			float pdf = vpl.pdf;
			Colour emitted = Colour(0.f,0.f,0.f); // Use fix number for now

			
			if (pdf <= 0.0f || N_VPLs <= 0) {
				vpl.Le = Colour(0.0f,0.f,0.f);
				continue;
			}

			// Le = emitted / (pmf * pdf * N_VPLs)
			
			vpl.Le = emitted / (pdf * static_cast<float>(N_VPLs));


		}
	}

	void VPLTracePath(Ray& r, Colour pathThroughput, Colour Le, Sampler* sampler) {


	}
	// Instant radiosity
	// light tracing
	void connectToCamera(Vec3 p, Vec3 n, Colour col) {
		// handle connections to camera
		Camera& camera = scene->camera;
		Vec3 cameraPos = camera.origin;
		Vec3 cameraDir = camera.viewDirection;

		// project scene to camera 
		float u, v;
		bool pOnCamera = scene->camera.projectOntoCamera(p, u, v);
		// check if p is on camera
		if (!pOnCamera) return;
		// starting a light path
		Vec3 wi = (p - cameraPos).normalize(); // direction



		float cosTheta = max(Dot(n, wi), 0.f);
		float cosThetaCam = max(Dot(cameraDir, -wi), 0.f);
		float distanceSq = (cameraPos - p).lengthSq();
		float g = (cosTheta * cosThetaCam) / distanceSq;

		float pixelSensitivity = std::pow(cosThetaCam, 4.0f);
		// Colour contribution = col * g; //we currently do not conside the sensitivity of cam

		Colour contribution = col * (cosTheta * cosThetaCam / distanceSq) * pixelSensitivity;
		
		// Impact film
		
	}



	void lightTrace(Sampler* sampler) {
		// handles starting a light path
		int depth = 10; // control bounce
		float lightpdf;
		// sample a light source
		Light* light = scene->sampleLight(sampler, lightpdf);
		if (lightpdf == 0 || !light) { return; }

		float pdfDirection = 0.f;
		float pdfPosition = 0.f;
		Vec3 p = light->samplePositionFromLight(sampler, pdfPosition);
		Vec3 wi = light->sampleDirectionFromLight(sampler, pdfDirection);


		// Colour Le = light->evaluate(-wi);
		Colour Le = Colour(1.f,1.f,1.f); // DEBUG ONLY
		float cosThetaLight = Dot(p,wi);
		Colour contribution = Le * cosThetaLight / (lightpdf * pdfPosition * pdfDirection);
		connectToCamera(p, light->normal(wi), contribution); //! fix normal method to make it return a vector


		////create a ray from p in direction wi
		Ray r(p, wi); // create a ray
		
		lightTracePath(r, 1, Colour(1.f, 1.f, 1.f), Le, sampler);
	}

	const int MAX_DEPTH_LIGHT = 10;

	void lightTracePath(Ray& r, int depth, Colour pathThroughput, Colour Le, Sampler* sampler) {
		// depth: initial depth is 0 or 1
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);

		if (shadingData.t < FLT_MAX)
		{
			
			if (depth > MAX_DEPTH_LIGHT)
				// if (depth > 4)
			{
				return;
			}
			connectToCamera(shadingData.x, shadingData.sNormal, pathThroughput*Le);

			float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);

			if (sampler->next() < russianRouletteProbability)
			{
				pathThroughput = pathThroughput / russianRouletteProbability;
			}
			else
			{
				return;
			}

			Colour bsdf;
			float pdf;
			//Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler->next(), sampler->next());
			 //float pdf;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, bsdf, pdf);
			float cosTheta = std::abs(Dot(shadingData.sNormal, wi));

			pathThroughput = pathThroughput * bsdf * fabsf(Dot(wi, shadingData.sNormal)) / pdf;
			// r.init(shadingData.x + (wi * EPSILON), wi);
			Vec3 offset = shadingData.sNormal * (Dot(wi, shadingData.sNormal) > 0 ? EPSILON : -EPSILON);
			// TODO TO FIX SHADOW ACNE, TRY A FIXED OFFSET

			Vec3 wo = -r.dir; // 确保使用正确出射方向
			

			r.init(shadingData.x + offset, wi);
			lightTracePath(r, depth + 1 ,pathThroughput, Le, sampler);
		}

	}


	void instantRadiosity() {

	}


	void pathTraceMIS() {

	}
	void render1() {
		//! LIGHT TRACING
		film->incrementSPP();
		int tileSize = 20; // try 16*16 first
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
			for (int s = 0; s < tileSize; ++s) {
				// 生成光源路径并累积贡献到胶片
				lightTrace(sampler);
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


	void render() {

		film->incrementSPP();
		int tileSize = 20; // try 16*16 first
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

					// Colour col = lightTrace(sampler);
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

