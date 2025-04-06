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
#include<algorithm>
#include<OpenImageDenoise/oidn.hpp>

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom* samplers;
	std::thread** threads;

	std::vector<std::thread> threads1;
	int numProcs;
	std::vector<VPL> VPLs; // store in a vector
	
	EnvironmentMap* envMap = nullptr;
	AreaLight* areaLight = nullptr;
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
			return (direct +pathTrace(r, pathThroughput, depth + 1, sampler, shadingData.bsdf->isPureSpecular()));
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
		VPLs.clear();

		for (int i = 0; i < N_VPLs; i++) {
			float lightpdf;
			Light* light = scene->sampleLight(sampler, lightpdf);
			if (!light || lightpdf == 0.f) {
				continue;
			}
			// sample light position, direction
			float pdfPosition = 0.f;
			float pdfDirection = 0.f;
			Vec3 p = light->samplePositionFromLight(sampler, pdfPosition);
			Vec3 wi = light->sampleDirectionFromLight(sampler, pdfDirection);
			// make sure values are valid
			if (pdfPosition < 1e-6f || pdfDirection < 1e-6f) {
				continue;
			}

			// get light normal and calculate costheta
			Vec3 lightNormal = light->normal(wi);
			float cosThetaLight = Dot(lightNormal, wi);
			if (cosThetaLight <= 0.f) {
				continue;
			}
			cosThetaLight = max(cosThetaLight, 0.f);


			Colour Le = light->evaluate(-wi);
			// calculate flux
			Colour flux = Le * cosThetaLight / (lightpdf * pdfPosition * pdfDirection);
			// init ray
			Ray r(p, wi);

			VPLTracePath(r, Colour(1.f, 1.f, 1.f), flux, sampler, 1);
		
		}
	}

	void VPLTracePath(Ray& r, Colour pathThroughput , Colour flux, Sampler* sampler, int depth) {
		// the intersection between ray and scene
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t >= FLT_MAX) {
			return;
		}

		// shading data at intersection point
		ShadingData shadingData = scene->calculateShadingData(intersection, r);

		// create VPL when intersection
		VPL vpl;
		vpl.position = shadingData.x;
		vpl.normal = shadingData.sNormal;
		// BSDF* flux
		vpl.pathThroughput = pathThroughput * flux;
		VPLs.push_back(vpl);

		// stop condition
		if (depth >= 10) {
			return;
		}

		float rouletteProbability = min(pathThroughput.Lum(), 0.9f);
		if (sampler->next() >= rouletteProbability) {
			return;
		}

		if (rouletteProbability > 1e-6f) {
			pathThroughput = pathThroughput / rouletteProbability;
		}

		//. sample new direction from current BSDF
		Colour bsdf;
		float pdf;
		Vec3 newWi = shadingData.bsdf->sample(shadingData, sampler, bsdf, pdf);
		if (pdf < 1e-6f) {
			return;
		}
		float cosTheta = fabsf(Dot(newWi, shadingData.sNormal));
		// pathThroughput
		pathThroughput = pathThroughput * bsdf * cosTheta / pdf;

		// avoid self-block
		Vec3 offset = shadingData.sNormal * (Dot(newWi, shadingData.sNormal) > 0.f ? EPSILON : -EPSILON);
		r.init(shadingData.x + offset, newWi);

		// track next path recursively
		VPLTracePath(r, pathThroughput, flux, sampler, depth + 1);
	}


	Colour computeInDirectVPL(ShadingData shadingData, Sampler* sampler)
	{
		Colour indirect(0.0f, 0.0f, 0.0f);

		for (const VPL& vpl : VPLs) {
			// direction from intersection to VPL
			Vec3 d = vpl.position - shadingData.x;
			float distSq = d.lengthSq();
			Vec3 wi = d.normalize();

			// cosine at intersection
			float G1 = max(Dot(shadingData.sNormal, wi), 0.f);
			// VPL normal cosine
			float G2 = max(-Dot(vpl.normal, wi), 0.f);
			float GTerm = (G1 * G2) / distSq;

			// visibiklity check
			if (GTerm > 0 && scene->visible(shadingData.x, vpl.position)) {
				// BSDF
				Colour bsdfVal = shadingData.bsdf->evaluate(shadingData, wi);
				// BSDF * VPL * geometry
				indirect =  indirect + bsdfVal * vpl.pathThroughput * GTerm;
			}
		}

		if (!VPLs.empty())
			indirect = indirect / float(VPLs.size());

		return indirect;
	}



	Colour directVPL(Ray& r, Sampler* sampler)
	{

		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX) {
			// if shoot light, return
			if (shadingData.bsdf->isLight()) {
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			// direct light
			Colour direct = computeDirect(shadingData, sampler); // direct light do not need VPL
			// indirect VPL light
			Colour indirect = computeInDirectVPL(shadingData,sampler);
			return direct + indirect;
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}

	Colour pathTraceVPL(Ray& r, Colour pathThroughput, int depth, Sampler* sampler, bool canHitLight = true)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);

		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				if (canHitLight)
					return pathThroughput * shadingData.bsdf->emit(shadingData, shadingData.wo);
				else
					return Colour(0.0f, 0.0f, 0.0f);
			}

			// direct light
			Colour direct = pathThroughput * computeDirect(shadingData, sampler);

			// indirect VPL light
			Colour indirect(0.0f, 0.0f, 0.0f);
			if (!shadingData.bsdf->isPureSpecular())
			{
				indirect = pathThroughput * computeInDirectVPL(shadingData, sampler);
			}

			if (depth > MAX_DEPTH)
			{
				return direct + indirect;
			}

			float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);
			if (sampler->next() < russianRouletteProbability)
			{
				pathThroughput = pathThroughput / russianRouletteProbability;
			}
			else
			{
				return direct + indirect;
			}

			Colour bsdf;
			float pdf;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, bsdf, pdf);
			if (pdf < 1e-6f)
			{
				return direct + indirect;
			}

			// upgrade paththrought
			pathThroughput = pathThroughput * bsdf * fabsf(Dot(wi, shadingData.sNormal)) / pdf;

			Vec3 offset = shadingData.sNormal * (Dot(wi, shadingData.sNormal) > 0 ? EPSILON : -EPSILON);
			r.init(shadingData.x + offset, wi);

			return (direct + indirect + pathTrace(r, pathThroughput, depth + 1, sampler, shadingData.bsdf->isPureSpecular()));
		}

		// cases when no item shoot
		return scene->background->evaluate(r.dir);
	}

	//! light tracing
	void connectToCamera(Vec3 p, Vec3 n, Colour col) {
		Camera& camera = scene->camera;
		Vec3 cameraPos = camera.origin;
		Vec3 cameraDir = camera.viewDirection;

		// project to camera
		float u, v;
		bool pOnCamera = scene->camera.projectOntoCamera(p, u, v);
		if (!pOnCamera) return;

		// direction from p to camera
		Vec3 wi = (cameraPos - p).normalize();

		// geometry item
		float cosTheta = max(Dot(n, wi), 0.f);
		float cosThetaCam = max(Dot(cameraDir, -wi), 0.f);
		float distanceSq = (cameraPos - p).lengthSq();
		float g = (cosTheta * cosThetaCam) / distanceSq;

		// avoid self-block
		Vec3 offsetP = p + n * EPSILON;
		//if (g > 0.f) {
			
			//if (scene->visible(offsetP, scene->camera.origin)) {
		
			//	float contri = camera.sensitivity / scene->camera.Afilm;
			//	film->splat(u, v, col * contri * g);
			//}
		//}
	}
	void lightTrace(Sampler* sampler) {
		// sample light
		int depth = 10;
		float lightpdf;
		Light* light = scene->sampleLight(sampler, lightpdf);
		if (!light || lightpdf == 0.f) {
			std::cout << "Light sampling failed or lightpdf == 0\n";
			return;
		}

		// get light pos and dir, and their pdf
		float pdfPosition = 0.f;
		float pdfDirection = 0.f;
		Vec3 p = light->samplePositionFromLight(sampler, pdfPosition);
		Vec3 wi = light->sampleDirectionFromLight(sampler, pdfDirection);

		// avoid divided by 0
		if (pdfPosition < 1e-6f || pdfDirection < 1e-6f) {
			std::cout << "Invalid PDF values: pdfPosition=%.3e, pdfDirection=%.3e\n" << pdfPosition << pdfDirection;
			return;
		}

		// get light normal according to sample directio 
		Vec3 lightNormal = light->normal(wi);
		float cosThetaLight = Dot(lightNormal, wi);
		if (cosThetaLight <= 0.f) {
			std::cout << "Invalid light sampling: cosThetaLight <= 0\n";
			return;
		}
		cosThetaLight = max(cosThetaLight, 0.f);

		Colour Le = light->evaluate(-wi);
		Colour contribution = Le * cosThetaLight / (lightpdf * pdfPosition * pdfDirection);

		connectToCamera(p, lightNormal, contribution);

		// create and trace light
		Ray r(p, wi);
		lightTracePath(r, 1, Colour(1.f, 1.f, 1.f), Le, sampler);
	}


	const int MAX_DEPTH_LIGHT = 10;
	void lightTracePath(Ray& r, int depth, Colour pathThroughput, Colour Le, Sampler* sampler) {
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);

		if (shadingData.t < FLT_MAX) {
			if (depth > MAX_DEPTH_LIGHT)
				return;

			// connect to camera when intersection
			connectToCamera(shadingData.x, shadingData.sNormal, pathThroughput * Le);


			float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);
			if (sampler->next() < russianRouletteProbability) {
				pathThroughput = pathThroughput / russianRouletteProbability;
			}
			else {
				return;
			}

			// sample bsdf and get bsdf value and pdf
			Colour bsdf;
			float pdf;
			Vec3 newWi = shadingData.bsdf->sample(shadingData, sampler, bsdf, pdf);
			if (pdf < 1e-6f) {
				printf("Invalid BSDF PDF at depth %d: pdf=%.3e\n", depth, pdf);
				return;
			}

			float cosTheta = fabsf(Dot(newWi, shadingData.sNormal));
			pathThroughput = pathThroughput * bsdf * cosTheta / pdf;

			// use epsilon to avoid block itself
			Vec3 offset = shadingData.sNormal * (Dot(newWi, shadingData.sNormal) > 0.f ? EPSILON : -EPSILON);
			r.init(shadingData.x + offset, newWi);
			lightTracePath(r, depth + 1, pathThroughput, Le, sampler);
		}
	}





	// TODO WHERE IS THE BUG?
	void renderwdawd() {
		//! LIGHT TRACING Render
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

			MTRandom* sampler = &samplers[threadIdx];

			// render tiles
			for (int s = 0; s < 10; s++) {
				
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

	// Denoise
	void denoise1(oidn::DeviceRef& device) {
		oidn::FilterRef filter = device.newFilter("RT");

		filter.setImage("color", film->colourBuffer,
			oidn::Format::Float3, film->width, film->height);
		filter.setImage("output", film->outputBuffer,
			oidn::Format::Float3, film->width, film->height);
		filter.set("hdr", true);
		filter.commit();

		filter.execute();

		const char* errorMessage;
		if (device.getError(errorMessage) != oidn::Error::None)
			std::cerr << "Error: " << errorMessage << std::endl;

		std::vector<float> displayBuffer(film->width * film->height * 3);
		film->outputBuffer.read(
			static_cast<size_t>(0),  
			static_cast<size_t>(film->width * film->height * 3 * sizeof(float)), 
			static_cast<void*>(displayBuffer.data()) 
		);

		for (int i = 0; i < film->width * film->height; ++i) {

			float r = displayBuffer[i * 3 + 0];
			float g = displayBuffer[i * 3 + 1];
			float b = displayBuffer[i * 3 + 2];

			film->film[i] = Colour(r, g, b);
		}
	}

	void tonemap() {
		for (int y = 0; y < film->height; ++y) {
			for (int x = 0; x < film->width; ++x) {
				unsigned char rc, gc, bc;

				film->tonemap(x, y, rc, gc, bc);
				canvas->draw(x, y, rc, gc, bc);
			}
		}
	}

	void prepareDenoiserInput(Film* film) {
		float scale = 1.0f;
		std::vector<float> tempBuffer(film->width * film->height * 3);

		for (int i = 0; i < film->width * film->height; ++i) {
			Colour c = film->film[i] * scale;
			tempBuffer[i * 3 + 0] = c.r;
			tempBuffer[i * 3 + 1] = c.g;
			tempBuffer[i * 3 + 2] = c.b;
		}


		film->colourBuffer.write(
			0, // byteOffset
			tempBuffer.size() * sizeof(float), // byteSize
			reinterpret_cast<const void*>(tempBuffer.data()) // srcHostPtr
		);

	}

	// TODO THIS IS PATH TRACING RENDER WITH INTEL DENOISER
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


			MTRandom* sampler = &samplers[threadIdx];
			
			// render tiles
			for (unsigned int y = yStart; y < yEnd; ++y) {
				for (unsigned int x = xStart; x < xEnd; ++x) {
					float px = x + 0.5f;
					float py = y + 0.5f;
					Ray ray = scene->camera.generateRay(px, py);

					 
					Colour col = pathTrace(ray, Colour(1.f, 1.f, 1.f), 0,sampler);
					// Colour col = pathTraceVPL(ray, Colour(1.f,1.f,1.f), 0, sampler);



					// Colour col = lightTrace(sampler);
					//Colour col = computeDirect()
					// Colour col = direct(ray, sampler);
					// Colour col = viewNormals(ray);
					 //film->splat1(px, py, col);
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
#if 0
		// Denoise component
		oidn::DeviceRef device = oidn::newDevice();
		
		device.commit();
		// initialse essential devices
		film->initBuffer(device);
		prepareDenoiserInput(film);
		// launch denoiser
		denoise1(device);
		// apply tone map
		tonemap();
#endif
	}
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Adaptive tile sampling
	float computeTileVariance(int xStart, int yStart, int xEnd, int yEnd, float mean) {
		float sumSqDiff = 0.0f;
		int count = 0;
		for (int y = yStart; y < yEnd; ++y) {
			for (int x = xStart; x < xEnd; ++x) {
				Colour col = film->getColour(x, y);
				float diff = col.r - mean;
				sumSqDiff += diff * diff;
				count++;
			}
		}
		return (count > 1) ? (sumSqDiff / (count - 1)) : 0.0f;
	}


	//void renderA() {
	//	film->incrementSPP();
	//	int tileSize = 20; // try 16*16 first
	//	int numTileX = (film->width + tileSize - 1) / tileSize; // calculate number of tiles needed for rendering
	//	int numTileY = (film->height + tileSize - 1) / tileSize; // calculate number of tiles needed for rendering
	//	int totalTiles = numTileX * numTileY; // total number of tiles	

	//	auto renderTiles = [this, tileSize, numTileX, numTileY](int tileIndex, int threadIdx) {
	//		int tileY = tileIndex / numTileX;
	//		int tileX = tileIndex % numTileX;
	//		unsigned int xStart = tileX * tileSize;
	//		unsigned int xEnd = min(xStart + tileSize, film->width);
	//		unsigned int yStart = tileY * tileSize;
	//		unsigned int yEnd = min(yStart + tileSize, film->height);

	//		MTRandom* sampler = &samplers[threadIdx];
	//		int tilePixelCount = (xEnd - xStart) * (yEnd - yStart);

	//		// 自适应采样循环
	//		while (film->tileSampleCount[tileIndex] < film->maxSPP) {
	//			// 渲染当前分块的所有像素一次
	//			for (unsigned int y = yStart; y < yEnd; ++y) {
	//				for (unsigned int x = xStart; x < xEnd; ++x) {
	//					float px = x + 0.5f;
	//					float py = y + 0.5f;
	//					Ray ray = scene->camera.generateRay(px, py);

	//					Colour col = pathTrace(ray, Colour(1.f, 1.f, 1.f), 0, sampler);
	//					film->tileColorSum[tileIndex] = film->tileColorSum[tileIndex] + col;
	//				}
	//			}
	//			film->tileSampleCount[tileIndex] += film->baseSPP;

	//			// 计算分块方差
	//			float meanR = film->tileColorSum[tileIndex].r / film->tileSampleCount[tileIndex];
	//			float variance = computeTileVariance(xStart, yStart, xEnd, yEnd, meanR);
	//			film->tileVariance[tileIndex] = variance;

	//			// 判断是否继续采样
	//			if (variance < film->varianceThreshold) {
	//				film->tileNeedsMoreSamples[tileIndex] = false;
	//				break;
	//			}
	//			else {
	//				film->tileNeedsMoreSamples[tileIndex] = true;
	//			}
	//		}

	//		// 将累积颜色写入最终像素
	//		for (unsigned int y = yStart; y < yEnd; ++y) {
	//			for (unsigned int x = xStart; x < xEnd; ++x) {
	//				int pixelIdx = y * film->width + x;
	//				Colour finalColor = film->tileColorSum[tileIndex] / film->tileSampleCount[tileIndex];
	//				film->splat1(x + 0.5f, y + 0.5f, finalColor);
	//			}
	//		}
	//		};

	//	// create and launch threads
	//	for (int i = 0; i < numProcs; ++i) {
	//		std::vector<int> highVarianceTiles;
	//		for (int i = 0; i < totalTiles; ++i) {
	//			if (film->tileVariance[i] > film->varianceThreshold) {
	//				highVarianceTiles.push_back(i);
	//			}
	//		}

	//		threads[i] = new std::thread([=]() {

	//			// we use = to ensure the exclusive resource for each thread
	//			// Assign task for each thread, i: current thread, 
	//			while (!highVarianceTiles.empty()) {
	//				int tileIndex = highVarianceTiles.back();
	//				highVarianceTiles.pop_back();
	//				renderTiles(tileIndex, i);
	//			}
	//		}
	//	}


	//	for (int i = 0; i < numProcs; ++i) {
	//		// check if threads exist and whether thread has already joined
	//		if (threads[i] && threads[i]->joinable()) {
	//			threads[i]->join();
	//			// delete all threads after mission accomplished
	//			delete threads[i];
	//			// set thread pointer to nullptr to avoid wild pointer
	//			threads[i] = nullptr;
	//		}
	//	}
	//	// Denoise component
	//	oidn::DeviceRef device = oidn::newDevice();
	//	// device.set("setAffinity", true);  // 优化多线程
	//	device.commit();
	//	// initialse essential devices
	//	film->initBuffer(device);
	//	prepareDenoiserInput(film);
	//	// launch denoiser
	//	denoise1(device);
	//	// apply tone map
	//	tonemap();

	//}


////////////////////////////////////////////////////////////////////////////////////////////////////////////
Colour computeDirectMIS(ShadingData shadingData, Sampler* sampler)
{
	Colour result;
	if (shadingData.bsdf->isPureSpecular())
	{
		return Colour(0.f,0.f,0.f);
	}

	float lightpmf;
	Light* light = scene->sampleLight(sampler, lightpmf);

	float lightpdf;
	Colour emitted(0.f,0.f,0.f);
	Vec3 lightsample = light->sample(shadingData, sampler, emitted, lightpdf);

	Vec3 wi;
	float pdf = lightpmf * lightpdf;
	float GTerm = 0.f;
	float pdfWeight = 0.f;

	if (light->isArea())
	{
		if (pdf <= 0) { return Colour(0.f,0.f,0.f); }

		wi = lightsample - shadingData.x;
		float distanceSq = wi.lengthSq();
		wi = wi.normalize();
		Vec3 lightNormal = light->normal(shadingData, wi);
		float cosThetaShading = max(Dot(wi, shadingData.sNormal), 0.0f);
		float cosThetaLight = max(-Dot(wi, lightNormal), 0.0f);
		GTerm = (cosThetaShading * cosThetaLight) / distanceSq;

		// Convert area PDF to solid angle PDF for MIS
		float convertFactor = (cosThetaLight > 0.0f) ? (distanceSq / cosThetaLight) : 0.0f;
		float lightPdfSolidAngle = lightpdf * convertFactor;
		float lightPdfTotal = lightpmf * lightPdfSolidAngle;

		bool isVisible = scene->visible(shadingData.x, lightsample);
		if (isVisible && GTerm > 0.0f)
		{
			float bsdfPdf = shadingData.bsdf->PDF(shadingData, wi);
			float misWeight = (lightPdfTotal * lightPdfTotal) / (lightPdfTotal * lightPdfTotal + bsdfPdf * bsdfPdf);
			Colour bsdfVal = shadingData.bsdf->evaluate(shadingData, wi);
			result = bsdfVal * emitted * GTerm * misWeight / (lightpmf * lightpdf);
			return result;
		}
		else
		{
			return Colour(0.f,0.f,0.f);
		}
	}
	else
	{
		wi = (lightsample - shadingData.x).normalize();
		bool isVisible = scene->visible(shadingData.x, shadingData.x + wi * 10000.0f);
		if (!isVisible)
		{
			return Colour(0.f,0.f,0.f);
		}

		GTerm = max(Dot(wi, shadingData.sNormal), 0.0f);
		if (GTerm <= 0.0f)
		{
			return Colour(0.f,0.f,0.f);
		}

		float bsdfPdf = shadingData.bsdf->PDF(shadingData, wi);
		pdfWeight = pdf / (pdf + bsdfPdf);
		Colour bsdfVal = shadingData.bsdf->evaluate(shadingData, wi);
		return (bsdfVal * emitted * GTerm / pdf) * pdfWeight;
	}
}

Colour pathTraceMIS(Ray& r, Colour pathThroughput, int depth, Sampler* sampler, bool canHitLight = true)
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
		Colour direct = pathThroughput * computeDirectMIS(shadingData, sampler);
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
		return (direct + pathTraceMIS(r, pathThroughput, depth + 1, sampler, shadingData.bsdf->isPureSpecular()));
	}
	// return scene->background->evaluate(shadingData, r.dir);
	return scene->background->evaluate(r.dir);
}

	void renderMIS() {
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
					Colour C = Colour(1.f, 1.f, 1.f);
					Colour col = pathTraceMIS(ray, C, 0, sampler);



					// Colour col = lightTrace(sampler);
					//Colour col = computeDirect()
					// Colour col = direct(ray, sampler);
					// Colour col = viewNormals(ray);
					film->splat1(px, py, col);


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
		// Denoise component
		//oidn::DeviceRef device = oidn::newDevice();
		//// device.set("setAffinity", true);  // 优化多线程
		//device.commit();
		//// initialse essential devices
		//film->initBuffer(device);
		//prepareDenoiserInput(film);
		//// launch denoiser
		//denoise1(device);
		// apply tone map
		//tonemap();

	}

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

