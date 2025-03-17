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
		// Is surface is specular we cannot computing direct lighting
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		// Compute direct lighting here
		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler)
	{
		// Add pathtracer code here
		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour direct(Ray& r, Sampler* sampler)
	{
		// Compute direct lighting for an image sampler here
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
			return shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
		}
		return scene->background->evaluate(shadingData, r.dir);
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
			std::cout << "Processing tile " << tileIndex
				<< " [" << xStart << "," << yStart << "]-["
				<< xEnd << "," << yEnd << "]\n";


			// 获取当前线程对应的随机数生成器
			MTRandom& sampler = samplers[threadIdx];
			// render tiles
			for (unsigned int y = yStart; y < yEnd; ++y) {
				for (unsigned int x = xStart; x < xEnd; ++x) {
					float px = x + 0.5f;
					float py = y + 0.5f;
					Ray ray = scene->camera.generateRay(px, py);

				
					// Colour col = pathTrace(ray, ... , sampler);

					Colour col = viewNormals(ray);
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