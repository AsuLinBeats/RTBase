#pragma once

#include "Core.h"
#include "Sampling.h"

class Ray
{
public:
	Vec3 o;
	Vec3 dir;
	Vec3 invDir;
	Ray()
	{
	}
	Ray(Vec3 _o, Vec3 _d)
	{
		init(_o, _d);
	}
	void init(Vec3 _o, Vec3 _d)
	{
		o = _o;
		dir = _d;
		invDir = Vec3(1.0f / dir.x, 1.0f / dir.y, 1.0f / dir.z);
	}
	Vec3 at(const float t) const
	{
		return (o + (dir * t));
	}
};

class Plane
{
public:
	Vec3 n;
	float d;
	void init(Vec3& _n, float _d)
	{
		n = _n;
		d = _d;
	}
	// Check if ray intersect with plane
	bool rayIntersect(Ray& r, float& t)
	{
		if (n.x * (r.o.x + t * r.dir.x) + n.y * (r.o.y + t * r.dir.y) + n.z * (r.o.z + t * r.dir.z) + d >= 0)
		{
			return true;
		}
		return false;
	}
};

#define EPSILON 0.001f   // a small float

class Triangle
{
public:
	Vertex vertices[3];
	Vec3 e1; // Edge 1
	Vec3 e2; // Edge 2
	Vec3 n; // Geometric Normal
	float area; // Triangle area
	float d; // For ray triangle if needed
	unsigned int materialIndex;
	void init(Vertex v0, Vertex v1, Vertex v2, unsigned int _materialIndex)
	{
		materialIndex = _materialIndex;
		vertices[0] = v0;
		vertices[1] = v1;
		vertices[2] = v2;
		e1 = vertices[2].p - vertices[1].p;
		e2 = vertices[0].p - vertices[2].p;
		n = e1.cross(e2).normalize();
		area = e1.cross(e2).length() * 0.5f;
		d = Dot(n, vertices[0].p);
	}
	Vec3 centre() const
	{
		// Get the centre of the triangle
		return (vertices[0].p + vertices[1].p + vertices[2].p) / 3.0f;
	}
#if 0
	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const
	{
		// 两步走: 点在平面上, 点在三角形内部.
		// calculate the distance between the ray origin and the plane
		float t1 = (d - Dot(n, r.o)) / Dot(n, r.dir);
		if (t1 < 0) {
			return false;
		}
		// 知道t就可以计算交点
		Vec3 ps = r.o + r.dir * t1; // 和平面的交点
		Vec3 e3 = vertices[1].p - vertices[0].p;

		// 计算交点是否在三角形内部
		Vec3 ee1 = ps - vertices[0].p;
		Vec3 ee2 = ps - vertices[1].p;
		Vec3 ee3 = ps - vertices[2].p;

		Vec3 c1 = ee1.cross(e3);
		Vec3 c2 = ee2.cross(e1);
		Vec3 c3 = ee3.cross(e2);

		if (Dot(c1, c2) > 0 && Dot(c1, c3) > 0)
		{
			return true;
		}

		return true;
	}
#endif

	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const {
		// 计算edge
		Vec3 e1 = vertices[1].p - vertices[0].p;
		Vec3 e2 = vertices[2].p - vertices[0].p;
		Vec3 p = Cross(r.dir, e2);

		float det = Dot(e1, p);
		//if (det == 0.f) {
		//	return false; // situation when ray and triangle are parallel
		//}

		// 浮点数计算存在精度误差,可能导致bug,因此表示浮点数==0时一般规定一个非常小的数然后让其小于那个数得到0
		if (det <= EPSILON) return false;
		float invdet = 1 / det;

		Vec3 T = r.o - vertices[0].p;
		u = Dot(T, p) * invdet;
		if (u > 1.f || u < 0.f) return false;
		Vec3 q = Cross(T, e1);
		v = Dot(r.dir, q) * invdet;
		if ((v < 0.f) || u + v > 1.f) return false;
		t = Dot(e2, q) * invdet;
		return t > 0.f;

	}

	bool rayIntersect1(const Ray& r, float& t, float& u, float& v) const
	{
		float denom = Dot(n, r.dir);
		if (denom == 0) { return false; }
		t = (d - Dot(n, r.o)) / denom;
		if (t < 0) { return false; }
		Vec3 p = r.at(t);
		float invArea = 1.0f / Dot(e1.cross(e2), n);
		u = Dot(e1.cross(p - vertices[1].p), n) * invArea;
		if (u < 0 || u > 1.0f) { return false; }
		v = Dot(e2.cross(p - vertices[2].p), n) * invArea;
		if (v < 0 || (u + v) > 1.0f) { return false; }
		return true;
	}

	void interpolateAttributes(const float alpha, const float beta, const float gamma, Vec3& interpolatedNormal, float& interpolatedU, float& interpolatedV) const
	{
		interpolatedNormal = vertices[0].normal * alpha + vertices[1].normal * beta + vertices[2].normal * gamma;
		interpolatedNormal = interpolatedNormal.normalize();
		interpolatedU = vertices[0].u * alpha + vertices[1].u * beta + vertices[2].u * gamma;
		interpolatedV = vertices[0].v * alpha + vertices[1].v * beta + vertices[2].v * gamma;
	}
	// Add code here
	Vec3 sample(Sampler* sampler, float& pdf)
	{
		return Vec3(0, 0, 0);
	}
	Vec3 gNormal()
	{
		return (n * (Dot(vertices[0].normal, n) > 0 ? 1.0f : -1.0f));
	}
};

class AABB
{
public:
	Vec3 max;
	Vec3 min;
	AABB()
	{
		reset();
	}
	void reset()
	{
		max = Vec3(-FLT_MAX, -FLT_MAX, -FLT_MAX);
		min = Vec3(FLT_MAX, FLT_MAX, FLT_MAX);
	}
	void extend(const Vec3 p)
	{
		max = Max(max, p);
		min = Min(min, p);
	}
	// Add code here
	bool rayAABB(const Ray& r, float& t)
	{

		float txmin = (min.x - r.o.x) / r.dir.x;
		float txmax = (max.x - r.o.x) / r.dir.x;
		if (r.dir.x < 0) std::swap(txmin, txmax);

		float tymin = (min.y - r.o.y) / r.dir.y;
		float tymax = (max.y - r.o.y) / r.dir.y;
		if (r.dir.y < 0) std::swap(tymin, tymax);

		float tzmin = (min.z - r.o.z) / r.dir.z;
		float tzmax = (max.z - r.o.z) / r.dir.z;
		if (r.dir.z < 0) std::swap(tzmin, tzmax);

		float tenter = std::max((txmin, tymin), tzmin);
		float texit = std::min((txmax, tymax), tzmax);

		if (tenter <= texit && texit > 0) {
			return true;
		}

		return false;
	}
	// Add code here
	bool rayAABB(const Ray& r)
	{
		float txmin = (min.x - r.o.x) / r.dir.x;
		float txmax = (max.x - r.o.x) / r.dir.x;
		if (r.dir.x < 0) std::swap(txmin, txmax);

		float tymin = (min.y - r.o.y) / r.dir.y;
		float tymax = (max.y - r.o.y) / r.dir.y;
		if (r.dir.y < 0) std::swap(tymin, tymax);

		float tzmin = (min.z - r.o.z) / r.dir.z;
		float tzmax = (max.z - r.o.z) / r.dir.z;
		if (r.dir.z < 0) std::swap(tzmin, tzmax);

		float tenter = std::max((txmin, tymin), tzmin);
		float texit = std::min((txmax, tymax), tzmax);

		if (tenter <= texit && texit > 0) {
			return true;
		}

		return false;
	}
	// Add code here
	float area()
	{
		Vec3 size = max - min;

		return ((size.x * size.y) + (size.y * size.z) + (size.x * size.z)) * 2.0f;
	}
};

class Sphere
{
public:
	Vec3 centre;
	float radius;
	void init(Vec3& _centre, float _radius)
	{
		centre = _centre;
		radius = _radius;
	}
	// Add code here
	bool rayIntersect(Ray& r, float& t)
	{
		return false;
	}
};

struct IntersectionData
{
	unsigned int ID;
	float t;
	float alpha;
	float beta;
	float gamma;
};

#define MAXNODE_TRIANGLES 8
#define TRAVERSE_COST 1.0f
#define TRIANGLE_COST 2.0f
#define BUILD_BINS 32

class BVHNode
{
public:
	AABB bounds;
	BVHNode* r;
	BVHNode* l;
	// This can store an offset and number of triangles in a global triangle list for example
	// But you can store this however you want!
	// unsigned int offset;
	// unsigned char num;
	BVHNode()
	{
		r = NULL;
		l = NULL;
	}
	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& inputTriangles)
	{
		// Add BVH building code here
	}
	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		// Add BVH Traversal code here
	}
	IntersectionData traverse(const Ray& ray, const std::vector<Triangle>& triangles)
	{
		IntersectionData intersection;
		intersection.t = FLT_MAX;
		traverse(ray, triangles, intersection);
		return intersection;
	}
	bool traverseVisible(const Ray& ray, const std::vector<Triangle>& triangles, const float maxT)
	{
		// Add visibility code here
		return true;
	}
};
