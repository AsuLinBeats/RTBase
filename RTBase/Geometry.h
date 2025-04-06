#pragma once

#include "Core.h"
#include "Sampling.h"
#include<algorithm>

Vec3 operator*(const float& num, Vec3& vec) {
	return Vec3(vec.x * num, vec.y * num, vec.z * num);
}

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

	bool rayIntersect(const Ray& r, float& t, float& u, float& v) const
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
		// Use 2 random number to generate barycentric coordinates, and sample a point inside thrangle using it.
		// pdf here is 1/ area, where area is given
		float r1 = sampler->next();
		float r2 = sampler->next();
		// barycentric coor
		float alpha = 1 - sqrt(r1);
		float beta = r2 * sqrt(r1);
		float gamma = 1 - alpha - beta;
		pdf = 1 / area; // change pdf
		return Vec3(alpha * vertices[0].p + beta * vertices[1].p + gamma * vertices[2].p);
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
	bool rayAABB1(const Ray& r, float& t)
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

		
		float tenter = std::max({ txmin, tymin, tzmin });
		float texit = std::min({ txmax, tymax, tzmax });

		if (tenter <= texit && texit > 0) {
			t = tenter;
			return true;
		}
		return false;
	}

	bool rayAABB(const Ray& r) const {
		float tmin = 0.0f;
		float tmax = FLT_MAX;

		for (int i = 0; i < 3; ++i) {
			if (fabs(r.dir[i]) < 1e-6) { 
				if (r.o[i] < min[i] || r.o[i] > max[i])
					return false;
				continue;
			}

			float invD = 1.0f / r.dir[i];
			float t0 = (min[i] - r.o[i]) * invD;
			float t1 = (max[i] - r.o[i]) * invD;

			if (invD < 0.0f)
				std::swap(t0, t1);

			tmin = fmax(t0, tmin);
			tmax = fmin(t1, tmax);

			if (tmin > tmax)
				return false;
		}
		return tmin < tmax && tmax >= 0.0f;
	}
	
	bool rayAABB(const Ray& r, float& t)
	{
		if (min.x > max.x || min.y > max.y || min.z > max.z) return false;

	
		auto safeDivide = [](float numerator, float denominator) -> float {
			if (denominator == 0) return (numerator >= 0) ? FLT_MAX : -FLT_MAX;
			return numerator / denominator;
			};

		float txmin = safeDivide(min.x - r.o.x, r.dir.x);
		float txmax = safeDivide(max.x - r.o.x, r.dir.x);
		if (r.dir.x < 0) std::swap(txmin, txmax);

		float tymin = safeDivide(min.y - r.o.y, r.dir.y);
		float tymax = safeDivide(max.y - r.o.y, r.dir.y);
		if (r.dir.y < 0) std::swap(tymin, tymax);

		float tzmin = safeDivide(min.z - r.o.z, r.dir.z);
		float tzmax = safeDivide(max.z - r.o.z, r.dir.z);
		if (r.dir.z < 0) std::swap(tzmin, tzmax);


		float tenter = std::max({ txmin, tymin, tzmin });
		float texit = std::min({ txmax, tymax, tzmax });

		if (tenter <= texit && texit >= 0) {
			t = tenter;
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
		Vec3 oc = r.o - centre;
		float a = Dot(r.dir, r.dir);
		float b = 2.0f * Dot(oc, r.dir);
		float c = Dot(oc, oc) - radius * radius;
		float delta = b * b - 4 * a * c;

		// cases no interaction
		if (delta < 0) {
			return false;
		}

		float sqrtDelta = sqrt(delta);
		float t0 = (-b - sqrtDelta) / (2 * a);
		float t1 = (-b + sqrtDelta) / (2 * a);

		// choose minumim value
		if (t0 > 0) {
			t = t0;
		} else if (t1 > 0) { 
			t = t1;
		} else return false; // interaction is at negative dir

		return true;
		
	}
};

struct IntersectionData
{
	unsigned int ID; // unique ID for intersection
	float t; // ray parameter
	// barycentric coordinate
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
	unsigned int offset;  // start index
	unsigned int num;    // num of triangle in a node
	bool isLeaf;         

	BVHNode()
	{
		r = nullptr;
		l = nullptr;
		offset = 0;
		num = 0;
		isLeaf = false;
	}

	~BVHNode()
	{
		if (r) delete r;
		if (l) delete l;
	}

	void build(std::vector<Triangle>& inputTriangles, std::vector<Triangle>& triangles)
	{
		// 如果是叶子节点
		if (inputTriangles.size() <= MAXNODE_TRIANGLES) {
			isLeaf = true;
			offset = (unsigned int)triangles.size();  // 设置三角形的起始索引
			num = (unsigned int)inputTriangles.size();

			// 将三角形添加到全局三角形列表中
			triangles.insert(triangles.end(), inputTriangles.begin(), inputTriangles.end());

			// 计算包围盒
			bounds.reset();
			for (const auto& triangle : inputTriangles) {
				bounds.extend(triangle.vertices[0].p);
				bounds.extend(triangle.vertices[1].p);
				bounds.extend(triangle.vertices[2].p);
			}
			return;
		}

		bounds.reset();
		for (const auto& triangle : inputTriangles) {
			bounds.extend(triangle.vertices[0].p);
			bounds.extend(triangle.vertices[1].p);
			bounds.extend(triangle.vertices[2].p);
		}

		// SAH参数（同原代码）
		const int BUCKETS = 12;
		const float C_trav = 1.0f;
		const float C_intersect = 2.0f;
		float minCost = FLT_MAX;
		int bestAxis = -1;
		float bestSplit = 0;

		// 遍历三个轴（X/Y/Z）
		for (int axis = 0; axis < 3; ++axis) {
			// 改用索引避免指针失效
			std::vector<std::pair<float, size_t>> sortedCenters;
			sortedCenters.reserve(inputTriangles.size());
			for (size_t i = 0; i < inputTriangles.size(); ++i) {
				Vec3 center = inputTriangles[i].centre();
				sortedCenters.emplace_back(center[axis], i);
			}
			std::sort(sortedCenters.begin(), sortedCenters.end());
			for (int b = 1; b < BUCKETS; ++b) {
				size_t splitIndex = (b * sortedCenters.size()) / BUCKETS;
				if (splitIndex == 0 || splitIndex >= sortedCenters.size()) continue;

				float splitValue = sortedCenters[splitIndex].first;

				AABB leftBox, rightBox;
				leftBox = this->bounds;
				rightBox = this->bounds;
				leftBox.reset();
				rightBox.reset();

				for (size_t i = 0; i < splitIndex; ++i) {
					const Triangle& tri = inputTriangles[sortedCenters[i].second];
					leftBox.extend(tri.vertices[0].p);
					leftBox.extend(tri.vertices[1].p);
					leftBox.extend(tri.vertices[2].p);
				}
				for (size_t i = splitIndex; i < sortedCenters.size(); ++i) {
					const Triangle& tri = inputTriangles[sortedCenters[i].second];
					rightBox.extend(tri.vertices[0].p);
					rightBox.extend(tri.vertices[1].p);
					rightBox.extend(tri.vertices[2].p);
				}

				// 计算SAH成本
				//float cost = C_trav +
				//	(leftBox.area() * splitIndex + rightBox.area() * (sortedCenters.size() - splitIndex)) * C_intersect / bounds.area();

				float parentArea = bounds.area();
				if (parentArea < 1e-6f) {
					parentArea = 1e-6f;
				}
				float cost = C_trav +
					(leftBox.area() * splitIndex + rightBox.area() * (sortedCenters.size() - splitIndex)) * C_intersect / bounds.area();

				if (cost < minCost) {
					minCost = cost;
					bestAxis = axis;
					bestSplit = splitValue;
				}
			}
		}

		// split triangle
		std::vector<Triangle> leftTriangles, rightTriangles;
		for (auto& tri : inputTriangles) {
			float centerValue = tri.centre()[bestAxis];
			if (centerValue <= bestSplit) {
				leftTriangles.push_back(tri);
			}
			else {
				rightTriangles.push_back(tri);
			}
		}

		// special case
		if (leftTriangles.empty() || rightTriangles.empty()) {
			bool splitSuccess = false;
			for (int retryAxis = 0; retryAxis < 3 && !splitSuccess; ++retryAxis) {
				if (retryAxis == bestAxis) continue;

				
				std::sort(inputTriangles.begin(), inputTriangles.end(),
					[retryAxis](const Triangle& a, const Triangle& b) {
						return a.centre()[retryAxis] < b.centre()[retryAxis];
					});

				
				size_t mid = inputTriangles.size() / 2;
				auto left = inputTriangles.begin() + mid;
				auto right = inputTriangles.begin() + mid;

				if (mid > 0 && mid < inputTriangles.size()) {
					leftTriangles.assign(inputTriangles.begin(), left);
					rightTriangles.assign(right, inputTriangles.end());
					splitSuccess = true;
				}
			}

			if (!splitSuccess) {
				
				size_t mid = inputTriangles.size() / 2;
				leftTriangles.assign(inputTriangles.begin(), inputTriangles.begin() + mid);
				rightTriangles.assign(inputTriangles.begin() + mid, inputTriangles.end());
			}
		}

		// build trees
		l = new BVHNode();
		r = new BVHNode();
		l->build(leftTriangles, triangles);
		r->build(rightTriangles, triangles);
	}

	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		// test intersect with AABB
		float boxT;
		if (!bounds.rayAABB(ray, boxT) || boxT > intersection.t) {
			return;
		}

		// if leaf, test intersection with triangle
		if (isLeaf) {
			for (unsigned int i = 0; i < num; i++) {
				float t, u, v;
				if (triangles[offset + i].rayIntersect(ray, t, u, v)) {
					if (t > 0 && t < intersection.t) {  
						intersection.t = t;
						intersection.ID = offset + i;
						intersection.alpha = 1.0f - (u + v);
						intersection.beta = u;
						intersection.gamma = v;
					}
				}
			}
			return;
		}

		// traverse node
		float tLeft = FLT_MAX, tRight = FLT_MAX;
		bool hitLeft = l ? l->bounds.rayAABB(ray, tLeft) : false;
		bool hitRight = r ? r->bounds.rayAABB(ray, tRight) : false;

		if (tLeft < tRight) {
			if (hitLeft) l->traverse(ray, triangles, intersection);
			if (hitRight && tRight < intersection.t) r->traverse(ray, triangles, intersection);
		}
		else {
			if (hitRight) r->traverse(ray, triangles, intersection);
			if (hitLeft && tLeft < intersection.t) l->traverse(ray, triangles, intersection);
		}
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
		// check if intersect with current AABB
		float t;
		if (!bounds.rayAABB(ray, t))
		{
			return true;
		}

		// left node, test intersection to all triangles
		if (isLeaf)
		{
			for (unsigned int i = 0; i < num; i++)
			{
				float t, u, v;
				if (triangles[offset + i].rayIntersect(ray, t, u, v))
				{
					if (t < maxT)
					{
						return false;
					}
				}
			}
			return true;
		}


		float tLeft, tRight;
		bool hitLeft = l->bounds.rayAABB(ray, tLeft);
		bool hitRight = r->bounds.rayAABB(ray, tRight);
		if (tLeft < tRight) {
			if (hitLeft && !l->traverseVisible(ray, triangles, maxT)) return false;
			if (hitRight && !r->traverseVisible(ray, triangles, maxT)) return false;
		}
		else {
			if (hitRight && !r->traverseVisible(ray, triangles, maxT)) return false;
			if (hitLeft && !l->traverseVisible(ray, triangles, maxT)) return false;
		}

		return true;
	}
};

