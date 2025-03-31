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

	bool rayIntersect1(const Ray& r, float& t, float& u, float& v) const
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
		// 求出重心坐标
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

		// 使用初始化列表计算最大和最小值
		float tenter = std::max({ txmin, tymin, tzmin });
		float texit = std::min({ txmax, tymax, tzmax });

		if (tenter <= texit && texit > 0) {
			t = tenter; // 更新最近的交点距离
			return true;
		}
		return false;
	}

	// Test if AABB collides with ray
	//bool rayAABB(const Ray& r)
	//{
	//	float txmin = (r.dir.x != 0) ? (min.x - r.o.x) / r.dir.x : (min.x >= r.o.x) ? FLT_MAX : -FLT_MAX;
	//	float txmax = (r.dir.x != 0) ? (max.x - r.o.x) / r.dir.x : (max.x >= r.o.x) ? FLT_MAX : -FLT_MAX;
	//	if (r.dir.x < 0) std::swap(txmin, txmax);

	//	// Y轴处理（新增零方向保护）
	//	float tymin = (r.dir.y != 0) ? (min.y - r.o.y) / r.dir.y : (min.y >= r.o.y) ? FLT_MAX : -FLT_MAX;
	//	float tymax = (r.dir.y != 0) ? (max.y - r.o.y) / r.dir.y : (max.y >= r.o.y) ? FLT_MAX : -FLT_MAX;
	//	if (r.dir.y < 0) std::swap(tymin, tymax);

	//	// Z轴处理（新增零方向保护）
	//	float tzmin = (r.dir.z != 0) ? (min.z - r.o.z) / r.dir.z : (min.z >= r.o.z) ? FLT_MAX : -FLT_MAX;
	//	float tzmax = (r.dir.z != 0) ? (max.z - r.o.z) / r.dir.z : (max.z >= r.o.z) ? FLT_MAX : -FLT_MAX;
	//	if (r.dir.z < 0) std::swap(tzmin, tzmax);

	//	// 计算进入和退出时间
	//	float tenter = std::max({ txmin, tymin, tzmin });
	//	float texit = std::min({ txmax, tymax, tzmax });

	//	return (tenter <= texit && texit > 0);
	//}
	bool rayAABB(const Ray& r) const {
		float tmin = 0.0f;
		float tmax = FLT_MAX;

		for (int i = 0; i < 3; ++i) {
			if (fabs(r.dir[i]) < 1e-6) { // 精确处理轴对齐情况
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
	// 修改后的AABB相交检测
	bool rayAABB(const Ray& r, float& t)
	{
		if (min.x > max.x || min.y > max.y || min.z > max.z) return false;

		// 处理方向分量为零时的情况
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
		return false;
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
	unsigned int offset;  // 三角形列表中的起始索引
	unsigned char num;    // 该节点包含的三角形数量
	bool isLeaf;         // 是否是叶子节点

	BVHNode()
	{
		r = NULL;
		l = NULL;
		offset = 0;
		num = 0;
		isLeaf = false;
	}

	~BVHNode()
	{
		if (r) delete r;
		if (l) delete l;
	}
#if 0
	// Note there are several options for how to implement the build method. Update this as required
	void build(std::vector<Triangle>& inputTriangles, std::vector<Triangle>& triangles)
	{
		// 如果是叶子节点
		if (inputTriangles.size() <= MAXNODE_TRIANGLES) {
			isLeaf = true;
			offset = (unsigned int)triangles.size();  // 设置三角形的起始索引
			num = (unsigned char)inputTriangles.size();

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

		// 计算整个节点的包围盒
		bounds.reset();
		for (const auto& triangle : inputTriangles) {
			bounds.extend(triangle.vertices[0].p);
			bounds.extend(triangle.vertices[1].p);
			bounds.extend(triangle.vertices[2].p);
		}
		// SAH
		const int BUCKETS = 12;
		const float C_trav = 1.0f;
		const float C_intersect = 2.0f;
		float minCost = FLT_MAX;
		int bestAxis = -1;
		float bestSplit = 0;
		

		// TODO USE SAH TO BOOST THE PERFORMANCE
		// 计算所有三角形的中心点
		std::vector<Vec3> centers;
		centers.reserve(inputTriangles.size());
		for (const auto& triangle : inputTriangles) {
			centers.push_back(triangle.centre());
		}

		// 选择最佳分割轴
		Vec3 extent = bounds.max - bounds.min;
		int axis = 0;
		if (extent.y > extent.x && extent.y > extent.z) axis = 1;
		else if (extent.z > extent.x && extent.z > extent.y) axis = 2;

		// 计算分割点（使用中位数）
		float split = 0.0f;
		std::vector<float> centerValues(centers.size());
		for (size_t i = 0; i < centers.size(); i++) {
			centerValues[i] = (axis == 0) ? centers[i].x : ((axis == 1) ? centers[i].y : centers[i].z);
		}
		size_t mid = centerValues.size() / 2;
		std::nth_element(centerValues.begin(), centerValues.begin() + mid, centerValues.end());
		split = centerValues[mid];

		// 分割三角形
		std::vector<Triangle> leftTriangles, rightTriangles;
		for (size_t i = 0; i < inputTriangles.size(); i++) {
			float centerValue = (axis == 0) ? centers[i].x : ((axis == 1) ? centers[i].y : centers[i].z);
			if (centerValue <= split) {
				leftTriangles.push_back(inputTriangles[i]);
			}
			else {
				rightTriangles.push_back(inputTriangles[i]);
			}
		}

		// 处理特殊情况：如果一边为空，则平均分配
		if (leftTriangles.empty() || rightTriangles.empty()) {
			size_t mid = inputTriangles.size() / 2;
			leftTriangles.assign(inputTriangles.begin(), inputTriangles.begin() + mid);
			rightTriangles.assign(inputTriangles.begin() + mid, inputTriangles.end());
		}

		// 递归构建左右子树
		l = new BVHNode();
		r = new BVHNode();
		l->build(leftTriangles, triangles);
		r->build(rightTriangles, triangles);
	}
#endif
	void build(std::vector<Triangle>& inputTriangles, std::vector<Triangle>& triangles)
	{
		// 如果是叶子节点
		if (inputTriangles.size() <= MAXNODE_TRIANGLES) {
			isLeaf = true;
			offset = (unsigned int)triangles.size();  // 设置三角形的起始索引
			num = (unsigned char)inputTriangles.size();

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

			// 为每个桶计算候选分割点
			for (int b = 1; b < BUCKETS; ++b) {
				size_t splitIndex = (b * sortedCenters.size()) / BUCKETS;
				if (splitIndex == 0 || splitIndex >= sortedCenters.size()) continue;

				// 分割点值
				float splitValue = sortedCenters[splitIndex].first;

				// 计算左右包围盒（初始化 leftBox 和 rightBox）
				AABB leftBox, rightBox;
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
				float cost = C_trav +
					(leftBox.area() * splitIndex + rightBox.area() * (sortedCenters.size() - splitIndex)) * C_intersect / bounds.area();

				if (cost < minCost) {
					minCost = cost;
					bestAxis = axis;
					bestSplit = splitValue;
				}
			}
		}

		// 按最佳轴和分割点划分三角形
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

		// 处理特殊情况：如果一边为空，则平均分配
		if (leftTriangles.empty() || rightTriangles.empty()) {
			size_t mid = inputTriangles.size() / 2;
			leftTriangles.assign(inputTriangles.begin(), inputTriangles.begin() + mid);
			rightTriangles.assign(inputTriangles.begin() + mid, inputTriangles.end());
		}

		// 递归构建左右子树前显式创建子节点
		l = new BVHNode();
		r = new BVHNode();
		l->build(leftTriangles, triangles);
		r->build(rightTriangles, triangles);
	}

	void traverse(const Ray& ray, const std::vector<Triangle>& triangles, IntersectionData& intersection)
	{
		// 首先检查射线是否与当前节点的包围盒相交
		float boxT;
		if (!bounds.rayAABB(ray, boxT) || boxT > intersection.t) {
			return;
		}

		// 如果是叶子节点，测试与所有三角形的相交
		if (isLeaf) {
			for (unsigned int i = 0; i < num; i++) {
				float t, u, v;
				if (triangles[offset + i].rayIntersect(ray, t, u, v)) {
					if (t > 0 && t < intersection.t) {  // 确保t为正且是最近的交点
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

		// 递归遍历子节点，先遍历更近的子节点
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
		// 首先检查射线是否与当前节点的包围盒相交
		float t;
		if (!bounds.rayAABB(ray, t))
		{
			return true;
		}

		// 如果是叶子节点,测试与所有三角形的相交
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

		// 如果是内部节点,递归遍历左右子树
		if (l && !l->traverseVisible(ray, triangles, maxT)) return false;
		if (r && !r->traverseVisible(ray, triangles, maxT)) return false;
		return true;
	}
};




