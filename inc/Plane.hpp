#pragma once

#ifndef PLANE_HPP
#define PLANE_HPP

#include <glm/glm.hpp>

class Plane {
public:
	glm::vec3 normal;
	glm::vec3 point;
	float distance;	// signed distance from origin
	float normLengthSq;

	bool isPointAbovePlane(const glm::vec3 p) const {
		float d = glm::dot(normal, p) + distance;

		return d >= 0;
	}

	Plane() = default;
	Plane(const glm::vec3& n, const glm::vec3& p) : normal(n), distance(glm::dot(-n, p)), normLengthSq(normal.x* normal.x + normal.y * normal.y + normal.z * normal.z) {}
};

inline float getSignedDistanceToPlane(const glm::vec3& v, const Plane& p) {
	return glm::dot(p.normal, v) + p.distance;
}

inline glm::vec3 getTriangleNormal(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c) {

	/*
	Manually compute cross product without constructing temp vectors and function overhead -- calling glm::cross()
	vec3 u = (a - c).
	vec3 v = (b - c).
	u x v = (u.y*v.z - u.z*v.x, u.z*v.x - u.x*v.z, u.x*v.y - u.y*v.x).
	*/

	float ux = a.x - c.x;
	float uy = a.y - c.y;
	float uz = a.z - c.z;

	float vx = b.x - c.x;
	float vy = b.y - c.y;
	float vz = b.z - c.z;

	float px = uy * vz - uz * vx;
	float py = uz * vx - ux * vz;
	float pz = ux * vy - uy * vx;

	return glm::vec3(px, py, pz);	// unnormalized
	
}

#endif // !PLANE_HPP
