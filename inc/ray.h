#pragma once

#ifndef RAY_H
#define RAY_H

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

// r = o + d*t
class Ray {
public:
	glm::vec3 origin;
	glm::vec3 direction;
	float t;
	float invLengthSquared;


	Ray() = default;
	Ray(glm::vec3& origin, glm::vec3& direction) : origin(origin), direction(direction), invLengthSquared(1 / (glm::length2(distance))) {
	}
};


inline float getSquaredDistanceToRay(const glm::vec3& p, const Ray& r) {

	// vector between ray origin(o) to point p
	const glm::vec3 op = p - r.origin;

	// projection of vector op onto ray
	float proj = glm::dot(op, r.direction);

	return glm::length2(op) - (proj * proj * r.invLengthSquared);

}


#endif // !RAY_H

