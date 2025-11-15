#ifndef MOUSEPICKING_HPP
#define MOUSEPICKING_HPP

#include <glm/glm.hpp>

#include <iostream>

#include "../physics/RigidBody.hpp"
#include "WorldTransform.hpp"
#include "Camera.hpp"
#include "Ray.hpp"

class MousePicking
{

public:
	MousePicking() = default;

	// use raycasting to project mouse pos. as 3D ray for object picking
	Ray ScreenToWorldRay(
		float xpos, float ypos,
		int SCR_WIDTH, int SCR_HEIGHT,
		glm::mat4 projection, glm::mat4 view)
	{

		// Screen Space --> Clip Space (NDC)
		// scale screen coordinates to NDC [-1, 1]
		float x = (2.0f * (float)xpos) / (float)SCR_WIDTH - 1.0f;
		float y = 1.0f - (2.0f * (float)ypos) / (float)SCR_HEIGHT; // flip the y-axis (starts from bot-left in opengl)

		glm::vec4 rayStart_clip(x, y, -1.0f, 1.0f);
		glm::vec4 rayEnd_clip(x, y, 1.0, 1.0f);

		/*
		// Clip Space --> Camera Space
		// projection matrix transforms ...
		glm::mat4 inverseProjectionMatrix = glm::inverse(projection);

		glm::vec4 rayStart_camera = inverseProjectionMatrix * rayStart_clip;
		glm::vec4 rayEnd_camera = inverseProjectionMatrix * rayEnd_clip;


		rayStart_camera /= rayStart_camera.w;
		rayEnd_camera /= rayEnd_camera.w;

		// Camera Space --> World Space
		// view matrix transforms ...
		glm::mat4 inverseViewMatrix = glm::inverse(view);

		glm::vec4 rayStart_world = inverseProjectionMatrix * rayStart_camera;
		glm::vec4 rayEnd_world = inverseProjectionMatrix * rayEnd_camera;

		rayStart_world /= rayStart_world.w;
		rayEnd_world /= rayEnd_world.w;
		*/

		// faster method -- just one inverse operation
		glm::mat4 inverseVPMatrix = glm::inverse(projection * view);
		glm::vec4 rayStart_world = inverseVPMatrix * rayStart_clip;
		glm::vec4 rayEnd_world = inverseVPMatrix * rayEnd_clip;

		// divide by homogeneous component
		rayStart_world /= rayStart_world.w;
		rayEnd_world /= rayEnd_world.w;

		glm::vec3 rayDir_world(rayEnd_world - rayStart_world);

		Ray r;
		r.origin = glm::vec3(rayStart_world);
		r.direction = glm::normalize(rayDir_world);

		return r;
	}

	// Drag body along plane perpendicular to camera
	bool RayPlaneIntersection(const Ray &ray, const glm::vec3 &planeNormal, const glm::vec3 &planePoint, float &t)
	{

		// Calculate ray-plane intersection
		float denom = glm::dot(planeNormal, ray.direction);

		// Check if ray is not parallel to plane -- avoid zero division
		if (std::abs(denom) < 1e-6)
		{
			return false;
		}

		t = glm::dot(planeNormal, planePoint - ray.origin) / denom;

		// return t >= 0;

		// min max intersect distances
		return (t >= 0.1f && t < 50.0f);
	}
};

#endif
