#pragma once

#ifndef MOUSEPICKING_HPP
#define MOUSEPICKING_HPP


#include <glm/glm.hpp>

#include "camera.h"
#include "ray.h"


class MousePicking {

private:

	glm::vec3 hitPoint = glm::vec3(0.0f, 0.0f, 0.0f);

public:

	MousePicking() {}


	void SetIntersect(glm::vec3 intersection) {
		this->hitPoint = intersection;
	}

	// use raycasting to project mouse pos. as 3D ray for object picking
	Ray ScreenToWorldRay(
		float xpos, float ypos,
		int SCR_WIDTH, int SCR_HEIGHT,
		glm::mat4 projection, glm::mat4 view) {



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

	glm::vec3 DragObject(const Camera& camera, const Ray& r) {

		// compute distance between selected object and camera
		glm::vec3 objectDist = hitPoint - camera.Position;


		float planeOffset = glm::dot(objectDist, camera.Front);						// project objectDist onto camera.Front unit vector
		glm::vec3 planeNormal = camera.Front;										// plane is perpendicular to camera.Front
		glm::vec3 planePoint = camera.Position + (camera.Front * planeOffset);		// some point on drag plane


		// ray-plane intersection
		// t = dot(N, P_0 - O) / dot(N, D)
		float denom = glm::dot(planeNormal, r.direction);
		if (std::abs(denom) > 1e-6) {	// avoid zero division

			float num = glm::dot(planeNormal, planePoint - r.origin);
			float t = num / denom;

			if (t >= 0.0f) {

				glm::vec3 rayIntersect = r.origin + r.direction * t;				   // compute ray @ t  when it intersects drag plane
				//glm::vec3 dragOffset =  rayIntersect - hitPoint;					   // translate object -- rigid bodies don't deform so if we translate hitPoint by some vector, centreOfMass translates exactly the same

				return rayIntersect;
			}

		}


		return hitPoint;		// default -- return previous hitpoint

	}





};


#endif