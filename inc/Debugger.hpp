#pragma once
#ifndef DEBUGGER_HPP
#define DEBUGGER_HPP

#include <glm/glm.hpp>

#include "Model.hpp"
#include "WorldTransform.hpp"
#include "Plane.hpp"

class Debugger {
	

	// Help Visualize Bugs??
	void drawAxes() {

	}

	// Draw Bounding Volume WireFrame
	void drawCollider(const Model& colliderModel, WorldTransform& colliderTrans, bool wireframe) {

	}

	// Draw Vector (Rays?)
	void drawVector(const glm::vec3 v) {

	}

	// Draw Normals 
	void drawNormals() {

	}

	void drawPlane(const Plane& p) {

	}
};

#endif