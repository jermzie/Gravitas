#pragma once
#ifndef DEBUGGER_HPP
#define DEBUGGER_HPP

#include <glm/glm.hpp>

#include "Model.hpp"
#include "WorldTransform.hpp"
#include "Plane.hpp"

class Debugger {

	Model arrowHead;
	
	Debugger() {

		arrowHead.LoadModel("pyramid.obj");
	}

	// Help Visualize Bugs??
	void drawAxes() {

		glm::vec3 origin(0.0f);
		glm::vec3 xAxis(1.0f, 0.0f, 0.0f);
		glm::vec3 yAxis(0.0f, 1.0f, 0.0f);
		glm::vec3 zAxis(0.0f, 0.0f, 1.0f);




	}

	// Draw Bounding Volume WireFrame
	void drawCollider(const Model& colliderModel, const WorldTransform& colliderTrans, const Shader& shader, glm::mat4 projection, glm::mat4 view) {

		// probably make a imgui toggle for this
	   // draw wireframe collision mesh
		glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
		shader.use();
		shader.setMat4("gWVP", projection * view * colliderTrans.GetMatrix());
		colliderModel.Draw(shader);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
	}

	// Draw Vector (Rays?)
	void drawVector(const glm::vec3 vertex, glm::vec3 color) {

		Shader vectorShader();
	}

	// Draw Normals 
	void drawNormals() {

	}

	void drawPlane(const Plane& p) {

	}
};

#endif