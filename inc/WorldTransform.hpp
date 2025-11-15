#pragma once
#ifndef  WORLDTRANSFORM_HPP
#define WORLDTRANSFORM_HPP

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

class WorldTransform {

private:

	glm::mat4 world = glm::mat4(1.0);;
	glm::mat4 translateMat4 = glm::mat4(1.0);
	glm::mat4 rotateMat4 = glm::mat4(1.0);
	glm::mat4 scaleMat4 = glm::mat4(1.0);


	glm::vec3 position = glm::vec3(0.0f);
	glm::vec3 scale = glm::vec3(0.0f);

public:

	WorldTransform() = default;


	void SetAbsPosition(glm::vec3 position) {
		translateMat4 = glm::translate(glm::mat4(1.0), position);
		this->position = position;
	}

	void SetRelPosition(glm::vec3 displacment) {
		translateMat4 = glm::translate(translateMat4, displacment);
		this->position = displacment;
	}

	/*
	// Absolute position in world space
	void SetPosition(glm::vec3 position) {
		translateMat4 = glm::translate(glm::mat4(1.0), position);
		this->position = position;
	}

	// Relative position; adding to current postion
	void Translate(glm::vec3 delta) {
		translateMat4 = glm::translate(translateMat4, delta);
		this->position += delta;
	}
	*/

	void SetScale(glm::vec3 scale) {
		scaleMat4 = glm::scale(scaleMat4, scale);
		this->scale = scale;
	}

	void SetScale(float scalingFactor) {
		scaleMat4 = glm::scale(scaleMat4, glm::vec3(scalingFactor, scalingFactor, scalingFactor));
		this->scale = glm::vec3(scalingFactor, scalingFactor, scalingFactor);
	}

	void SetAbsRotation(float angle, glm::vec3 rotationAxis) {
		rotateMat4 = glm::rotate(glm::mat4(1.0), angle, rotationAxis);
	}

	void SetAbsRotation(glm::mat4 orientation) {
		rotateMat4 = orientation;
	}

	void SetRelRotation(float angle, glm::vec3 rotationAxis){
		rotateMat4 = glm::rotate(rotateMat4, angle, rotationAxis);
	}

	void SetRelRotation(glm::mat4 deltaRotation) {
		rotateMat4 = deltaRotation * rotateMat4;
	}

	void SetRotationAbtPoint(const glm::mat4& rotation, const glm::vec3& point) {

		glm::mat4 translateToPoint = glm::translate(glm::mat4(1.0f), -point);
		glm::mat4 translateBack = glm::translate(glm::mat4(1.0f), point);

		glm::mat4 rotateAbtPoint = translateBack * rotation * translateToPoint;
		SetRelRotation(rotateAbtPoint);
	}

	glm::mat4 GetMatrix() {

		world = translateMat4 * rotateMat4 * scaleMat4;

		return world;
	}

	glm::mat4 GetMatrix() const {

		glm::mat4 model = translateMat4 * rotateMat4 * scaleMat4;

		return model;
	}


	glm::vec3 GetPosition() {

		return position;
	}

	glm::mat4 GetRotation() {
		
		return rotateMat4;
	}


};

#endif
