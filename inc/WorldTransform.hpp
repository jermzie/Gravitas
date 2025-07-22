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

	WorldTransform() {}


	void SetPosition(glm::vec3 position) {
		translateMat4 = glm::translate(translateMat4, position);
		this->position = position;
	}

	void SetScale(glm::vec3 scale) {
		scaleMat4 = glm::scale(scaleMat4, scale);
		this->scale = scale;
	}

	void SetScale(float scalingFactor) {
		scaleMat4 = glm::scale(scaleMat4, glm::vec3(scalingFactor, scalingFactor, scalingFactor));
		this->scale = glm::vec3(scalingFactor, scalingFactor, scalingFactor);
	}


	void SetRotate(float angle, glm::vec3 rotationAxis){
		rotateMat4 = glm::rotate(rotateMat4, angle, rotationAxis);
	}

	void SetRotate(glm::mat4 orientation) {
		rotateMat4 = orientation;
	}

	glm::mat4 GetMatrix() {

		world = translateMat4 * rotateMat4 * scaleMat4;

		return world;
	}

	glm::vec3 GetPosition() {

		return position;
	}


};

#endif // ! WORLDTRANSFORM_HPP