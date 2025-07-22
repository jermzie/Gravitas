#pragma once

/*
#ifndef PROJECT_SOURCE_DIR
#error "PROJECT_SOURCE_DIR is not defined"
#endif

#include <glad/glad.h>

#include "shader.h"

const std::string shaderDir = std::string(PROJECT_SOURCE_DIR) + "/shaders/";

class Grid {
private:
	Shader grid(shaderDir + "grid.vert", shaderDir + "grid.frag");

public:
	void setup(){
	}

	void darkMode(bool darkMode = true) {

		grid.use();


		grid.setVec4("gGridColorThin", vec4(1.0, 1.0, 1.0, 1.0));
		grid.setVec4("gGridColorThick", vec4(1.0, 1.0, 1.0, 1.0));


	}

	void renderGrid() {

	}

};
*/