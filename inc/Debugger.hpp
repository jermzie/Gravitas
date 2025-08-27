#ifndef DEBUGGER_HPP
#define DEBUGGER_HPP

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <iostream>
#include <algorithm>
#include <vector>

#include "../inc/Model.hpp"
#include "../inc/Mesh.hpp"
#include "../inc/Shader.hpp"
#include "../inc/Gui.hpp"
#include "../inc/Plane.hpp"
#include "../physics/RigidBody.hpp"

class Debugger {

private:

	unsigned int VAO, VBO;

public:

	Debugger() {

		// create buffers

	}


	void drawAxes() {

		std::vector<float> axes{
			// x	 y     z	   r     g     b
			1.0f, 0.0f, 0.0f,	1.0f, 0.0f, 0.0f,
			0.0f, 0.0f, 0.0f,	1.0f, 0.0f, 0.0f,
			0.0f, 1.0f, 0.0f,   0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 0.0f,   0.0f, 1.0f, 0.0f,
			0.0f, 0.0f, 1.0f,   0.0f, 0.0f, 1.0f,
			0.0f, 0.0f, 0.0f,   0.0f, 0.0f, 1.0f,
		};

		// load data into vertex buffers
		glBindBuffer(GL_ARRAY_BUFFER, VBO);
		glBufferData(GL_ARRAY_BUFFER, axes.size() * sizeof(float), &axes[0], GL_STATIC_DRAW);


		// configure vertex attribute pointers

		// vertex Positions
		glEnableVertexAttribArray(0);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)0);

		// vertex colors
		glEnableVertexAttribArray(1);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));

		Shader debug("debug.vert", "debug.frag");

		debug.use();

		glLineWidth(5);

		// draw mesh
		glBindVertexArray(VAO);
		glDrawArrays(GL_LINES, 0, static_cast<unsigned int>(vertices.size()));
		glBindVertexArray(0);
		


	}

	void drawPoint() {

	}

	void drawVector() {

	}

	void drawPlane() {

	}

	void drawAABB() {

	}

	void drawConvexHull() {

	}
};

#endif
