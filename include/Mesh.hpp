#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <string>
#include <vector>

#include "Shader.hpp"

using namespace std;

#define MAX_BONE_INFLUENCE 4

// vertex attributes
struct Vertex {

  // position
  glm::vec3 position;

  // normal
  glm::vec3 normal;

  // texCoords
  glm::vec2 texture_coordinates;

  // tangent
  glm::vec3 tangent;

  // bitangent
  glm::vec3 bitangent;

  // bones used in skeletal animations -- joints
  // bone indexes which will influence this vertex
  int m_bone_ids[MAX_BONE_INFLUENCE];

  // weights from each bone
  float m_weights[MAX_BONE_INFLUENCE];
};

// texture image details
struct Texture {
  unsigned int id;
  string type;
  string path;
};

class Mesh {
public:
  // mesh Data
  vector<Vertex> vertices;
  vector<unsigned int> indices;
  vector<Texture> textures;

  unsigned int VAO, VBO, EBO;

  // constructor
  Mesh(vector<Vertex> vertices, vector<unsigned int> indices = {},
       vector<Texture> textures = {}) {
    this->vertices = vertices;
    this->indices = indices;
    this->textures = textures;

    // now that we have all the required data, set the vertex buffers and its
    // attribute pointers.
    setup_mesh();
  }

  // render mesh data
  void draw(Shader &shader) {
    // bind appropriate textures
    unsigned int diffuseNr = 1;
    unsigned int specularNr = 1;
    unsigned int normalNr = 1;
    unsigned int heightNr = 1;

    for (unsigned int i = 0; i < textures.size(); i++) {
      glActiveTexture(GL_TEXTURE0 +
                      i); // active proper texture unit before binding
      // retrieve texture number (the N in diffuse_textureN)
      string number;
      string name = textures[i].type;
      if (name == "texture_diffuse")
        number = std::to_string(diffuseNr++);
      else if (name == "texture_specular")
        number =
            std::to_string(specularNr++); // transfer unsigned int to string
      else if (name == "texture_normal")
        number = std::to_string(normalNr++); // transfer unsigned int to string
      else if (name == "texture_height")
        number = std::to_string(heightNr++); // transfer unsigned int to string

      // now set the sampler to the correct texture unit
      glUniform1i(glGetUniformLocation(shader.id, (name + number).c_str()), i);
      // and finally bind the texture
      glBindTexture(GL_TEXTURE_2D, textures[i].id);
    }

    // draw mesh
    glBindVertexArray(VAO);
    if (!indices.empty()) {
      glDrawElements(GL_TRIANGLES, static_cast<unsigned int>(indices.size()),
                     GL_UNSIGNED_INT, 0);
    } else {
      glDrawArrays(GL_TRIANGLES, 0, static_cast<unsigned int>(vertices.size()));
    }

    // always good practice to set everything back to defaults once configured.
    glActiveTexture(GL_TEXTURE0);
    glBindVertexArray(0);
  }

private:
  // initializes all the buffer objects/arrays
  void setup_mesh() {
    // create buffers/arrays
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);

    glBindVertexArray(VAO);

    // load data into vertex buffers
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Vertex),
                 &vertices[0], GL_STATIC_DRAW);

    if (!indices.empty()) {
      glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
      glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                   indices.size() * sizeof(unsigned int), &indices[0],
                   GL_STATIC_DRAW);
    }

    // configure vertex attribute pointers

    // vertex Positions
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex), (void *)0);

    // vertex normals
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void *)offsetof(Vertex, normal));

    // vertex texture coords
    glEnableVertexAttribArray(2);
    glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void *)offsetof(Vertex, texture_coordinates));

    // vertex tangent
    glEnableVertexAttribArray(3);
    glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void *)offsetof(Vertex, tangent));

    // vertex bitangent
    glEnableVertexAttribArray(4);
    glVertexAttribPointer(4, 3, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void *)offsetof(Vertex, bitangent));

    // ids
    glEnableVertexAttribArray(5);
    glVertexAttribIPointer(5, 4, GL_INT, sizeof(Vertex),
                           (void *)offsetof(Vertex, m_bone_ids));

    // weights
    glEnableVertexAttribArray(6);
    glVertexAttribPointer(6, 4, GL_FLOAT, GL_FALSE, sizeof(Vertex),
                          (void *)offsetof(Vertex, m_weights));
    glBindVertexArray(0);
  }
};
