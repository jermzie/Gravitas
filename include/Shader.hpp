#pragma once

#ifndef PROJECT_SOURCE_DIR
#error "PROJECT_SOURCE_DIR is not defined"
#endif

#include <glad/glad.h>
#include <glm/glm.hpp>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

class Shader {
public:
  unsigned int id;

  // constructor generates the shader on the fly
  // ------------------------------------------------------------------------
  Shader(const char *vertex_name, const char *fragment_name) {
    init(vertex_name, fragment_name);
  }

  // default constructor
  Shader() {};

  void init(const char *vertex_name, const char *fragment_name) {

    // 1. retrieve the vertex/fragment source code from filePath
    std::string vertex_string;
    std::string fragment_string;
    std::ifstream v_shader_file;
    std::ifstream f_shader_file;

    std::string vertex_path = std::string(PROJECT_SOURCE_DIR) + "/shaders/" +
                              std::string(vertex_name);
    std::string fragment_path = std::string(PROJECT_SOURCE_DIR) + "/shaders/" +
                                std::string(fragment_name);
    // ensure ifstream objects can throw exceptions:
    v_shader_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    f_shader_file.exceptions(std::ifstream::failbit | std::ifstream::badbit);
    try {
      // open files
      v_shader_file.open(vertex_path);
      f_shader_file.open(fragment_path);
      std::stringstream v_shader_stream, f_shader_stream;

      // read file's buffer contents into streams
      v_shader_stream << v_shader_file.rdbuf();
      f_shader_stream << f_shader_file.rdbuf();

      // close file handlers
      v_shader_file.close();
      f_shader_file.close();

      // convert stream into string
      vertex_string = v_shader_stream.str();
      fragment_string = f_shader_stream.str();

    } catch (std::ifstream::failure &e) {
      std::cout << "ERROR::SHADER::FILE_NOT_SUCCESSFULLY_READ: " << e.what()
                << std::endl;
    }
    const char *v_shader_code = vertex_string.c_str();
    const char *f_shader_code = fragment_string.c_str();
    // 2. compile shaders
    unsigned int vertex, fragment;
    // vertex shader
    vertex = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex, 1, &v_shader_code, NULL);
    glCompileShader(vertex);
    check_compile_errors(vertex, "VERTEX");
    // fragment Shader
    fragment = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment, 1, &f_shader_code, NULL);
    glCompileShader(fragment);
    check_compile_errors(fragment, "FRAGMENT");
    // shader Program
    id = glCreateProgram();
    glAttachShader(id, vertex);
    glAttachShader(id, fragment);
    glLinkProgram(id);
    check_compile_errors(id, "PROGRAM");
    // delete the shaders as they're linked into our program now and no longer
    // necessary
    glDeleteShader(vertex);
    glDeleteShader(fragment);
  }
  // activate the shader
  // ------------------------------------------------------------------------
  void use() const { glUseProgram(id); }
  // utility uniform functions
  // ------------------------------------------------------------------------
  void set_bool(const std::string &name, bool value) const {
    glUniform1i(glGetUniformLocation(id, name.c_str()), (int)value);
  }
  // ------------------------------------------------------------------------
  void set_int(const std::string &name, int value) const {
    glUniform1i(glGetUniformLocation(id, name.c_str()), value);
  }

  void set_uint(const std::string &name, unsigned int value) const {
    glUniform1ui(glGetUniformLocation(id, name.c_str()), value);
  }

  // ------------------------------------------------------------------------
  void set_float(const std::string &name, float value) const {
    glUniform1f(glGetUniformLocation(id, name.c_str()), value);
  }
  // ------------------------------------------------------------------------
  void set_vec2(const std::string &name, const glm::vec2 &value) const {
    glUniform2fv(glGetUniformLocation(id, name.c_str()), 1, &value[0]);
  }
  void set_vec2(const std::string &name, float x, float y) const {
    glUniform2f(glGetUniformLocation(id, name.c_str()), x, y);
  }
  // ------------------------------------------------------------------------
  void set_vec3(const std::string &name, const glm::vec3 &value) const {
    glUniform3fv(glGetUniformLocation(id, name.c_str()), 1, &value[0]);
  }
  void set_vec3(const std::string &name, float x, float y, float z) const {
    glUniform3f(glGetUniformLocation(id, name.c_str()), x, y, z);
  }
  // ------------------------------------------------------------------------
  void set_vec4(const std::string &name, const glm::vec4 &value) const {
    glUniform4fv(glGetUniformLocation(id, name.c_str()), 1, &value[0]);
  }
  void set_vec4(const std::string &name, float x, float y, float z,
                float w) const {
    glUniform4f(glGetUniformLocation(id, name.c_str()), x, y, z, w);
  }
  // ------------------------------------------------------------------------
  void set_mat2(const std::string &name, const glm::mat2 &mat) const {
    glUniformMatrix2fv(glGetUniformLocation(id, name.c_str()), 1, GL_FALSE,
                       &mat[0][0]);
  }
  // ------------------------------------------------------------------------
  void set_mat3(const std::string &name, const glm::mat3 &mat) const {
    glUniformMatrix3fv(glGetUniformLocation(id, name.c_str()), 1, GL_FALSE,
                       &mat[0][0]);
  }
  // ------------------------------------------------------------------------
  void set_mat4(const std::string &name, const glm::mat4 &mat) const {
    glUniformMatrix4fv(glGetUniformLocation(id, name.c_str()), 1, GL_FALSE,
                       &mat[0][0]);
  }

private:
  // utility function for checking shader compilation/linking errors.
  // ------------------------------------------------------------------------
  void check_compile_errors(GLuint shader, std::string type) {
    GLint success;
    GLchar infoLog[1024];
    if (type != "PROGRAM") {
      glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
      if (!success) {
        glGetShaderInfoLog(shader, 1024, NULL, infoLog);
        std::cout
            << "ERROR::SHADER_COMPILATION_ERROR of type: " << type << "\n"
            << infoLog
            << "\n -- --------------------------------------------------- -- "
            << std::endl;
      }
    } else {
      glGetProgramiv(shader, GL_LINK_STATUS, &success);
      if (!success) {
        glGetProgramInfoLog(shader, 1024, NULL, infoLog);
        std::cout
            << "ERROR::PROGRAM_LINKING_ERROR of type: " << type << "\n"
            << infoLog
            << "\n -- --------------------------------------------------- -- "
            << std::endl;
      }
    }
  }
};
