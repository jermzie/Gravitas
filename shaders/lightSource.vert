#version 330 core
layout (location = 0) in vec3 aPos;


// transformation matrices
uniform mat4 gWVP;

void main()
{
    // transformed vector
    gl_Position = gWVP * vec4(aPos, 1.0f);

}