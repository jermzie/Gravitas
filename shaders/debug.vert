#version 330
layout (location = 0) in vec3 position;
layout (location = 1) in vec4 color;

out vec4 frag_color;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

void main()
{
    frag_color = color;
    gl_Position = projection * view * model * vec4(position, 1.0);
}

