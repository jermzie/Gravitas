#version 330 core
out vec4 FragColor;

void main()
{
    // Gradient color based on point position in the array (newer points are brighter)
    FragColor = vec4(1.0, 0.6, 0.0, 1.0); // Orange color for path
}