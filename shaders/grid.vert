#version 330

out vec3 WorldPos;

uniform mat4 gVP = mat4(1.0);							// view projection matrix
uniform float gGridSize = 100.0;
uniform vec3 gCameraWorldPos = vec3(0.0, 0.0, 0.0);		// camera world position

const vec3 Pos[4] = vec3[4](
	vec3(-1.0, 0.0, -1.0),		// botLeft
	vec3( 1.0, 0.0, -1.0),		// botRight
	vec3( 1.0, 0.0,  1.0),		// topRight
	vec3(-1.0, 0.0,  1.0)		// botLeft

);

const int Indices[6] = int[6](0, 2, 1, 2, 0, 3);

void main(){
	int i = Indices[gl_VertexID];

	vec3 vPos3 = Pos[i] * gGridSize;		

	vPos3.x += gCameraWorldPos.x;	// lock xz-grid to camera
	vPos3.z += gCameraWorldPos.z;

	vec4 vPos4 = vec4(vPos3, 1.0);

	gl_Position = gVP * vPos4;

	WorldPos = vPos3;
}