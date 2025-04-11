#version 330 

layout(location=0) in vec3 vertexPosition;
layout(location=1) in vec2 vertexTexCoord; 
layout(location=2) in vec3 vertexNormal;
layout(location=3) in vec4 vertexColor;

out vec3 normal;
out vec3 color;
out vec2 uv;

uniform mat4 mvp;
uniform mat4 matModel;
uniform mat4 matView;

void main() 
{
	vec4 pos = vec4(vertexPosition, 1);

	uv = vertexTexCoord;
	normal = (matModel * vec4(vertexNormal, 0)).xyz;
    color = vertexColor.rgb;

	gl_Position = mvp * pos;
}   
