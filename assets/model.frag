#version 330

in vec3 normal;
in vec3 color;
in vec2 uv;

out vec4 finalColor;

uniform vec4 colDiffuse;


void main()
{
    finalColor = vec4(normal, 1);
}  
