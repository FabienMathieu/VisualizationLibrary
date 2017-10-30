#version 330 core

uniform mat4 ViewMatrix;
uniform mat4 ModelMatrix;
uniform mat4 ProjectionMatrix;

in vec4 VertexPosition;

void main()
{
   gl_Position = ProjectionMatrix * ViewMatrix * ModelMatrix * VertexPosition;
}
