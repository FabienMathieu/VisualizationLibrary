#version 330

layout (location = 0) in vec3 vl_VertexPosition;
layout (location = 1) in float Type;
layout (location = 2) in vec3 Velocity;
layout (location = 3) in float Age;

out float Type0;
out vec3 Position0;
out vec3 Velocity0;
out float Age0;

void main()
{
  Type0 = Type;
  Position0 = vl_VertexPosition;
  Velocity0 = Velocity;
  Age0 = Age;
}
