#version 330 core

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec3 in_color;

uniform mat4 uM; // Model
uniform mat4 uV; // View
uniform mat4 uP; // Projection

out vec3 v_color;
out vec3 v_normal;
out vec3 v_world_pos;

void main() {
    vec4 world_pos = uM * vec4(in_position, 1.0);
    v_color = in_color;
    v_normal = mat3(transpose(inverse(uM))) * in_normal;
    v_world_pos = world_pos.xyz;

    gl_Position = uP * uV * world_pos;
}
