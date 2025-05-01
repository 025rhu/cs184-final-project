#version 330 core
#define MAX_BONES 100

layout(location = 0) in vec3  in_position;
layout(location = 1) in vec3  in_normal;
layout(location = 2) in vec3  in_color;
layout(location = 4) in ivec4 in_bone_ids;
layout(location = 5) in vec4  in_weights;

uniform mat4  uM, uV, uP;
uniform mat4  uBones[MAX_BONES];
uniform float uTime;

out vec3 v_color;
out vec3 v_normal;
out vec3 v_world_pos;

void main()
{
    /* skinning matrix */
    int i0 = clamp(in_bone_ids.x, 0, MAX_BONES-1);
    int i1 = clamp(in_bone_ids.y, 0, MAX_BONES-1);
    int i2 = clamp(in_bone_ids.z, 0, MAX_BONES-1);
    int i3 = clamp(in_bone_ids.w, 0, MAX_BONES-1);

    mat4 skin =
          in_weights.x * uBones[i0] +
          in_weights.y * uBones[i1] +
          in_weights.z * uBones[i2] +
          in_weights.w * uBones[i3];

    vec3 pos = (skin * vec4(in_position,1.0)).xyz;

    /* bobbing */
    pos.y += 0.5 * sin(uTime * 2.0);

    vec4 world_pos = uM * vec4(pos, 1.0);
    gl_Position    = uP * uV * world_pos;

    v_color      = in_color;
    v_normal     = mat3(skin) * in_normal;   // skinned normal
    v_world_pos  = world_pos.xyz;
}
