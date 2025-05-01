#version 330 core
#define MAX_BONES 100   

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec3 in_color;
layout(location = 4) in ivec4 in_bone_ids;   // 4 ints
layout(location = 5) in vec4  in_weights;  

uniform mat4 uM; // Model
uniform mat4 uV; // View
uniform mat4 uP; // Projection
uniform mat4  uBones[MAX_BONES];  
uniform float uTime;

out vec3 v_color;
out vec3 v_normal;
out vec3 v_world_pos;

void main()
{
    /* skinning indices (clamped) */
    int i0 = clamp(in_bone_ids.x, 0, MAX_BONES-1);
    int i1 = clamp(in_bone_ids.y, 0, MAX_BONES-1);
    int i2 = clamp(in_bone_ids.z, 0, MAX_BONES-1);
    int i3 = clamp(in_bone_ids.w, 0, MAX_BONES-1);

    /* weighted skin matrix */
    mat4 skin =
          in_weights.x * uBones[i0] +
          in_weights.y * uBones[i1] +
          in_weights.z * uBones[i2] +
          in_weights.w * uBones[i3];

    /* skinned position in object space */
    vec4 skinned_pos = skin * vec4(in_position, 1.0);

    /* sinusoidal test offset (½-unit bobbing) */
    skinned_pos.y += 0.5 * sin(uTime);

    /* standard transforms */
    v_color = in_color;
    vec4 world_pos = uM * skinned_pos;
    gl_Position    = uP * uV * world_pos;

    /* pass varyings if you use them */
    v_normal     = mat3(transpose(inverse(uM))) * in_normal;
    v_world_pos  = world_pos.xyz;
}
