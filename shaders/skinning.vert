#version 330 core

layout(location = 0) in vec3 in_position;
layout(location = 1) in vec3 in_normal;
layout(location = 2) in vec3 in_color;
layout(location = 3) in uvec4 in_boneIndices;
layout(location = 4) in vec4 in_weights;

uniform mat4 uModel;
uniform mat4 uView;
uniform mat4 uProjection;

uniform mat4 uBoneMatrices[100];  // or however many bones you use

out vec3 v_color;

void main() {
    mat4 skinMatrix = 
        uBoneMatrices[in_boneIndices[0]] * in_weights[0] +
        uBoneMatrices[in_boneIndices[1]] * in_weights[1] +
        uBoneMatrices[in_boneIndices[2]] * in_weights[2] +
        uBoneMatrices[in_boneIndices[3]] * in_weights[3];


    //mat4 skinMatrix = mat4(1.0);  // TEMP: Skip skinning

    //mat4 skinMatrix = uBoneMatrices[0] * 10.0;

    //skinMatrix[0][0] *= 100.0;
    //skinMatrix[1][1] *= 100.0;
    //skinMatrix[2][2] *= 100.0;



    vec4 worldPosition = uModel * skinMatrix * vec4(in_position, 1.0);
    gl_Position = uProjection * uView * worldPosition;

    v_color = in_color;
    //v_color = vec3(1.0, 0.0, 0.0); // Bright red

}
