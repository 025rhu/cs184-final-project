#version 330 core
#define MAX_BONES 100
layout(location=0) in vec3 in_pos;
layout(location=1) in vec3 in_norm;
layout(location=4) in ivec4 in_ids;
layout(location=5) in vec4  in_w;
uniform mat4 uM,uV,uP;
uniform mat4 uBones[MAX_BONES];
void main(){
    int i0=clamp(in_ids.x,0,MAX_BONES-1);
    int i1=clamp(in_ids.y,0,MAX_BONES-1);
    int i2=clamp(in_ids.z,0,MAX_BONES-1);
    int i3=clamp(in_ids.w,0,MAX_BONES-1);
    mat4 skin = in_w.x*uBones[i0]+in_w.y*uBones[i1]+in_w.z*uBones[i2]+in_w.w*uBones[i3];
    gl_Position = uP*uV*uM*skin*vec4(in_pos,1.0);
}
