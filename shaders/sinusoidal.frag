#version 330 core
in  vec3 v_color;
in  vec3 v_normal;
in  vec3 v_world_pos;

out vec4 out_color;

void main()
{
    vec3 lightDir = normalize(vec3(2.0, 4.0, 3.0));
    vec3 N        = normalize(v_normal);

    float diff = max(dot(N, lightDir), 0.0);
    vec3  rgb  = v_color * diff + 0.1 * v_color;

    out_color = vec4(rgb, 1.0);
}
