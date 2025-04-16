#version 330 core

in vec3 v_color;
in vec3 v_normal;
in vec3 v_world_pos;

out vec4 out_color;

void main() {
    vec3 light_dir = normalize(vec3(2.0, 4.0, 3.0));
    vec3 norm = normalize(v_normal);
    
    float diff = max(dot(norm, light_dir), 0.0);
    vec3 lighting = v_color * diff + 0.1 * v_color; // simple diffuse + ambient

    out_color = vec4(lighting, 1.0);
}
