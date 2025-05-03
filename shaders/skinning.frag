#version 330 core

in vec3 v_color;
in vec3 v_normal;
in vec3 worldPosition;

out vec4 FragColor;

uniform vec3 uLightDir = normalize(vec3(1.0, 1.0, 1.0)); 

void main() {
    vec3 normal = normalize(v_normal);
    float diff = max(dot(normal, uLightDir), 0.0);
    // vec3 baseColor = v_color;
    vec3 baseColor = vec3(101.0/255.0, 67.0/255.0, 33.0/255.0);

    vec3 shadedColor = diff * baseColor;
    FragColor = vec4(shadedColor, 1.0);
}
