#version 450

layout(location = 0) in vec3 fragWorldPos;
layout(location = 1) in vec3 fragNormal;
layout(location = 2) in vec4 fragColor;

layout(location = 0) out vec4 outColor;

void main()
{
    vec3 sun  = normalize(vec3(0.3, 1.0, 0.5));
    vec3 n    = normalize(fragNormal);
    float diff = max(dot(n, sun), 0.0);
    float light = 0.35 + 0.65 * diff;

    vec3 col;
    if (fragColor.a < 0.5) {
        // Ground checkerboard (2m squares)
        float cx = floor(fragWorldPos.x * 0.5);
        float cz = floor(fragWorldPos.z * 0.5);
        float check = mod(cx + cz, 2.0);
        col = mix(vec3(0.32, 0.38, 0.26), vec3(0.45, 0.50, 0.35), check);
    } else {
        col = fragColor.rgb;
    }

    outColor = vec4(col * light, 1.0);
}
