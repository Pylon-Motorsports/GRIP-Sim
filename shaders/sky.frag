#version 450

layout(location = 0) in vec2 uv;
layout(location = 0) out vec4 outColor;

void main()
{
    // Vertical gradient: blue sky at top, pale horizon at bottom
    // In Vulkan, uv.y = 0 is top of screen, 1 is bottom
    vec3 sky     = vec3(0.35, 0.55, 0.85);
    vec3 horizon = vec3(0.72, 0.80, 0.88);
    outColor = vec4(mix(sky, horizon, uv.y), 1.0);
}
