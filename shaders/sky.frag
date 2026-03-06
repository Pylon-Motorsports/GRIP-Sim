#version 450

layout(location = 0) in float ndcY;

layout(location = 0) out vec4 outColor;

void main()
{
    // ndcY: -1 = top of screen (zenith), +1 = bottom (horizon) in Vulkan NDC
    float t = ndcY * 0.5 + 0.5;   // remap to [0=zenith, 1=horizon]

    vec3 zenith  = vec3(0.05, 0.18, 0.55);   // deep rally blue
    vec3 horizon = vec3(0.55, 0.75, 0.95);   // hazy light blue

    outColor = vec4(mix(zenith, horizon, t), 1.0);
}
