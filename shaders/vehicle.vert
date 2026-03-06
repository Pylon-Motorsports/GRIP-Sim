#version 450

layout(location = 0) in vec3 inPos;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec4 inColor;

layout(push_constant) uniform PC {
    mat4 viewProj;
    mat4 model;
} pc;

layout(location = 0) out vec4 outColor;
layout(location = 1) out vec3 outNormal;

void main()
{
    gl_Position = pc.viewProj * pc.model * vec4(inPos, 1.0);
    // Normal matrix: assumes uniform scale or no scale (debug geometry).
    outNormal = normalize(mat3(pc.model) * inNormal);
    outColor  = inColor;
}
