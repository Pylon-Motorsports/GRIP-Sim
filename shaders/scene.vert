#version 450

layout(location = 0) in vec3 inPos;
layout(location = 1) in vec3 inNormal;
layout(location = 2) in vec4 inColor;

layout(push_constant) uniform PC {
    mat4 viewProj;
    mat4 model;
} pc;

layout(location = 0) out vec3 fragWorldPos;
layout(location = 1) out vec3 fragNormal;
layout(location = 2) out vec4 fragColor;

void main()
{
    vec4 wp     = pc.model * vec4(inPos, 1.0);
    gl_Position = pc.viewProj * wp;
    fragWorldPos = wp.xyz;
    fragNormal   = normalize(mat3(pc.model) * inNormal);
    fragColor    = inColor;
}
