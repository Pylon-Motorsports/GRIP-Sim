#version 450

layout(location = 0) in  vec4 inColor;
layout(location = 1) in  vec3 inNormal;

layout(location = 0) out vec4 outColor;

void main()
{
    vec3  L   = normalize(vec3(0.4, 1.0, 0.3));
    float d   = clamp(dot(inNormal, L), 0.0, 1.0);
    float lit = 0.15 + 0.85 * d;
    outColor  = vec4(inColor.rgb * lit, inColor.a);
}
