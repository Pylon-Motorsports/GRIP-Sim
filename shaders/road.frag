#version 450

layout(location = 0) in vec3  fragNormal;
layout(location = 1) in vec2  fragUV;
layout(location = 2) in float fragSurfaceId;
layout(location = 3) in vec3  fragWorldPos;

layout(location = 0) out vec4 outColor;

// Surface colour lookup table — indices match RoadBuilder::surfaceId() mapping
// 0=Tarmac  1=Gravel  2=Grass  3=Snow  4=Ice  5=Rock  6=Sand
const vec3 SURFACE_COLORS[7] = vec3[](
    vec3(0.15, 0.15, 0.15),   // 0 Tarmac      – dark grey
    vec3(0.62, 0.56, 0.40),   // 1 Gravel       – tan
    vec3(0.32, 0.42, 0.20),   // 2 Grass        – green-brown
    vec3(0.88, 0.94, 0.98),   // 3 Snow         – blue-white
    vec3(0.70, 0.88, 0.95),   // 4 Ice          – light blue
    vec3(0.48, 0.40, 0.30),   // 5 Rock         – grey-brown
    vec3(0.85, 0.78, 0.55)    // 6 Sand         – sandy yellow
);

void main()
{
    // Simple directional light (fixed sun direction, world space)
    vec3 sunDir  = normalize(vec3(0.5, 1.0, 0.3));
    vec3 norm    = normalize(fragNormal);

    float diffuse = max(dot(norm, sunDir), 0.0);
    float ambient = 0.35;
    float light   = ambient + diffuse * 0.65;

    // Subtle V-tint: slightly darker toward road edges
    float edgeDarken = 1.0 - 0.15 * abs(fragUV.y - 0.5) * 2.0;

    int   id    = clamp(int(fragSurfaceId), 0, 6);
    vec3  color = SURFACE_COLORS[id] * light * edgeDarken;

    outColor = vec4(color, 1.0);
}
