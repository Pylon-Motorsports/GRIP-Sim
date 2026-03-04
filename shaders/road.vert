#version 450

// Vertex inputs — must match RoadVertex layout (36 bytes)
layout(location = 0) in vec3  inPosition;
layout(location = 1) in vec3  inNormal;
layout(location = 2) in vec2  inUV;
layout(location = 3) in float inSurfaceId;

// Outputs to fragment shader
layout(location = 0) out vec3  fragNormal;
layout(location = 1) out vec2  fragUV;
layout(location = 2) out float fragSurfaceId;
layout(location = 3) out vec3  fragWorldPos;

// Push constants: view-projection + model matrix
layout(push_constant) uniform PushConstants {
    mat4 viewProj;
    mat4 model;
} pc;

void main()
{
    vec4 worldPos = pc.model * vec4(inPosition, 1.0);
    gl_Position   = pc.viewProj * worldPos;

    fragWorldPos  = worldPos.xyz;
    fragNormal    = mat3(transpose(inverse(pc.model))) * inNormal;
    fragUV        = inUV;
    fragSurfaceId = inSurfaceId;
}
