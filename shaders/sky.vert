#version 450

// Full-screen triangle: no vertex buffer needed.
// Three vertices cover the entire clip space using the gl_VertexIndex trick.
layout(location = 0) out float ndcY;

void main()
{
    // Vertices: (-1,-1), (3,-1), (-1,3) — covers [-1,1]x[-1,1] clip space
    vec2 uv     = vec2((gl_VertexIndex << 1) & 2, gl_VertexIndex & 2);
    gl_Position = vec4(uv * 2.0 - 1.0, 0.0, 1.0);
    ndcY        = gl_Position.y;   // -1 = top (zenith), +1 = bottom (horizon) in Vulkan NDC
}
