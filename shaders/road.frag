#version 450

layout(location = 0) in vec3  fragNormal;
layout(location = 1) in vec2  fragUV;
layout(location = 2) in float fragSurfaceId;
layout(location = 3) in vec3  fragWorldPos;

layout(location = 0) out vec4 outColor;

// Surface colour lookup table — indices match RoadBuilder surfaceId values:
// 0=Tarmac  1=Gravel  2=Grass  3=Snow  4=Ice  5=Rock  6=Sand
// 7=CliffFace (rock wall)  8=Ground (flat at top/bottom of cliff)
const vec3 SURFACE_COLORS[9] = vec3[](
    vec3(0.45, 0.45, 0.47),   // 0 Tarmac      – dark grey asphalt
    vec3(0.55, 0.48, 0.35),   // 1 Gravel       – tan
    vec3(0.30, 0.40, 0.18),   // 2 Grass        – green-brown
    vec3(0.88, 0.94, 0.98),   // 3 Snow         – blue-white
    vec3(0.70, 0.88, 0.95),   // 4 Ice          – light blue
    vec3(0.48, 0.40, 0.30),   // 5 Rock         – grey-brown
    vec3(0.85, 0.78, 0.55),   // 6 Sand         – sandy yellow
    vec3(0.45, 0.38, 0.28),   // 7 CliffFace    – dark grey-brown rock
    vec3(0.25, 0.38, 0.15)    // 8 Ground       – grass green
);

// Hash function for procedural noise
float hash(vec2 p)
{
    vec3 p3 = fract(vec3(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

// Value noise
float noise(vec2 p)
{
    vec2 i = floor(p);
    vec2 f = fract(p);
    f = f * f * (3.0 - 2.0 * f);  // smoothstep

    float a = hash(i);
    float b = hash(i + vec2(1.0, 0.0));
    float c = hash(i + vec2(0.0, 1.0));
    float d = hash(i + vec2(1.0, 1.0));

    return mix(mix(a, b, f.x), mix(c, d, f.x), f.y);
}

void main()
{
    // Directional sun
    vec3 sunDir  = normalize(vec3(0.4, 0.8, 0.3));
    vec3 norm    = normalize(fragNormal);

    float NdotL  = dot(norm, sunDir);
    float diffuse = max(NdotL, 0.0);

    // Wrap lighting: fills in shadows a bit (half-lambert feel)
    float wrap = max(NdotL * 0.5 + 0.5, 0.0);

    float ambient = 0.25;
    float light   = ambient + wrap * 0.55 + diffuse * 0.20;

    int   id    = clamp(int(fragSurfaceId), 0, 8);
    vec3  baseColor = SURFACE_COLORS[id];

    // --- Procedural detail for tarmac (id 0) ---
    if (id == 0) {
        // Asphalt grain: two octaves of noise at different scales
        float grain1 = noise(fragWorldPos.xz * 3.0);
        float grain2 = noise(fragWorldPos.xz * 12.0);
        float grain  = grain1 * 0.6 + grain2 * 0.4;

        // Subtle variation: +/-8% brightness
        baseColor *= 0.92 + grain * 0.16;

        // Centre line dashes: v=0.50 is road centre (from RoadBuilder UV layout)
        // Draw dashed white line ±0.005 around v=0.50
        float centerDist = abs(fragUV.y - 0.50);
        if (centerDist < 0.004) {
            // Dashes every 3m along road (u = distance along road in meters)
            float dashPhase = mod(fragUV.x, 6.0);
            if (dashPhase < 2.0) {
                baseColor = mix(baseColor, vec3(0.85, 0.85, 0.80), 0.7);
            }
        }

        // Edge lines at road edges: v=0.25 (left) and v=0.75 (right)
        float leftEdgeDist  = abs(fragUV.y - 0.25);
        float rightEdgeDist = abs(fragUV.y - 0.75);
        float edgeLine = min(leftEdgeDist, rightEdgeDist);
        if (edgeLine < 0.003) {
            baseColor = mix(baseColor, vec3(0.85, 0.85, 0.80), 0.5);
        }

        // Subtle specular for wet-look surface curvature
        vec3 viewDir = normalize(vec3(0.0, 1.0, 0.2));  // approximate top-down-ish view
        vec3 halfVec = normalize(sunDir + viewDir);
        float spec = pow(max(dot(norm, halfVec), 0.0), 32.0);
        baseColor += vec3(0.06) * spec;
    }

    // --- Grass / ground noise ---
    if (id == 2 || id == 8) {
        float grassNoise = noise(fragWorldPos.xz * 2.0);
        float grassDetail = noise(fragWorldPos.xz * 8.0);
        float combined = grassNoise * 0.6 + grassDetail * 0.4;
        baseColor *= 0.85 + combined * 0.30;
    }

    // --- Gravel noise ---
    if (id == 1) {
        float gravelNoise = noise(fragWorldPos.xz * 6.0);
        baseColor *= 0.88 + gravelNoise * 0.24;
    }

    vec3 color = baseColor * light;

    // Distance fog (subtle, helps depth perception)
    float dist = length(fragWorldPos);
    float fog  = 1.0 - exp(-dist * dist * 0.000003);
    vec3 fogColor = vec3(0.65, 0.72, 0.82);
    color = mix(color, fogColor, fog);

    outColor = vec4(color, 1.0);
}
