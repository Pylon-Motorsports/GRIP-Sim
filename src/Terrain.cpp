#include "Terrain.hpp"

static constexpr float PI = 3.14159265f;

// ─────────────────────────────────────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────────────────────────────────────

Terrain::Terrain()
    : heights_(N * N, 0.f)
    , surface_(N * N, (uint8_t)Surface::Grass)
{}

// ─────────────────────────────────────────────────────────────────────────────
// Physics queries
// ─────────────────────────────────────────────────────────────────────────────

float Terrain::heightAt(float x, float z) const
{
    // Map world to continuous grid coords
    float gx = (x + HALF) / CELL;
    float gz = (z + HALF) / CELL;

    // Outside the grid: flat ground at y=0
    if (gx < 0.f || gx > (float)(N - 1) || gz < 0.f || gz > (float)(N - 1))
        return 0.f;


    int ix = std::min((int)gx, N - 2);
    int iz = std::min((int)gz, N - 2);
    float fx = gx - ix;
    float fz = gz - iz;

    float h00 = height(ix,     iz);
    float h10 = height(ix + 1, iz);
    float h01 = height(ix,     iz + 1);
    float h11 = height(ix + 1, iz + 1);

    return h00 * (1 - fx) * (1 - fz)
         + h10 * fx       * (1 - fz)
         + h01 * (1 - fx) * fz
         + h11 * fx       * fz;
}

glm::vec3 Terrain::normalAt(float x, float z) const
{
    // Central differences
    float dhdx = (heightAt(x + CELL, z) - heightAt(x - CELL, z)) / (2.f * CELL);
    float dhdz = (heightAt(x, z + CELL) - heightAt(x, z - CELL)) / (2.f * CELL);
    return glm::normalize(glm::vec3{-dhdx, 1.f, -dhdz});
}

TerrainContact Terrain::contactAt(const glm::vec3& pos) const
{
    TerrainContact tc;
    tc.penetration = heightAt(pos.x, pos.z) - pos.y;
    if (tc.penetration > 0.f)
        tc.normal = normalAt(pos.x, pos.z);
    return tc;
}

// ─────────────────────────────────────────────────────────────────────────────
// Helpers
// ─────────────────────────────────────────────────────────────────────────────

void Terrain::gridRange(float cx, float cz, float extent,
                        int& ixMin, int& ixMax, int& izMin, int& izMax) const
{
    ixMin = std::max(0, (int)((cx - extent + HALF) / CELL));
    ixMax = std::min(N - 1, (int)((cx + extent + HALF) / CELL) + 1);
    izMin = std::max(0, (int)((cz - extent + HALF) / CELL));
    izMax = std::min(N - 1, (int)((cz + extent + HALF) / CELL) + 1);
}

void Terrain::toLocal(float wx, float wz, float cx, float cz, float heading,
                      float& lx, float& lz)
{
    float dx = wx - cx;
    float dz = wz - cz;
    float cs = std::cos(heading);
    float sn = std::sin(heading);
    lx =  dx * cs + dz * sn;
    lz = -dx * sn + dz * cs;
}

// ─────────────────────────────────────────────────────────────────────────────
// Stamp functions
// ─────────────────────────────────────────────────────────────────────────────

void Terrain::stampCosine(float cx, float cz, float hw, float hl, float h, float heading)
{
    float extent = std::max(hw, hl) * 1.5f;
    int ixMin, ixMax, izMin, izMax;
    gridRange(cx, cz, extent, ixMin, ixMax, izMin, izMax);

    for (int iz = izMin; iz <= izMax; ++iz) {
        for (int ix = ixMin; ix <= ixMax; ++ix) {
            float lx, lz;
            toLocal(worldX(ix), worldZ(iz), cx, cz, heading, lx, lz);
            float alx = std::abs(lx), alz = std::abs(lz);
            if (alx >= hw || alz >= hl) continue;
            float bh = h * 0.5f * (1.f + std::cos(PI * alx / hw))
                          * 0.5f * (1.f + std::cos(PI * alz / hl));
            height(ix, iz) = std::max(height(ix, iz), bh);
        }
    }
}

void Terrain::stampRoller(float cx, float cz, float hw, float hl, float h, float heading)
{
    float extent = std::max(hw, hl) * 1.5f;
    int ixMin, ixMax, izMin, izMax;
    gridRange(cx, cz, extent, ixMin, ixMax, izMin, izMax);

    for (int iz = izMin; iz <= izMax; ++iz) {
        for (int ix = ixMin; ix <= ixMax; ++ix) {
            float lx, lz;
            toLocal(worldX(ix), worldZ(iz), cx, cz, heading, lx, lz);
            float alx = std::abs(lx);
            if (alx >= hw || std::abs(lz) >= hl) continue;
            float bh = h * 0.5f * (1.f + std::cos(PI * alx / hw));
            height(ix, iz) = std::max(height(ix, iz), bh);
        }
    }
}

void Terrain::stampBowl(float cx, float cz, float radius, float depth)
{
    int ixMin, ixMax, izMin, izMax;
    gridRange(cx, cz, radius + CELL, ixMin, ixMax, izMin, izMax);

    for (int iz = izMin; iz <= izMax; ++iz) {
        for (int ix = ixMin; ix <= ixMax; ++ix) {
            float dx = worldX(ix) - cx;
            float dz = worldZ(iz) - cz;
            float r = std::sqrt(dx * dx + dz * dz);
            if (r >= radius) continue;
            // Cosine profile: -depth at center, 0 at rim, smooth both ends
            float bh = -depth * 0.5f * (1.f + std::cos(PI * r / radius));
            height(ix, iz) = std::min(height(ix, iz), bh);
        }
    }
}

void Terrain::stampWall(float cx, float cz, float hw, float hl, float h, float heading)
{
    float extent = std::max(hw, hl) * 1.5f;
    int ixMin, ixMax, izMin, izMax;
    gridRange(cx, cz, extent, ixMin, ixMax, izMin, izMax);

    for (int iz = izMin; iz <= izMax; ++iz) {
        for (int ix = ixMin; ix <= ixMax; ++ix) {
            float lx, lz;
            toLocal(worldX(ix), worldZ(iz), cx, cz, heading, lx, lz);
            if (std::abs(lx) < hw && std::abs(lz) < hl)
                height(ix, iz) = std::max(height(ix, iz), h);
        }
    }
}

void Terrain::stampPipe(float cx, float cz, float radius, float hl, float heading)
{
    float extent = std::max(radius + CELL, hl) * 1.5f;
    int ixMin, ixMax, izMin, izMax;
    gridRange(cx, cz, extent, ixMin, ixMax, izMin, izMax);

    for (int iz = izMin; iz <= izMax; ++iz) {
        for (int ix = ixMin; ix <= ixMax; ++ix) {
            float lx, lz;
            toLocal(worldX(ix), worldZ(iz), cx, cz, heading, lx, lz);
            if (std::abs(lz) >= hl) continue;
            float alx = std::abs(lx);
            if (alx >= radius + CELL) continue;

            float bh;
            if (alx <= radius) {
                // Concave floor
                bh = radius - std::sqrt(radius * radius - lx * lx);
            } else {
                // Side wall (one cell beyond radius)
                bh = radius;
            }
            height(ix, iz) = std::max(height(ix, iz), bh);
        }
    }
}

void Terrain::stampColumn(float cx, float cz, float radius, float h)
{
    // Ensure at least 2-cell radius for reliable collision on a 0.5m grid
    float effR = std::max(radius, CELL * 2.f);
    int ixMin, ixMax, izMin, izMax;
    gridRange(cx, cz, effR + CELL, ixMin, ixMax, izMin, izMax);

    for (int iz = izMin; iz <= izMax; ++iz) {
        for (int ix = ixMin; ix <= ixMax; ++ix) {
            float dx = worldX(ix) - cx;
            float dz = worldZ(iz) - cz;
            if (dx * dx + dz * dz <= effR * effR)
                height(ix, iz) = std::max(height(ix, iz), h);
        }
    }
}

void Terrain::stampBank(float cx, float cz, float hw, float hl,
                        float bankHeight, float heading)
{
    // Bank: linear height ramp across width (left edge = -bankHeight, right = +bankHeight)
    // with cosine fade along length so it blends smoothly at ends
    float extent = std::max(hw, hl) * 1.5f;
    int ixMin, ixMax, izMin, izMax;
    gridRange(cx, cz, extent, ixMin, ixMax, izMin, izMax);

    for (int iz = izMin; iz <= izMax; ++iz) {
        for (int ix = ixMin; ix <= ixMax; ++ix) {
            float lx, lz;
            toLocal(worldX(ix), worldZ(iz), cx, cz, heading, lx, lz);
            if (std::abs(lx) >= hw || std::abs(lz) >= hl) continue;
            // Linear cross-slope: positive lx = right = higher
            float crossSlope = bankHeight * (lx / hw);
            // Cosine fade along length for smooth entry/exit
            float lengthFade = 0.5f * (1.f + std::cos(PI * std::abs(lz) / hl));
            float bh = crossSlope * lengthFade;
            height(ix, iz) += bh;
        }
    }
}

void Terrain::stampQuarterPipe(float cx, float cz, float radius, float hl, float heading)
{
    // Quarter pipe: concave floor on one side only (positive local-X),
    // rising from ground to vertical. Flat ground on the approach side.
    float extent = std::max(radius + CELL, hl) * 1.5f;
    int ixMin, ixMax, izMin, izMax;
    gridRange(cx, cz, extent, ixMin, ixMax, izMin, izMax);

    for (int iz = izMin; iz <= izMax; ++iz) {
        for (int ix = ixMin; ix <= ixMax; ++ix) {
            float lx, lz;
            toLocal(worldX(ix), worldZ(iz), cx, cz, heading, lx, lz);
            if (std::abs(lz) >= hl) continue;

            float bh;
            if (lx < 0.f) {
                // Approach side: flat ground
                continue;
            } else if (lx <= radius) {
                // Concave quarter-circle: h = R - sqrt(R² - x²)
                bh = radius - std::sqrt(radius * radius - lx * lx);
            } else {
                // Top wall (one cell beyond radius)
                bh = radius;
            }
            height(ix, iz) = std::max(height(ix, iz), bh);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Surface painting
// ─────────────────────────────────────────────────────────────────────────────

void Terrain::setSurfaceRect(float cx, float cz, float hw, float hl, float heading, Surface s)
{
    float extent = std::max(hw, hl) * 1.5f;
    int ixMin, ixMax, izMin, izMax;
    gridRange(cx, cz, extent, ixMin, ixMax, izMin, izMax);

    for (int iz = izMin; iz <= izMax; ++iz) {
        for (int ix = ixMin; ix <= ixMax; ++ix) {
            float lx, lz;
            toLocal(worldX(ix), worldZ(iz), cx, cz, heading, lx, lz);
            if (std::abs(lx) < hw && std::abs(lz) < hl)
                setSurface(ix, iz, s);
        }
    }
}

void Terrain::setSurfaceCircle(float cx, float cz, float radius, Surface s)
{
    int ixMin, ixMax, izMin, izMax;
    gridRange(cx, cz, radius + CELL, ixMin, ixMax, izMin, izMax);

    for (int iz = izMin; iz <= izMax; ++iz) {
        for (int ix = ixMin; ix <= ixMax; ++ix) {
            float dx = worldX(ix) - cx;
            float dz = worldZ(iz) - cz;
            if (dx * dx + dz * dz <= radius * radius)
                setSurface(ix, iz, s);
        }
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// Surface colors
// ─────────────────────────────────────────────────────────────────────────────

glm::vec4 Terrain::surfaceColor(Surface s)
{
    switch (s) {
    case Surface::Grass:    return {0.25f, 0.55f, 0.20f, 1.f};
    case Surface::Tarmac:   return {0.30f, 0.30f, 0.32f, 1.f};
    case Surface::Concrete: return {0.65f, 0.65f, 0.68f, 1.f};
    case Surface::Dirt:     return {0.55f, 0.40f, 0.25f, 1.f};
    case Surface::Gravel:   return {0.50f, 0.48f, 0.42f, 1.f};
    case Surface::Sand:     return {0.85f, 0.78f, 0.55f, 1.f};
    }
    return {0.5f, 0.5f, 0.5f, 1.f};
}

// ─────────────────────────────────────────────────────────────────────────────
// Playground generation
// ─────────────────────────────────────────────────────────────────────────────

// Helper: position along angle from origin
static void polarPos(float r, float theta, float& x, float& z)
{
    x = r * std::sin(theta);
    z = r * std::cos(theta);
}

// Helper: stamp at (r, theta), oriented radially
static void stampRadialCosine(Terrain& t, float r, float theta,
                              float hw, float hl, float h)
{
    float cx, cz;
    polarPos(r, theta, cx, cz);
    t.stampCosine(cx, cz, hw, hl, h, theta);
}

static void stampRadialRoller(Terrain& t, float r, float theta,
                              float hw, float hl, float h)
{
    float cx, cz;
    polarPos(r, theta, cx, cz);
    t.stampRoller(cx, cz, hw, hl, h, theta);
}

void Terrain::generate()
{
    // Start flat + grass everywhere
    std::fill(heights_.begin(), heights_.end(), 0.f);
    std::fill(surface_.begin(), surface_.end(), (uint8_t)Surface::Grass);
    labels_.clear();

    // ════════ N (0°): FLAT GROUND ════════════════════════════════════════════
    {
        float theta = 0.f;
        float lx, lz;
        polarPos(18.f, theta, lx, lz);
        labels_.push_back({"FLAT", {lx, 0.01f, lz}, theta});
    }

    // ════════ NE (45°): ROLLING HILLS ════════════════════════════════════════
    {
        float theta = PI * 0.25f;
        float lx, lz;
        polarPos(18.f, theta, lx, lz);
        labels_.push_back({"ROLLING HILLS", {lx, 0.01f, lz}, theta});

        for (int i = 0; i < 8; ++i) {
            float r = 28.f + i * 12.f;
            float h = 0.35f + 0.25f * std::sin(i * 1.3f);
            stampRadialCosine(*this, r, theta, 5.f, 8.f, h);
        }
    }

    // ════════ E (90°): SUNKEN BOWL ══════════════════════════════════════════
    {
        float theta = PI * 0.5f;
        float cx, cz;
        polarPos(55.f, theta, cx, cz);
        labels_.push_back({"SUNKEN BOWL", {cx, 0.01f, cz - 18.f}, theta});

        float radius = 15.24f;  // 100' diameter / 2 = 50' ≈ 15.24m
        float depth  = 5.f;
        stampBowl(cx, cz, radius, depth);
        setSurfaceCircle(cx, cz, radius + 3.f, Surface::Concrete);
        setSurfaceCircle(cx, cz, radius, Surface::Sand);
    }

    // ════════ SE (135°): BANKED CORNERS ═════════════════════════════════════
    {
        float theta = PI * 0.75f;
        float lx, lz;
        polarPos(18.f, theta, lx, lz);
        labels_.push_back({"BANKED CORNERS", {lx, 0.01f, lz}, theta});

        // Wide banked road sections with alternating camber direction.
        // Each section is a wide tarmac strip tilted sideways.
        for (int i = 0; i < 8; ++i) {
            float r = 30.f + i * 12.f;
            float cx, cz;
            polarPos(r, theta, cx, cz);

            // Wide tarmac surface
            float hw = 8.f;   // 16m wide road
            float hl = 6.f;   // 12m long section
            setSurfaceRect(cx, cz, hw, hl, theta, Surface::Tarmac);

            // Alternating bank: cross-slope tilts left then right
            float bankDir = (i % 2 == 0) ? 1.f : -1.f;
            stampBank(cx, cz, hw, hl, 1.2f * bankDir, theta);
        }
    }

    // ════════ S (180°): WINDING ROAD WITH TREES ════════════════════════════
    {
        float theta = PI;
        float lx, lz;
        polarPos(18.f, theta, lx, lz);
        labels_.push_back({"TREE ROAD", {lx, 0.01f, lz}, theta});

        float fwdX = std::sin(theta);
        float fwdZ = std::cos(theta);
        float perpX = std::cos(theta);
        float perpZ = -std::sin(theta);

        // Road segments with gentle curves and elevation changes
        for (int i = 0; i < 12; ++i) {
            float r = 25.f + i * 7.f;
            float cx = r * fwdX;
            float cz = r * fwdZ;

            // Gentle lateral weave
            float weave = 3.f * std::sin(i * 0.8f);
            cx += perpX * weave;
            cz += perpZ * weave;

            // Elevation change (gentle hills along road)
            float roadH = 0.4f * std::sin(i * 0.5f);
            if (roadH > 0.f)
                stampCosine(cx, cz, 4.f, 4.f, roadH, theta);

            // Tarmac surface
            setSurfaceRect(cx, cz, 4.f, 4.f, theta, Surface::Tarmac);

            // Trees on both sides (tall columns)
            for (int side = -1; side <= 1; side += 2) {
                float tx = cx + perpX * (6.f + (i % 3) * 0.5f) * side;
                float tz = cz + perpZ * (6.f + (i % 3) * 0.5f) * side;
                stampColumn(tx, tz, 0.4f, 4.f);
                setSurfaceCircle(tx, tz, 0.4f, Surface::Dirt);
            }
        }
    }

    // ════════ SW (225°): WALL MAZE ══════════════════════════════════════════
    {
        float theta = PI * 1.25f;
        float lx, lz;
        polarPos(18.f, theta, lx, lz);
        labels_.push_back({"MAZE", {lx, 0.01f, lz}, theta});

        float mazeCx, mazeCz;
        polarPos(50.f, theta, mazeCx, mazeCz);

        // Tarmac floor for the maze area
        setSurfaceRect(mazeCx, mazeCz, 18.f, 18.f, theta, Surface::Tarmac);

        // Wall layout: a simple maze grid with openings
        // Outer walls
        float wallH = 2.5f;
        // Outer walls — east wall split with 4m entrance gap facing origin
        stampWall(mazeCx, mazeCz + 15.f, 15.f, 0.3f, wallH, 0.f);  // north wall
        stampWall(mazeCx, mazeCz - 15.f, 15.f, 0.3f, wallH, 0.f);  // south wall
        stampWall(mazeCx - 15.f, mazeCz, 0.3f, 15.f, wallH, 0.f);  // west wall
        stampWall(mazeCx + 15.f, mazeCz + 9.5f, 0.3f, 5.5f, wallH, 0.f);  // east wall top
        stampWall(mazeCx + 15.f, mazeCz - 9.5f, 0.3f, 5.5f, wallH, 0.f);  // east wall bottom

        // Interior walls with gaps for passages
        // Horizontal interior walls
        stampWall(mazeCx - 5.f, mazeCz + 7.5f, 8.f, 0.3f, wallH, 0.f);
        stampWall(mazeCx + 8.f, mazeCz + 7.5f, 5.f, 0.3f, wallH, 0.f);
        stampWall(mazeCx + 3.f, mazeCz, 10.f, 0.3f, wallH, 0.f);
        stampWall(mazeCx - 8.f, mazeCz - 7.5f, 5.f, 0.3f, wallH, 0.f);
        stampWall(mazeCx + 5.f, mazeCz - 7.5f, 8.f, 0.3f, wallH, 0.f);

        // Vertical interior walls
        stampWall(mazeCx - 7.5f, mazeCz + 3.f, 0.3f, 5.f, wallH, 0.f);
        stampWall(mazeCx, mazeCz + 11.f, 0.3f, 4.f, wallH, 0.f);
        stampWall(mazeCx + 7.5f, mazeCz - 3.f, 0.3f, 5.f, wallH, 0.f);
        stampWall(mazeCx, mazeCz - 3.f, 0.3f, 5.f, wallH, 0.f);

        // Set wall surfaces to concrete
        // (Walls already have correct height; mark their footprint)
        for (int iz = 0; iz < N; ++iz) {
            for (int ix = 0; ix < N; ++ix) {
                float dx = worldX(ix) - mazeCx;
                float dz = worldZ(iz) - mazeCz;
                if (std::abs(dx) < 16.f && std::abs(dz) < 16.f) {
                    if (height(ix, iz) > 1.f)
                        setSurface(ix, iz, Surface::Concrete);
                }
            }
        }
    }

    // ════════ W (270°): SPEED BUMPS ═════════════════════════════════════════
    {
        float theta = PI * 1.5f;
        float lx, lz;
        polarPos(18.f, theta, lx, lz);
        labels_.push_back({"SPEED BUMPS", {lx, 0.01f, lz}, theta});

        // Tarmac road strip
        for (int i = 0; i < 10; ++i) {
            float r = 25.f + i * 8.f;
            float cx, cz;
            polarPos(r, theta, cx, cz);
            setSurfaceRect(cx, cz, 5.f, 4.f, theta, Surface::Tarmac);
        }

        // Roller bumps
        for (int i = 0; i < 10; ++i) {
            float r = 25.f + i * 8.f;
            stampRadialRoller(*this, r, theta, 4.f, 0.30f, 0.10f);
        }
    }

    // ════════ NW (315°): QUARTER PIPE ═══════════════════════════════════════
    {
        float theta = PI * 1.75f;
        float lx, lz;
        polarPos(18.f, theta, lx, lz);
        labels_.push_back({"QUARTER PIPE", {lx, 0.01f, lz}, theta});

        float cx, cz;
        polarPos(50.f, theta, cx, cz);

        // Quarter pipe: concave ramp on one side, flat approach on the other.
        // Car drives toward it from the origin and rides up the curved wall.
        float R = 10.f;
        stampQuarterPipe(cx, cz, R, 20.f, theta + PI);
        setSurfaceRect(cx, cz, R + 1.f, 20.f, theta, Surface::Concrete);
    }

    // ════════ START LABEL ═══════════════════════════════════════════════════
    labels_.push_back({"START", {0.f, 0.01f, 0.f}, 0.f});
}
