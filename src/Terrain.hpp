#pragma once
#include <glm/glm.hpp>
#include <vector>
#include <cmath>
#include <cstdint>
#include <algorithm>

struct TerrainContact {
    float penetration = 0.f;
    glm::vec3 normal{0.f, 1.f, 0.f};
};

struct GroundLabel {
    const char* text;
    glm::vec3 position;
    float heading;
};

class Terrain {
public:
    static constexpr int   N    = 401;      // 401×401 vertices
    static constexpr float CELL = 0.5f;     // meters per cell
    static constexpr float HALF = (N - 1) * CELL * 0.5f;  // 100m

    enum class Surface : uint8_t {
        Grass, Tarmac, Concrete, Dirt, Gravel, Sand
    };

    Terrain();

    // --- Physics queries (fast, called per-wheel per-frame) ---
    float     heightAt(float x, float z) const;
    glm::vec3 normalAt(float x, float z) const;
    TerrainContact contactAt(const glm::vec3& pos) const;

    // --- Grid access ---
    float  height(int ix, int iz) const { return heights_[iz * N + ix]; }
    float& height(int ix, int iz)       { return heights_[iz * N + ix]; }
    Surface surfaceAt(int ix, int iz) const { return (Surface)surface_[iz * N + ix]; }
    void    setSurface(int ix, int iz, Surface s) { surface_[iz * N + ix] = (uint8_t)s; }

    // --- Coordinate helpers ---
    float worldX(int ix) const { return ix * CELL - HALF; }
    float worldZ(int iz) const { return iz * CELL - HALF; }

    // --- Stamp functions (paint analytical shapes into the grid) ---
    void stampCosine(float cx, float cz, float hw, float hl, float h, float heading);
    void stampRoller(float cx, float cz, float hw, float hl, float h, float heading);
    void stampBowl(float cx, float cz, float radius, float depth);
    void stampWall(float cx, float cz, float hw, float hl, float h, float heading);
    void stampPipe(float cx, float cz, float radius, float hl, float heading);
    void stampColumn(float cx, float cz, float radius, float h);
    void stampBank(float cx, float cz, float hw, float hl, float bankHeight, float heading);
    void stampQuarterPipe(float cx, float cz, float radius, float hl, float heading);

    // --- Surface painting ---
    void setSurfaceRect(float cx, float cz, float hw, float hl, float heading, Surface s);
    void setSurfaceCircle(float cx, float cz, float radius, Surface s);

    // --- Playground generation ---
    void generate();
    const std::vector<GroundLabel>& labels() const { return labels_; }

    // --- Surface color (for rendering) ---
    static glm::vec4 surfaceColor(Surface s);

private:
    std::vector<float>   heights_;
    std::vector<uint8_t> surface_;
    std::vector<GroundLabel> labels_;

    // Grid bounding box for a world-space region
    void gridRange(float cx, float cz, float extent,
                   int& ixMin, int& ixMax, int& izMin, int& izMax) const;

    // Rotate world point into bump-local coordinates
    static void toLocal(float wx, float wz, float cx, float cz, float heading,
                        float& lx, float& lz);
};
