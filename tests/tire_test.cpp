#include "test_common.hpp"
#include "../src/Tire.hpp"
#include "../src/BrushTire.hpp"
#include "../src/Terrain.hpp"
#include <glm/glm.hpp>

// ============================================================================
// BrushTire unit tests
// ============================================================================

static void testTireDeflectionBounds()
{
    BrushTire t;
    t.radius = 0.30f;
    t.maxDeflection = 0.025f;

    // No contact: hub above ground + R
    CHECK(t.computeDeflection(0.35f, 0.f) == 0.f, "no deflection when hub above ground+R");

    // Partial deflection
    float d = t.computeDeflection(0.29f, 0.f);  // 10mm overlap
    CHECK(APPROX(d, 0.01f, 0.001f), "correct deflection for 10mm overlap");

    // Beyond dMax: unclamped (bump stop handles force)
    float dOver = t.computeDeflection(0.20f, 0.f);  // way below ground
    CHECK(dOver > t.maxDeflection, "deflection exceeds dMax (bump stop region)");
}

static void testTireNormalForce()
{
    BrushTire t;
    t.radialStiffness = 200000.f;
    t.radialDamping = 500.f;

    // Spring force at 10mm deflection: 200000 * 0.01 = 2000 N
    float Fn = t.computeNormalForce(0.01f, 0.f);
    CHECK(APPROX(Fn, 2000.f, 1.f), "normal force from spring at 10mm");

    // Damper adds force when compressing (dDot > 0)
    float FnDamped = t.computeNormalForce(0.01f, 1.f);  // 1 m/s compress
    CHECK(FnDamped > Fn, "damper adds force during compression");
    CHECK(APPROX(FnDamped, 2500.f, 1.f), "spring + damper force correct");

    // Tire can't pull: if damper overwhelms spring, force floors at 0
    float FnRebound = t.computeNormalForce(0.001f, -5.f);  // fast rebound
    CHECK(FnRebound >= 0.f, "tire can't pull (normal force >= 0)");

    // Bump stop: force increases dramatically beyond maxDeflection
    float FnAtMax = t.computeNormalForce(t.maxDeflection, 0.f);
    float FnBeyond = t.computeNormalForce(t.maxDeflection + 0.03f, 0.f);
    CHECK(FnBeyond > FnAtMax * 2.f, "bump stop adds significant force beyond dMax");

    // No contact
    CHECK(t.computeNormalForce(0.f, 0.f) == 0.f, "zero force at zero deflection");
}

static void testTireContactPatch()
{
    BrushTire t;
    t.radius = 0.30f;
    t.width = 0.20f;

    // At 10mm deflection: L = 2*sqrt(2*0.3*0.01 - 0.01^2) = 2*sqrt(0.0059) ~ 0.154m
    t.updateContactPatch(0.01f);
    CHECK(t.contactPatchLength > 0.14f && t.contactPatchLength < 0.16f,
          "contact patch length reasonable at 10mm deflection");
    CHECK(APPROX(t.contactPatchArea, t.width * t.contactPatchLength, 0.001f),
          "patch area = width * length");

    // Zero deflection = no patch
    t.updateContactPatch(0.f);
    CHECK(t.contactPatchArea == 0.f, "no patch at zero deflection");

    // More deflection = bigger patch
    t.updateContactPatch(0.005f);
    float smallPatch = t.contactPatchArea;
    t.updateContactPatch(0.020f);
    float bigPatch = t.contactPatchArea;
    CHECK(bigPatch > smallPatch, "more deflection = bigger contact patch");
}

static void testTireBrushModel()
{
    BrushTire t;
    t.mu = 1.0f;
    t.slipStiffnessPerArea = 3.0e6f;
    t.width = 0.20f;
    t.radius = 0.30f;

    // Setup a loaded tire with contact
    t.updateContactPatch(0.01f);  // ~10mm deflection
    float Fn = 2000.f;

    // Small slip: force should be roughly linear (proportional to slip)
    float F1 = t.computeLongitudinalForce(0.01f, Fn);
    float F2 = t.computeLongitudinalForce(0.02f, Fn);
    CHECK(F2 > F1, "more slip = more force in linear region");
    CHECK(F1 > 0.f, "positive slip = positive force");

    // Large slip: drops toward muSliding * Fn (sliding friction)
    float Fsat = t.computeLongitudinalForce(1.0f, Fn);
    CHECK(Fsat <= t.mu * Fn + 1.f, "brush model doesn't exceed mu*Fn");
    CHECK(Fsat >= t.muSliding * Fn - 1.f, "brush model stays above muSliding*Fn");

    // Negative slip: negative force
    float Fneg = t.computeLongitudinalForce(-0.05f, Fn);
    CHECK(Fneg < 0.f, "negative slip = negative force");

    // No normal load: no force
    CHECK(t.computeLongitudinalForce(0.1f, 0.f) == 0.f, "no force without normal load");
}

static void testTireLateralBrushModel()
{
    BrushTire t;
    t.mu = 1.0f;
    t.lateralSlipStiffnessPerArea = 1.5e6f;
    t.width = 0.20f;
    t.radius = 0.30f;

    t.updateContactPatch(0.01f);
    float Fn = 2000.f;

    // Positive slip angle (velocity to right) -> negative force (pushes left)
    float F1 = t.computeLateralForce(0.05f, Fn);
    CHECK(F1 < 0.f, "positive slip angle -> negative lateral force");

    // Negative slip angle -> positive force
    float F2 = t.computeLateralForce(-0.05f, Fn);
    CHECK(F2 > 0.f, "negative slip angle -> positive lateral force");

    // Larger slip angle -> larger magnitude
    float F3 = t.computeLateralForce(0.10f, Fn);
    CHECK(std::abs(F3) > std::abs(F1), "more slip angle = more force");

    // Large slip angle: drops toward muSliding * Fn
    float Fsat = t.computeLateralForce(1.0f, Fn);
    CHECK(std::abs(Fsat) <= t.mu * Fn + 1.f, "lateral doesn't exceed mu*Fn");
    CHECK(std::abs(Fsat) >= t.muSliding * Fn - 1.f, "lateral stays above muSliding*Fn");

    // No contact = no force
    CHECK(t.computeLateralForce(0.1f, 0.f) == 0.f, "no lateral force without load");
}

static void testFrictionCircle()
{
    BrushTire t;
    t.mu = 1.0f;
    float Fn = 1000.f;

    // Forces within circle: no change
    float fx = 500.f, fy = 500.f;
    t.frictionCircleClamp(fx, fy, Fn);
    CHECK(APPROX(fx, 500.f, 0.1f), "friction circle: within -> unchanged");

    // Forces exceeding circle: scaled down
    fx = 800.f; fy = 800.f;
    t.frictionCircleClamp(fx, fy, Fn);
    float combined = std::sqrt(fx * fx + fy * fy);
    CHECK(APPROX(combined, 1000.f, 1.f), "friction circle: clamped to mu*Fn");
    CHECK(APPROX(fx, fy, 0.1f), "friction circle: ratio preserved");
}

// ============================================================================
// Terrain contact tests
// ============================================================================

static void testTerrainContactFlat()
{
    // contactAt on flat terrain (default height = 0)
    Terrain terrain;

    // Above ground: no contact
    auto c1 = terrain.contactAt({0.f, 0.1f, 0.f});
    CHECK(c1.penetration <= 0.f, "terrain: above ground = no contact");

    // Below ground: contact
    auto c2 = terrain.contactAt({0.f, -0.01f, 0.f});
    CHECK(c2.penetration > 0.f, "terrain: below ground = contact");
    CHECK(APPROX(c2.normal.y, 1.f, 0.01f), "terrain: flat ground normal = up");
}

static void testTerrainContactPipe()
{
    // Stamp a pipe into terrain and test contact queries
    Terrain terrain;
    terrain.stampPipe(0.f, 0.f, 8.f, 50.f, 0.f);

    // Point inside pipe above floor: no contact
    // Pipe floor at x=3: h = R - sqrt(R²-x²) = 8 - sqrt(64-9) ≈ 0.58
    auto c1 = terrain.contactAt({3.f, 1.f, 0.f});
    CHECK(c1.penetration <= 0.f, "terrain: inside pipe above floor = no contact");

    // Point on pipe floor (below floor height): contact
    auto c2 = terrain.contactAt({0.f, -0.1f, 0.f});
    CHECK(c2.penetration > 0.f, "terrain: below pipe floor = contact");

    // Point outside pipe (past the wall — heightmap clamps to wall height)
    // At the pipe edge, the heightmap should be very tall
    float edgeH = terrain.heightAt(8.f, 0.f);
    std::printf("  pipe edge height at x=8: %.2f\n", edgeH);
    CHECK(edgeH > 2.f, "terrain: pipe wall creates tall heightmap edge");
}

int main()
{
    std::printf("Running tire tests...\n\n");

    // BrushTire
    testTireDeflectionBounds();
    testTireNormalForce();
    testTireContactPatch();
    testTireBrushModel();
    testTireLateralBrushModel();
    testFrictionCircle();

    // Terrain contact
    testTerrainContactFlat();
    testTerrainContactPipe();

    return reportResults();
}
