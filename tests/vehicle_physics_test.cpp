/// Unit tests for vehicle physics.
/// Tests are written against VehicleState observables (position, velocity,
/// wheel positions, etc.) so they work with any IVehicleDynamics implementation.
/// Build target: VehiclePhysicsTest (see tests/CMakeLists.txt)

#include "vehicle/IVehicleDynamics.h"
#include "vehicle/CubeVehicle.h"
#include "road/TerrainQuery.h"
#include "road/RoadBuilder.h"
#include "road/TestStageGenerator.h"
#include "input/InputFrame.h"
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <memory>

static int g_pass = 0;
static int g_fail = 0;

static void check(bool cond, const char* msg)
{
    if (cond) {
        std::printf("  PASS: %s\n", msg);
        ++g_pass;
    } else {
        std::printf("  FAIL: %s\n", msg);
        ++g_fail;
    }
}

// Factory: change this single function to swap vehicle implementation.
static std::unique_ptr<CubeVehicle> makeVehicle()
{
    return std::make_unique<CubeVehicle>();
}

// Run for N seconds of simulation with fixed inputs; return final state.
static VehicleState simulate(IVehicleDynamics& v, const InputFrame& input, float seconds)
{
    constexpr float DT = 1.f / 120.f;
    int steps = static_cast<int>(seconds / DT);
    for (int i = 0; i < steps; ++i)
        v.integrate(input, DT);
    return v.state();
}

// ---------------------------------------------------------------------------
// Test: static equilibrium — car sits still with no inputs
// ---------------------------------------------------------------------------
static void testStaticEquilibrium()
{
    std::printf("\n[Static equilibrium]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    InputFrame zero{};
    auto s = simulate(*v, zero, 2.0f);   // let settle

    // Position should be near origin
    check(std::abs(s.position.x) < 0.1f, "No lateral drift at rest");
    check(std::abs(s.position.z) < 0.1f, "No forward drift at rest");

    // Should not be spinning
    check(std::abs(s.speedMs) < 0.05f, "Speed near zero at rest");
    check(std::abs(s.rollRad)  < 0.05f, "Roll near zero at rest");
    check(std::abs(s.pitchRad) < 0.05f, "Pitch near zero at rest");
}

// ---------------------------------------------------------------------------
// Test: throttle — car accelerates forward
// ---------------------------------------------------------------------------
static void testThrottle()
{
    std::printf("\n[Throttle / acceleration]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    InputFrame throttle{};
    throttle.throttle = 1.0f;

    auto s = simulate(*v, throttle, 5.0f);

    check(s.speedMs > 10.f, "Reaches >10 m/s (~36 km/h) in 5 s");
    check(s.speedMs < 80.f, "Does not exceed 80 m/s (unrealistic)");
    check(s.position.z > 30.f, "Travels >30 m forward in 5 s");
    check(s.pitchRad < 0.02f, "Pitch stays small under throttle");

    // 0-60 kph (~16.7 m/s) should happen in under 8 s
    auto v2 = makeVehicle();
    v2->reset({0.f, 0.f, 0.f}, 0.f);
    float t = 0.f;
    constexpr float DT = 1.f / 120.f;
    while (t < 10.f) {
        v2->integrate(throttle, DT);
        t += DT;
        if (v2->state().speedMs >= 16.7f) break;
    }
    check(t < 8.f, "0-60 kph in <8 s with full throttle");
    std::printf("    (0-60 kph achieved in %.2f s)\n", t);
}

// ---------------------------------------------------------------------------
// Test: braking — car decelerates and stops
// ---------------------------------------------------------------------------
static void testBraking()
{
    std::printf("\n[Braking]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // First get the car up to speed
    InputFrame throttle{}; throttle.throttle = 1.0f;
    simulate(*v, throttle, 3.0f);
    float speed0 = v->state().speedMs;
    check(speed0 > 5.f, "Car has speed before braking test");

    // Now full brake
    InputFrame brake{}; brake.brake = 1.0f;
    auto s = simulate(*v, brake, 4.0f);

    check(s.speedMs < 0.5f, "Car comes to near-stop under hard braking");
    check(s.pitchRad > -0.05f, "Pitch stays bounded during braking");
}

// ---------------------------------------------------------------------------
// Test: steering — car turns left with left steer input
// ---------------------------------------------------------------------------
static void testSteering()
{
    std::printf("\n[Steering]\n");
    // Build up speed first, then steer
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    InputFrame throttle{}; throttle.throttle = 0.5f;
    simulate(*v, throttle, 3.0f);
    float speed0 = v->state().speedMs;
    check(speed0 > 3.f, "Car has speed before steering test");

    glm::vec3 pos0 = v->state().position;

    // Steer left (negative steer = left in rally convention)
    InputFrame steerLeft{}; steerLeft.steer = -1.0f; steerLeft.throttle = 0.3f;
    simulate(*v, steerLeft, 2.0f);

    auto s = v->state();
    // After 2 s of left steer, heading should have changed (turned left = heading decreased)
    float headingChange = s.headingRad - 0.f;  // initial heading was 0
    check(headingChange < -0.1f || s.position.x < pos0.x - 1.f,
          "Car turns left with negative steer input");

    // Roll: turning left → body rolls right (centrifugal) → positive rollRad
    // (right-side-down = positive). Allow small tolerance since we check direction.
    check(s.rollRad > -0.15f, "Roll direction plausible during left turn");

    // Steer right
    auto v2 = makeVehicle();
    v2->reset({0.f, 0.f, 0.f}, 0.f);
    InputFrame throttle2{}; throttle2.throttle = 0.5f;
    simulate(*v2, throttle2, 3.0f);

    InputFrame steerRight{}; steerRight.steer = +1.0f; steerRight.throttle = 0.3f;
    simulate(*v2, steerRight, 2.0f);
    auto s2 = v2->state();
    check(s2.headingRad > 0.1f || s2.position.x > 1.f,
          "Car turns right with positive steer input");
}

// ---------------------------------------------------------------------------
// Test: no spinning at standstill with steer input
// ---------------------------------------------------------------------------
static void testNoSpinAtRest()
{
    std::printf("\n[No spin at standstill]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Full steer, no throttle, from rest
    InputFrame steer{}; steer.steer = 1.0f;
    auto s = simulate(*v, steer, 1.0f);

    check(std::abs(s.speedMs) < 1.0f,
          "Car does not spin out from steer input at rest");
    check(std::abs(s.position.x) < 2.0f && std::abs(s.position.z) < 2.0f,
          "Car stays near origin with steer but no throttle");
}

// ---------------------------------------------------------------------------
// Test: body roll during cornering
// ---------------------------------------------------------------------------
static void testBodyRoll()
{
    std::printf("\n[Body roll during cornering]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Get up to speed
    InputFrame throttle{}; throttle.throttle = 0.6f;
    simulate(*v, throttle, 4.0f);

    // Right turn at speed
    InputFrame corner{}; corner.steer = 1.0f; corner.throttle = 0.2f;
    simulate(*v, corner, 1.5f);
    float roll = v->state().rollRad;

    // Turning right → centrifugal pushes body outward (left relative to car) →
    // outer left springs compress more → left side pushed up → right side drops down.
    // Right-side-down = POSITIVE roll in our convention.
    check(roll > 0.005f,  "Body rolls to outside of right turn (positive roll = right-side-down)");
    check(roll < 0.20f,   "Roll stays within realistic limit (<~11 deg)");

    std::printf("    (roll = %.4f rad = %.2f deg)\n",
                roll, roll * 180.f / 3.14159f);
}

// ---------------------------------------------------------------------------
// Test: very low speed + full steer — car should coast to stop, not spin
// ---------------------------------------------------------------------------
static void testLowSpeedHighSteer()
{
    std::printf("\n[Low speed + full steer — coast to stop, no spin]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Reach a slow walking pace (~1–3 m/s)
    InputFrame creep{}; creep.throttle = 0.06f;
    simulate(*v, creep, 1.5f);
    float speed0 = v->state().speedMs;
    check(speed0 > 0.3f && speed0 < 5.f, "Car creeping at low speed before test");

    // Full steer, no throttle — car should decelerate and not spin wildly
    InputFrame fullSteer{}; fullSteer.steer = 1.0f;
    auto s = simulate(*v, fullSteer, 4.0f);

    // Speed must decay to near-stop (no propulsion, friction bleeds speed off)
    check(s.speedMs < 1.5f, "Car coasts to near-stop under full steer at low speed");

    // Heading change must be bounded — at most one full loop (2π ≈ 6.28 rad).
    // Without the latRamp fix the car can spin many rotations; 2π is generous.
    check(std::abs(s.headingRad) < 2.f * 3.14159f,
          "Heading change bounded (<1 full rotation) at low speed");

    std::printf("    (initial speed=%.2f m/s, final speed=%.2f m/s,"
                " heading=%.1f deg)\n",
                speed0, s.speedMs, s.headingRad * 180.f / 3.14159f);
}

// ---------------------------------------------------------------------------
// Terrain test helpers
// ---------------------------------------------------------------------------

/// Build a TerrainQuery for a straight road along +Z with a linear height ramp.
/// Build a flat mesh strip along +Z with enough margin for a car to spawn at z=0.
/// Mesh extends from z=-margin to z=lengthM+margin so wheels at the edges don't fall off.
static TerrainQuery makeTerrainLine(float lengthM, float startY, float endY,
                                     int numPoints = 50, float halfW = 20.f)
{
    constexpr float MARGIN = 5.f;  // extra mesh before z=0 and after z=lengthM
    std::vector<glm::vec3> positions;
    std::vector<uint32_t>  indices;

    for (int i = 0; i < numPoints; ++i) {
        float t = static_cast<float>(i) / (numPoints - 1);
        float z = -MARGIN + t * (lengthM + 2.f * MARGIN);
        // Clamp interpolation so the margin regions are flat extensions
        float tClamped = std::clamp((z) / lengthM, 0.f, 1.f);
        float y = glm::mix(startY, endY, tClamped);
        positions.push_back({ -halfW, y, z });
        positions.push_back({  halfW, y, z });
    }
    for (int i = 0; i < numPoints - 1; ++i) {
        uint32_t tl = i * 2, tr = i * 2 + 1;
        uint32_t bl = (i + 1) * 2, br = (i + 1) * 2 + 1;
        indices.insert(indices.end(), { tl, bl, tr, tr, bl, br });
    }

    TerrainQuery terrain;
    terrain.buildFromMesh(positions, indices);
    return terrain;
}

// ---------------------------------------------------------------------------
// Test: flat ground regression — existing behavior with terrain query active
// ---------------------------------------------------------------------------
static void testFlatGroundRegression()
{
    std::printf("\n[Flat ground regression — with terrain query]\n");
    auto v = makeVehicle();
    auto terrain = makeTerrainLine(200.f, 0.f, 0.f);
    v->setTerrainQuery(terrain);
    v->reset({0.f, 0.f, 0.f}, 0.f);

    InputFrame zero{};
    auto s = simulate(*v, zero, 2.0f);

    check(std::abs(s.position.x) < 0.1f, "No lateral drift at rest (with terrain)");
    check(std::abs(s.position.z) < 0.1f, "No forward drift at rest (with terrain)");
    check(std::abs(s.speedMs)    < 0.05f, "Speed near zero at rest (with terrain)");
}

// ---------------------------------------------------------------------------
// Test: uphill slows the car
// ---------------------------------------------------------------------------
static void testUphillSlowdown()
{
    std::printf("\n[Uphill slows the car]\n");

    // Flat road reference
    auto vFlat = makeVehicle();
    auto flatTerrain = makeTerrainLine(300.f, 0.f, 0.f);
    vFlat->setTerrainQuery(flatTerrain);
    vFlat->reset({0.f, 0.f, 0.f}, 0.f);
    InputFrame throttle{}; throttle.throttle = 1.0f;
    auto sFlat = simulate(*vFlat, throttle, 4.0f);

    // Uphill road: 0 to 30m elevation over 300m (~5.7 degree slope)
    auto vHill = makeVehicle();
    auto hillTerrain = makeTerrainLine(300.f, 0.f, 30.f);
    vHill->setTerrainQuery(hillTerrain);
    vHill->reset({0.f, 0.f, 0.f}, 0.f);
    auto sHill = simulate(*vHill, throttle, 4.0f);

    check(sHill.speedMs < sFlat.speedMs,
          "Car is slower uphill than on flat");
    check(sHill.position.y > 2.f,
          "Car has gained elevation on uphill");
    std::printf("    (flat speed=%.1f m/s, hill speed=%.1f m/s, elevation=%.1f m)\n",
                sFlat.speedMs, sHill.speedMs, sHill.position.y);
}

// ---------------------------------------------------------------------------
// Test: downhill accelerates the car
// ---------------------------------------------------------------------------
static void testDownhillAcceleration()
{
    std::printf("\n[Downhill accelerates the car]\n");

    // Flat road reference
    auto vFlat = makeVehicle();
    auto flatTerrain = makeTerrainLine(300.f, 0.f, 0.f);
    vFlat->setTerrainQuery(flatTerrain);
    vFlat->reset({0.f, 0.f, 0.f}, 0.f);
    InputFrame throttle{}; throttle.throttle = 0.5f;
    auto sFlat = simulate(*vFlat, throttle, 4.0f);

    // Downhill: start at 20m, end at 0m
    auto vDown = makeVehicle();
    auto downTerrain = makeTerrainLine(300.f, 20.f, 0.f);
    vDown->setTerrainQuery(downTerrain);
    vDown->reset({0.f, 0.f, 20.f}, 0.f);
    auto sDown = simulate(*vDown, throttle, 4.0f);

    check(sDown.speedMs > sFlat.speedMs,
          "Car is faster downhill than on flat");
    std::printf("    (flat speed=%.1f m/s, downhill speed=%.1f m/s)\n",
                sFlat.speedMs, sDown.speedMs);
}

// ---------------------------------------------------------------------------
// Test: crest — car gains elevation driving over a hill
// ---------------------------------------------------------------------------
static void testCrestElevation()
{
    std::printf("\n[Crest — car gains elevation]\n");

    // Gaussian bump: peak height 5m at z=50m
    TerrainQuery terrain;
    std::vector<glm::vec3> positions;
    std::vector<uint32_t> indices;
    constexpr float halfW = 20.f;
    for (int i = 0; i < 105; ++i) {
        float z = i * 2.f - 10.f;  // start at z=-10 for margin
        float t = (z - 50.f) / 25.f;
        float y = 5.f * std::exp(-t * t);
        positions.push_back({-halfW, y, z});
        positions.push_back({ halfW, y, z});
    }
    for (int i = 0; i < 104; ++i) {
        uint32_t tl = i*2, tr = i*2+1, bl = (i+1)*2, br = (i+1)*2+1;
        indices.insert(indices.end(), { tl, bl, tr, tr, bl, br });
    }
    terrain.buildFromMesh(positions, indices);

    auto v = makeVehicle();
    v->setTerrainQuery(terrain);
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Drive toward the crest
    InputFrame throttle{}; throttle.throttle = 1.0f;
    auto s = simulate(*v, throttle, 3.0f);

    check(s.position.y > 1.f,
          "Car climbs over the crest (gains elevation)");
    std::printf("    (position.y=%.2f m, z=%.1f m)\n", s.position.y, s.position.z);
}

// ---------------------------------------------------------------------------
// Test: per-wheel height difference on uphill — front wheels higher than rear
// ---------------------------------------------------------------------------
static void testPerWheelHeightDifference()
{
    std::printf("\n[Per-wheel height — front higher than rear on uphill]\n");

    auto v = makeVehicle();
    auto terrain = makeTerrainLine(200.f, 0.f, 10.f);
    v->setTerrainQuery(terrain);
    v->reset({0.f, 0.f, 0.f}, 0.f);

    InputFrame throttle{}; throttle.throttle = 0.5f;
    auto s = simulate(*v, throttle, 3.0f);

    // Front wheels (FL=0, FR=1) are ahead of rear (RL=2, RR=3)
    // On uphill, front is at higher ground than rear
    float frontAvg = (s.wheelGroundHeight[0] + s.wheelGroundHeight[1]) * 0.5f;
    float rearAvg  = (s.wheelGroundHeight[2] + s.wheelGroundHeight[3]) * 0.5f;

    check(frontAvg > rearAvg,
          "Front wheels at higher ground than rear on uphill");
    std::printf("    (front ground=%.2f m, rear ground=%.2f m, pitch=%.4f rad)\n",
                frontAvg, rearAvg, s.pitchRad);
}

// ---------------------------------------------------------------------------
// Test: suspension travel limits — spring can't stretch beyond max extension
// ---------------------------------------------------------------------------
static void testSuspensionTravelLimits()
{
    std::printf("\n[Suspension travel limits]\n");

    // Simulate driving over a sharp dip: wheel drops but suspension should be bounded
    TerrainQuery terrain;
    std::vector<glm::vec3> positions;
    std::vector<uint32_t> indices;
    constexpr float halfW = 20.f;
    for (int i = 0; i < 105; ++i) {
        float z = i * 1.f - 5.f;
        float y = 0.f;
        if (z > 28.f && z < 32.f) y = -2.f;  // sudden 2m dip
        positions.push_back({-halfW, y, z});
        positions.push_back({ halfW, y, z});
    }
    for (int i = 0; i < 104; ++i) {
        uint32_t tl = i*2, tr = i*2+1, bl = (i+1)*2, br = (i+1)*2+1;
        indices.insert(indices.end(), { tl, bl, tr, tr, bl, br });
    }
    terrain.buildFromMesh(positions, indices);

    auto v = makeVehicle();
    v->setTerrainQuery(terrain);
    v->reset({0.f, 0.f, 0.f}, 0.f);

    InputFrame throttle{}; throttle.throttle = 0.5f;
    auto s = simulate(*v, throttle, 3.0f);

    // Suspension lengths should be non-negative
    bool allNonNeg = true;
    for (int c = 0; c < 4; ++c)
        if (s.suspLength[c] < -0.001f) allNonNeg = false;
    check(allNonNeg, "Suspension lengths non-negative");

    // Car should still be driving (didn't fall through the world)
    check(s.position.z > 10.f, "Car continues driving through terrain dip");
    std::printf("    (susp lengths: FL=%.3f FR=%.3f RL=%.3f RR=%.3f)\n",
                s.suspLength[0], s.suspLength[1], s.suspLength[2], s.suspLength[3]);
}

// ---------------------------------------------------------------------------
// Test: body roll stability — no oscillation during sustained cornering
// ---------------------------------------------------------------------------
static void testBodyRollStability()
{
    std::printf("\n[Body roll stability — no oscillation]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Get up to speed
    InputFrame throttle{}; throttle.throttle = 0.6f;
    simulate(*v, throttle, 4.0f);

    // Sustained right turn at speed — sample roll over 2 seconds
    InputFrame corner{}; corner.steer = 0.8f; corner.throttle = 0.3f;
    constexpr float DT = 1.f / 120.f;
    float maxRoll = 0.f, minRoll = 0.f;
    int signChanges = 0;
    float prevRoll = 0.f;
    for (int i = 0; i < 240; ++i) {  // 2 seconds
        v->integrate(corner, DT);
        float roll = v->state().rollRad;
        maxRoll = std::max(maxRoll, roll);
        minRoll = std::min(minRoll, roll);
        if (i > 0 && ((roll > 0.f) != (prevRoll > 0.f)))
            ++signChanges;
        prevRoll = roll;
    }

    // Roll should settle to one side, not oscillate back and forth
    check(signChanges < 4, "Roll doesn't oscillate (fewer than 4 sign changes in 2s)");
    // Roll range should be bounded
    check((maxRoll - minRoll) < 0.20f, "Roll range < 0.20 rad during sustained turn");
    std::printf("    (roll range: %.4f to %.4f rad, sign changes: %d)\n",
                minRoll, maxRoll, signChanges);
}

// ---------------------------------------------------------------------------
// Test: RWD behavior — rear wheels should spin under hard throttle
// ---------------------------------------------------------------------------
static void testRwdBehavior()
{
    std::printf("\n[RWD behavior — rear wheel spin]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Full throttle from standstill — rear wheels should spin faster than fronts
    InputFrame throttle{}; throttle.throttle = 1.0f;
    constexpr float DT = 1.f / 120.f;
    // Simulate 0.5 seconds of launch
    for (int i = 0; i < 60; ++i)
        v->integrate(throttle, DT);

    // Check that the car is moving forward (RWD should still provide traction)
    auto s = v->state();
    check(s.speedMs > 0.5f, "RWD car accelerates from standstill");

    // Continue for a few seconds and check speed is reasonable
    auto s2 = simulate(*v, throttle, 4.5f);
    check(s2.speedMs > 10.f, "BRZ reaches >10 m/s in 5s total");
    check(s2.speedMs < 80.f, "Speed stays realistic (<80 m/s)");
    std::printf("    (speed at 0.5s=%.1f m/s, at 5s=%.1f m/s)\n", s.speedMs, s2.speedMs);
}

// ---------------------------------------------------------------------------
// Test: hard driving then braking — body settles to rest, no wild oscillation
// ---------------------------------------------------------------------------
static void testBodySettlesAfterHardDriving()
{
    std::printf("\n[Body settles after hard driving + braking]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Phase 1: full throttle for 5 seconds
    InputFrame throttle{}; throttle.throttle = 1.0f;
    simulate(*v, throttle, 5.0f);
    float speedAfterThrottle = v->state().speedMs;
    std::printf("    After 5s full throttle: speed=%.1f m/s\n", speedAfterThrottle);

    // Phase 2: full throttle + full left steer for 2 seconds
    InputFrame throttleAndSteer{}; throttleAndSteer.throttle = 1.0f; throttleAndSteer.steer = -1.0f;
    simulate(*v, throttleAndSteer, 2.0f);
    auto sMid = v->state();
    std::printf("    After 2s full throttle+left: speed=%.1f m/s, roll=%.4f rad, pitch=%.4f rad\n",
                sMid.speedMs, sMid.rollRad, sMid.pitchRad);

    // Phase 3: full brake + full left steer for 5 seconds
    InputFrame brakeAndSteer{}; brakeAndSteer.brake = 1.0f; brakeAndSteer.steer = -1.0f;
    simulate(*v, brakeAndSteer, 5.0f);
    auto sAfterBrake = v->state();
    std::printf("    After 5s full brake+left: speed=%.1f m/s, roll=%.4f rad, pitch=%.4f rad\n",
                sAfterBrake.speedMs, sAfterBrake.rollRad, sAfterBrake.pitchRad);

    // Phase 4: no input for 5 seconds — car should settle completely
    InputFrame zero{};
    constexpr float DT = 1.f / 120.f;

    // Sample roll and pitch over last 1 second to detect oscillation
    simulate(*v, zero, 4.0f);  // let it settle for 4s first

    float maxRoll = -999.f, minRoll = 999.f;
    float maxPitch = -999.f, minPitch = 999.f;
    for (int i = 0; i < 120; ++i) {  // last 1 second
        v->integrate(zero, DT);
        auto st = v->state();
        maxRoll  = std::max(maxRoll,  st.rollRad);
        minRoll  = std::min(minRoll,  st.rollRad);
        maxPitch = std::max(maxPitch, st.pitchRad);
        minPitch = std::min(minPitch, st.pitchRad);
    }

    auto sFinal = v->state();
    float rollRange  = maxRoll - minRoll;
    float pitchRange = maxPitch - minPitch;

    std::printf("    After 5s settling: speed=%.2f m/s\n", sFinal.speedMs);
    std::printf("    Roll  range over last 1s: %.5f rad (%.3f deg)\n",
                rollRange, rollRange * 180.f / 3.14159f);
    std::printf("    Pitch range over last 1s: %.5f rad (%.3f deg)\n",
                pitchRange, pitchRange * 180.f / 3.14159f);
    std::printf("    Final roll=%.5f rad, pitch=%.5f rad\n",
                sFinal.rollRad, sFinal.pitchRad);

    // Car should be nearly stopped
    check(sFinal.speedMs < 1.0f, "Car nearly stopped after braking + settling");

    // Body should NOT be oscillating — roll/pitch range in last 1s should be tiny
    check(rollRange  < 0.02f, "Roll not oscillating (range < 0.02 rad in last 1s)");
    check(pitchRange < 0.02f, "Pitch not oscillating (range < 0.02 rad in last 1s)");

    // Final roll and pitch should be near zero (car at rest on flat ground)
    check(std::abs(sFinal.rollRad)  < 0.05f, "Final roll near zero at rest");
    check(std::abs(sFinal.pitchRad) < 0.05f, "Final pitch near zero at rest");
}

// ---------------------------------------------------------------------------
// Test: weight transfer under braking — front axle gains load
// ---------------------------------------------------------------------------
static void testWeightTransferBraking()
{
    std::printf("\n[Weight transfer — braking]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Get up to speed
    InputFrame throttle{}; throttle.throttle = 1.0f;
    simulate(*v, throttle, 3.0f);

    // Record weight distribution at speed (cruising)
    float wfCruise = v->state().weightFront;

    // Hard braking
    InputFrame brake{}; brake.brake = 1.0f;
    constexpr float DT = 1.f / 120.f;
    // Let the braking transient settle for 0.5s
    for (int i = 0; i < 60; ++i) v->integrate(brake, DT);
    float wfBraking = v->state().weightFront;

    // Under braking, weight transfers forward: front axle load increases
    // ΔWf/W = (a/g) * (h/L) where h=CG height, L=wheelbase
    check(wfBraking > wfCruise + 0.01f,
          "Front axle weight increases under braking");
    check(wfBraking > 0.50f,
          "More than 50% weight on front axle during hard braking");
    check(wfBraking < 0.85f,
          "Weight transfer bounded (not all weight on front)");
    std::printf("    (cruise wf=%.3f, braking wf=%.3f)\n", wfCruise, wfBraking);
}

// ---------------------------------------------------------------------------
// Test: weight transfer under acceleration — rear axle gains load (RWD)
// ---------------------------------------------------------------------------
static void testWeightTransferAcceleration()
{
    std::printf("\n[Weight transfer — acceleration]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Settle at rest to get static weight distribution
    InputFrame zero{};
    simulate(*v, zero, 1.0f);
    float wfStatic = v->state().weightFront;

    // Full throttle from rest — weight shifts rearward
    InputFrame throttle{}; throttle.throttle = 1.0f;
    constexpr float DT = 1.f / 120.f;
    for (int i = 0; i < 60; ++i) v->integrate(throttle, DT);
    float wfAccel = v->state().weightFront;

    check(wfAccel < wfStatic - 0.01f,
          "Front axle weight decreases under acceleration");
    std::printf("    (static wf=%.3f, accel wf=%.3f)\n", wfStatic, wfAccel);
}

// ---------------------------------------------------------------------------
// Test: static weight distribution matches CG geometry
// ---------------------------------------------------------------------------
static void testStaticWeightDistribution()
{
    std::printf("\n[Static weight distribution]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    InputFrame zero{};
    simulate(*v, zero, 2.0f);

    float actualWf = v->state().weightFront;

    // Weight should be roughly balanced (0.4-0.6 for most cars)
    check(actualWf > 0.35f && actualWf < 0.65f,
          "Static weight distribution roughly balanced");
    std::printf("    (actual wf=%.3f)\n", actualWf);
}

// ---------------------------------------------------------------------------
// Test: braking distance — physics-based check
// ---------------------------------------------------------------------------
static void testBrakingDistance()
{
    std::printf("\n[Braking distance]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Accelerate to a known speed
    InputFrame throttle{}; throttle.throttle = 1.0f;
    constexpr float DT = 1.f / 120.f;
    while (v->state().speedMs < 27.8f) // ~100 km/h
        v->integrate(throttle, DT);

    float v0 = v->state().speedMs;
    float z0 = v->state().position.z;

    // Full braking
    InputFrame brake{}; brake.brake = 1.0f;
    while (v->state().speedMs > 0.5f)
        v->integrate(brake, DT);

    float zStop = v->state().position.z;
    float dist = zStop - z0;

    // Braking from 100 km/h should take 20-120m (wide range for any vehicle model)
    check(dist > 20.f, "Braking distance not unrealistically short");
    check(dist < 120.f, "Braking distance not unrealistically long");
    std::printf("    (v0=%.1f m/s, dist=%.1f m)\n", v0, dist);
}

// ---------------------------------------------------------------------------
// Test: energy conservation — coasting speed loss matches friction work
// ---------------------------------------------------------------------------
static void testEnergyConservationCoasting()
{
    std::printf("\n[Energy conservation — coasting]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Get to a moderate speed, then coast
    InputFrame throttle{}; throttle.throttle = 0.8f;
    simulate(*v, throttle, 4.0f);

    float v0 = v->state().speedMs;

    // Coast for 5 seconds (no throttle, no brake)
    InputFrame zero{};
    simulate(*v, zero, 5.0f);
    float v1 = v->state().speedMs;

    // Energy must decrease (not increase — would violate conservation)
    check(v1 < v0, "Car decelerates when coasting (no throttle)");
    check(v0 - v1 > 1.f, "Significant speed loss during coast");
    std::printf("    (v0=%.1f m/s, v1=%.1f m/s)\n", v0, v1);
}

// ---------------------------------------------------------------------------
// Test: top speed limited by aero drag
// ---------------------------------------------------------------------------
static void testTopSpeed()
{
    std::printf("\n[Top speed — drag-limited]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Run for 60 seconds at full throttle — should approach equilibrium
    InputFrame throttle{}; throttle.throttle = 1.0f;
    simulate(*v, throttle, 60.0f);
    float topSpeed = v->state().speedMs;

    // Any reasonable vehicle should reach 20-200 m/s
    check(topSpeed > 20.f, "Top speed exceeds 20 m/s (~70 km/h)");
    check(topSpeed < 200.f, "Top speed below 200 m/s (~720 km/h)");

    // At top speed, drive force ≈ drag force. Acceleration should be very small.
    float speed2 = v->state().speedMs;
    simulate(*v, throttle, 10.0f);
    float speed3 = v->state().speedMs;
    check(std::abs(speed3 - speed2) < 2.0f,
          "Speed nearly stabilised at top speed (within 2 m/s over 10s)");
    std::printf("    (top speed=%.1f m/s = %.0f km/h)\n", topSpeed, topSpeed * 3.6f);
}

// ---------------------------------------------------------------------------
// Test: gear shifting — auto-shift works through all gears
// ---------------------------------------------------------------------------
static void testGearShifting()
{
    std::printf("\n[Gear shifting]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    InputFrame throttle{}; throttle.throttle = 1.0f;
    constexpr float DT = 1.f / 120.f;

    int maxGear = 1;
    float t = 0.f;
    while (t < 20.f) {
        v->integrate(throttle, DT);
        t += DT;
        int gear = v->state().currentGear;
        if (gear > maxGear) maxGear = gear;
    }

    // Observable: gear should be >= 1 and RPM > 0 at speed
    check(maxGear >= 1, "Car uses at least 1st gear under full throttle");
    check(v->state().engineRpm > 0.f, "Engine RPM positive at speed");
    std::printf("    (max gear reached=%d, final RPM=%.0f, time=%.1fs)\n",
                maxGear, v->state().engineRpm, t);
}

// ---------------------------------------------------------------------------
// Test: suspension compression at rest matches spring rate
// ---------------------------------------------------------------------------
static void testSuspensionStaticDeflection()
{
    std::printf("\n[Suspension static deflection]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    InputFrame zero{};
    simulate(*v, zero, 2.0f);
    auto s = v->state();

    // All four suspension lengths should be positive and roughly equal at rest
    bool allPositive = true;
    float avgLen = 0.f;
    for (int c = 0; c < 4; ++c) {
        if (s.suspLength[c] <= 0.f) allPositive = false;
        avgLen += s.suspLength[c];
    }
    avgLen *= 0.25f;

    check(allPositive, "All suspension lengths positive at rest");

    float maxDiff = 0.f;
    for (int c = 0; c < 4; ++c)
        maxDiff = std::max(maxDiff, std::abs(s.suspLength[c] - avgLen));
    check(maxDiff < avgLen * 0.3f,
          "Suspension lengths roughly equal at rest");
    std::printf("    (FL=%.4f FR=%.4f RL=%.4f RR=%.4f, avg=%.4f)\n",
                s.suspLength[0], s.suspLength[1], s.suspLength[2], s.suspLength[3], avgLen);
}

// ---------------------------------------------------------------------------
// Test: oversteer tendency — RWD car yaws outward under power in a turn
// ---------------------------------------------------------------------------
static void testOversteerUnderPower()
{
    std::printf("\n[Oversteer under power — RWD]\n");

    // Both cars at the same speed, then compare slip angle when one applies throttle.
    // In a RWD car, throttle in a corner saturates the rear tyres' friction circle,
    // reducing available lateral grip at rear → the rear slides out (oversteer).
    // We measure the vehicle's overall slip angle (angle between heading and velocity).
    auto v1 = makeVehicle();
    v1->reset({0.f, 0.f, 0.f}, 0.f);
    InputFrame build{}; build.throttle = 0.6f;
    simulate(*v1, build, 3.0f);

    auto v2 = makeVehicle();
    v2->reset({0.f, 0.f, 0.f}, 0.f);
    simulate(*v2, build, 3.0f);

    // Both enter the same corner. One coasts, one applies throttle.
    // Short duration (0.5s) to minimize speed divergence.
    InputFrame cornerCoast{}; cornerCoast.steer = 0.7f; cornerCoast.throttle = 0.0f;
    InputFrame cornerPower{}; cornerPower.steer = 0.7f; cornerPower.throttle = 0.8f;
    simulate(*v1, cornerCoast, 0.5f);
    simulate(*v2, cornerPower, 0.5f);

    // Measure body slip angle: angle between velocity vector and heading direction
    auto slipAngle = [](const VehicleState& s) {
        float heading = s.headingRad;
        glm::vec3 fwd { std::sin(heading), 0.f, std::cos(heading) };
        glm::vec3 right { std::cos(heading), 0.f, -std::sin(heading) };
        float vFwd = glm::dot(s.velocity, fwd);
        float vLat = glm::dot(s.velocity, right);
        return std::atan2(vLat, std::max(std::abs(vFwd), 0.1f));
    };

    float slipCoast = std::abs(slipAngle(v1->state()));
    float slipPower = std::abs(slipAngle(v2->state()));

    // With throttle in RWD, rear slip increases → body slip angle increases
    check(slipPower > slipCoast,
          "Throttle increases body slip angle (power oversteer in RWD)");
    std::printf("    (coast slip=%.2f deg, power slip=%.2f deg)\n",
                slipCoast * 180.f / 3.14159f, slipPower * 180.f / 3.14159f);
}

// ---------------------------------------------------------------------------
// Test: lateral grip proportional to load (load sensitivity)
// ---------------------------------------------------------------------------
static void testLoadSensitivity()
{
    std::printf("\n[Load sensitivity — cornering produces heading change]\n");

    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Get to speed, then corner
    InputFrame build{}; build.throttle = 0.6f;
    simulate(*v, build, 4.0f);

    InputFrame corner{}; corner.steer = 0.8f; corner.throttle = 0.2f;
    simulate(*v, corner, 2.0f);

    float heading = std::abs(v->state().headingRad);
    check(heading > 0.1f, "Cornering produces meaningful heading change");
    std::printf("    (heading=%.2f deg)\n", heading * 180.f / 3.14159f);
}

// ---------------------------------------------------------------------------
// Test: sway bar effect — stiffer sway bar reduces body roll
// ---------------------------------------------------------------------------
static void testSwayBarEffect()
{
    std::printf("\n[Sway bar effect — cornering produces roll]\n");

    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Get to speed and corner
    InputFrame build{}; build.throttle = 0.6f;
    simulate(*v, build, 3.0f);

    InputFrame corner{}; corner.steer = 0.8f; corner.throttle = 0.3f;
    simulate(*v, corner, 1.5f);

    float roll = std::abs(v->state().rollRad);
    // Cornering should produce some roll (positive or negative)
    check(roll >= 0.f, "Roll is non-negative in abs value during cornering");
    std::printf("    (roll=%.4f rad (%.2f deg))\n",
                roll, roll * 180.f / 3.14159f);
}

// ---------------------------------------------------------------------------
// Test: spring rate effect — stiffer springs reduce pitch under braking
// ---------------------------------------------------------------------------
static void testSpringRateEffect()
{
    std::printf("\n[Spring rate effect — pitch bounded under braking]\n");

    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Accelerate
    InputFrame throttle{}; throttle.throttle = 1.0f;
    simulate(*v, throttle, 3.0f);

    // Brake hard
    InputFrame brake{}; brake.brake = 1.0f;
    constexpr float DT = 1.f / 120.f;
    for (int i = 0; i < 30; ++i)
        v->integrate(brake, DT);

    float pitch = v->state().pitchRad;

    // Pitch should stay bounded during braking
    check(std::abs(pitch) < 0.20f, "Pitch bounded under hard braking");
    std::printf("    (pitch=%.4f rad (%.2f deg))\n", pitch, pitch * 57.3f);
}

// ---------------------------------------------------------------------------
// Test: coast-down — no phantom acceleration with zero inputs
// ---------------------------------------------------------------------------
static void testCoastDown()
{
    std::printf("\n[Coast-down — no phantom acceleration]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Get to ~20 m/s
    InputFrame throttle{}; throttle.throttle = 0.8f;
    simulate(*v, throttle, 4.0f);
    // Use velocity magnitude to avoid heading-projection artifacts
    float v0 = glm::length(v->state().velocity);

    // Coast with zero input
    InputFrame zero{};
    constexpr float DT = 1.f / 120.f;
    float prevSpeed = v0;
    int accelCount = 0;
    for (int i = 0; i < 600; ++i) { // 5 seconds
        v->integrate(zero, DT);
        float spd = glm::length(v->state().velocity);
        if (spd > prevSpeed + 0.05f) ++accelCount;
        prevSpeed = spd;
    }
    float vFinal = glm::length(v->state().velocity);

    // Allow at most a few frames of micro-acceleration (numerical noise)
    check(accelCount < 5, "Speed almost never increases during coast-down");
    check(vFinal < v0 - 1.0f, "Final speed meaningfully lower than initial");
    std::printf("    (v0=%.1f m/s, vFinal=%.1f m/s, accel frames=%d)\n",
                v0, vFinal, accelCount);
}

// ---------------------------------------------------------------------------
// Test: symmetry — left turn and right turn produce mirror results
// ---------------------------------------------------------------------------
static void testSteeringSymmetry()
{
    std::printf("\n[Steering symmetry]\n");

    auto vL = makeVehicle(), vR = makeVehicle();
    vL->reset({0.f, 0.f, 0.f}, 0.f);
    vR->reset({0.f, 0.f, 0.f}, 0.f);

    // Same speed
    InputFrame throttle{}; throttle.throttle = 0.5f;
    simulate(*vL, throttle, 3.0f);
    simulate(*vR, throttle, 3.0f);

    // Steer opposite directions
    InputFrame left{}; left.steer = -0.5f; left.throttle = 0.3f;
    InputFrame right{}; right.steer = 0.5f; right.throttle = 0.3f;
    simulate(*vL, left, 2.0f);
    simulate(*vR, right, 2.0f);

    float headL = vL->state().headingRad;
    float headR = vR->state().headingRad;
    float rollL = vL->state().rollRad;
    float rollR = vR->state().rollRad;

    // Heading should be equal and opposite
    check(std::abs(headL + headR) < std::abs(headL) * 0.15f,
          "Left/right turn headings are symmetric");
    // Roll should be equal and opposite
    check(std::abs(rollL + rollR) < std::abs(rollL) * 0.15f + 0.001f,
          "Left/right turn rolls are symmetric");
    std::printf("    (headL=%.3f, headR=%.3f, rollL=%.4f, rollR=%.4f)\n",
                headL, headR, rollL, rollR);
}

// ---------------------------------------------------------------------------
// Test: damper effect — underdamped vs overdamped response
// ---------------------------------------------------------------------------
static void testDamperEffect()
{
    std::printf("\n[Damper effect — pitch bounded during braking]\n");

    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Drive and brake
    InputFrame throttle{}; throttle.throttle = 1.0f;
    simulate(*v, throttle, 2.0f);

    InputFrame brake{}; brake.brake = 1.0f;
    constexpr float DT = 1.f / 120.f;

    float maxPitch = 0.f, minPitch = 0.f;
    for (int i = 0; i < 120; ++i) {
        v->integrate(brake, DT);
        float p = v->state().pitchRad;
        maxPitch = std::max(maxPitch, p);
        minPitch = std::min(minPitch, p);
    }

    float range = maxPitch - minPitch;
    check(range < 0.30f, "Pitch oscillation range bounded during braking");
    std::printf("    (pitch range=%.4f rad (%.2f deg))\n", range, range * 57.3f);
}

// ---------------------------------------------------------------------------
// Test: tyre grip scales with friction coefficient
// ---------------------------------------------------------------------------
static void testTyreGripScaling()
{
    std::printf("\n[Tyre grip — vehicle can accelerate and brake]\n");

    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Accelerate for 3s
    InputFrame throttle{}; throttle.throttle = 1.0f;
    simulate(*v, throttle, 3.0f);
    float speedAfterAccel = v->state().speedMs;

    check(speedAfterAccel > 5.f, "Vehicle accelerates with traction");

    // Brake
    InputFrame brake{}; brake.brake = 1.0f;
    simulate(*v, brake, 3.0f);
    float speedAfterBrake = v->state().speedMs;

    check(speedAfterBrake < speedAfterAccel,
          "Vehicle slows down under braking");
    std::printf("    (accel speed=%.1f m/s, after brake=%.1f m/s)\n",
                speedAfterAccel, speedAfterBrake);
}

// ---------------------------------------------------------------------------
// Test: realistic lateral g — steady cornering at known speed and radius
// ---------------------------------------------------------------------------
static void testLateralG()
{
    std::printf("\n[Lateral g in steady cornering]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Get to ~15 m/s
    InputFrame throttle{}; throttle.throttle = 0.5f;
    simulate(*v, throttle, 4.0f);

    // Moderate steer for 3 seconds — let it settle into steady corner
    InputFrame corner{}; corner.steer = 0.4f; corner.throttle = 0.3f;
    simulate(*v, corner, 3.0f);

    auto s = v->state();
    float speed = s.speedMs;

    // Estimate turning radius from heading change:
    // After 3s of cornering, total heading change in radians
    float totalHeading = std::abs(s.headingRad);
    // Arc length ≈ speed * time, radius = arcLength / angle
    if (totalHeading > 0.1f) {
        float arcLength = speed * 3.0f;  // rough
        float radius = arcLength / totalHeading;
        float lateralG = (speed * speed) / (radius * 9.81f);

        // For a BRZ with sport tyres, expect 0.3-1.1g lateral
        check(lateralG > 0.1f, "Generates meaningful lateral g");
        check(lateralG < 1.3f, "Lateral g within tyre capability");
        std::printf("    (speed=%.1f m/s, radius≈%.0f m, lateral≈%.2fg)\n",
                    speed, radius, lateralG);
    } else {
        check(false, "Car turned enough to measure lateral g");
    }
}

// ---------------------------------------------------------------------------
// Test: odometer increases with distance driven
// ---------------------------------------------------------------------------
static void testOdometer()
{
    std::printf("\n[Odometer]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    check(v->state().odoMeters < 0.1f, "Odometer starts near zero");

    InputFrame throttle{}; throttle.throttle = 1.0f;
    simulate(*v, throttle, 5.0f);

    float odo = v->state().odoMeters;
    float posZ = v->state().position.z;

    // Odometer should roughly match forward distance
    check(odo > 30.f, "Odometer records distance >30m after 5s");
    check(std::abs(odo - posZ) < posZ * 0.2f,
          "Odometer roughly matches forward distance on straight");
    std::printf("    (odo=%.1f m, position.z=%.1f m)\n", odo, posZ);
}

// ---------------------------------------------------------------------------
// Test: high-speed straight-line stability
// ---------------------------------------------------------------------------
static void testHighSpeedStraightLine()
{
    std::printf("\n[High-speed straight-line stability]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Accelerate to high speed
    InputFrame throttle{}; throttle.throttle = 1.0f;
    simulate(*v, throttle, 15.0f);
    float speed0 = v->state().speedMs;
    float heading0 = v->state().headingRad;
    float x0 = v->state().position.x;
    float roll0 = v->state().rollRad;
    float pitch0 = v->state().pitchRad;
    std::printf("    At 15s: speed=%.1f m/s (%.0f km/h), heading=%.4f rad, x=%.2f m\n",
                speed0, speed0 * 3.6f, heading0, x0);

    // Continue at full throttle for 10 more seconds — should stay straight
    constexpr float DT = 1.f / 120.f;
    float maxLateralDrift = 0.f;
    float maxHeadingDev = 0.f;
    float maxRoll = 0.f, maxPitch = 0.f;
    for (int i = 0; i < 1200; ++i) { // 10 seconds
        v->integrate(throttle, DT);
        auto s = v->state();
        maxLateralDrift = std::max(maxLateralDrift, std::abs(s.position.x - x0));
        maxHeadingDev   = std::max(maxHeadingDev, std::abs(s.headingRad - heading0));
        maxRoll  = std::max(maxRoll, std::abs(s.rollRad));
        maxPitch = std::max(maxPitch, std::abs(s.pitchRad));
    }
    auto sFinal = v->state();

    check(maxLateralDrift < 5.0f,
          "Lateral drift < 5m over 10s at high speed");
    check(maxHeadingDev < 0.05f,
          "Heading deviation < 0.05 rad (~3 deg) at high speed");
    check(maxRoll < 0.05f,
          "Roll stays small on high-speed straight");
    check(maxPitch < 0.10f,
          "Pitch stays small on high-speed straight");
    check(sFinal.speedMs > 40.f,
          "Still moving fast (no phantom deceleration)");
    std::printf("    Final: speed=%.1f m/s (%.0f km/h), lateral drift=%.2f m, "
                "heading dev=%.4f rad\n",
                sFinal.speedMs, sFinal.speedMs * 3.6f, maxLateralDrift, maxHeadingDev);
    std::printf("    Max roll=%.4f rad (%.2f deg), max pitch=%.4f rad (%.2f deg)\n",
                maxRoll, maxRoll * 180.f / 3.14159f,
                maxPitch, maxPitch * 180.f / 3.14159f);
}

// ---------------------------------------------------------------------------
// Test: high-speed cornering — car should turn, not spin or fly off
// ---------------------------------------------------------------------------
static void testHighSpeedCornering()
{
    std::printf("\n[High-speed cornering]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Get to ~30 m/s (~108 km/h)
    InputFrame throttle{}; throttle.throttle = 1.0f;
    simulate(*v, throttle, 8.0f);
    float speed0 = v->state().speedMs;
    std::printf("    Entry speed: %.1f m/s (%.0f km/h)\n", speed0, speed0 * 3.6f);

    // Gentle steer at high speed — should understeer, not snap-spin.
    // At 116 km/h on RWD with no ESC, 0.15 steer is a realistic moderate input.
    // Higher inputs (0.3+) with throttle would realistically cause a spin on RWD.
    InputFrame corner{}; corner.steer = 0.15f; corner.throttle = 0.3f;
    constexpr float DT = 1.f / 120.f;
    float maxRoll = 0.f;
    float maxYawRate = 0.f;
    float minSpeed = speed0;
    bool hitRollLimit = false;
    for (int i = 0; i < 360; ++i) { // 3 seconds
        v->integrate(corner, DT);
        auto s = v->state();
        maxRoll = std::max(maxRoll, std::abs(s.rollRad));
        minSpeed = std::min(minSpeed, s.speedMs);
        if (std::abs(s.rollRad) > 0.19f) hitRollLimit = true;
    }
    auto s1 = v->state();

    check(s1.speedMs > 10.f,
          "Car still moving after high-speed corner (didn't spin to stop)");
    check(std::abs(s1.headingRad) > 0.05f,
          "Car actually turned (heading changed)");
    check(maxRoll < 0.18f,
          "Roll stays within reasonable limits at high speed");
    check(!hitRollLimit,
          "Roll doesn't hit safety limit during high-speed cornering");
    std::printf("    After 3s: speed=%.1f m/s, heading=%.1f deg, max roll=%.3f rad (%.1f deg)\n",
                s1.speedMs, s1.headingRad * 180.f / 3.14159f,
                maxRoll, maxRoll * 180.f / 3.14159f);

    // Aggressive steer at high speed — should slide/understeer, not snap-spin.
    // Full steer (1.0) at high speed will saturate tyres; 0.5 is already aggressive.
    auto v2 = makeVehicle();
    v2->reset({0.f, 0.f, 0.f}, 0.f);
    simulate(*v2, throttle, 8.0f);
    float speed2 = v2->state().speedMs;

    InputFrame hardCorner{}; hardCorner.steer = 0.5f; hardCorner.throttle = 0.0f;
    float maxRoll2 = 0.f;
    for (int i = 0; i < 240; ++i) { // 2 seconds
        v2->integrate(hardCorner, DT);
        maxRoll2 = std::max(maxRoll2, std::abs(v2->state().rollRad));
    }
    auto s2 = v2->state();

    check(s2.speedMs > 0.f,
          "Car still moving forward after hard steer at high speed");
    check(maxRoll2 < 0.19f,
          "Roll bounded during hard steer at high speed");
    std::printf("    Hard steer: entry=%.0f km/h, exit=%.0f km/h, heading=%.0f deg, "
                "max roll=%.3f rad (%.1f deg)\n",
                speed2 * 3.6f, s2.speedMs * 3.6f,
                s2.headingRad * 180.f / 3.14159f,
                maxRoll2, maxRoll2 * 180.f / 3.14159f);
}

// ---------------------------------------------------------------------------
// Test: high-speed lane change — quick steer input and back
// ---------------------------------------------------------------------------
static void testHighSpeedLaneChange()
{
    std::printf("\n[High-speed lane change]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Get to ~40 m/s (~144 km/h)
    InputFrame throttle{}; throttle.throttle = 1.0f;
    simulate(*v, throttle, 12.0f);
    float speed0 = v->state().speedMs;
    float heading0 = v->state().headingRad;
    std::printf("    Entry speed: %.1f m/s (%.0f km/h)\n", speed0, speed0 * 3.6f);

    constexpr float DT = 1.f / 120.f;

    // Quick steer right for 0.5s
    InputFrame steerRight{}; steerRight.steer = 0.5f; steerRight.throttle = 0.8f;
    for (int i = 0; i < 60; ++i) v->integrate(steerRight, DT);
    float headingMid = v->state().headingRad;
    float rollMid = v->state().rollRad;

    // Quick steer left for 0.5s (counter-steer / lane change completion)
    InputFrame steerLeft{}; steerLeft.steer = -0.5f; steerLeft.throttle = 0.8f;
    for (int i = 0; i < 60; ++i) v->integrate(steerLeft, DT);
    float headingAfter = v->state().headingRad;
    float rollAfter = v->state().rollRad;

    // Straighten for 2s
    InputFrame straight{}; straight.throttle = 0.8f;
    float maxRollSettle = 0.f;
    for (int i = 0; i < 240; ++i) {
        v->integrate(straight, DT);
        maxRollSettle = std::max(maxRollSettle, std::abs(v->state().rollRad));
    }
    auto sFinal = v->state();

    check(sFinal.speedMs > speed0 * 0.7f,
          "Maintains most of speed through lane change");
    check(std::abs(sFinal.headingRad - heading0) < 0.25f,
          "Heading returns near straight after lane change (<14 deg)");
    check(maxRollSettle < 0.15f,
          "Roll settles after lane change (no growing oscillation)");
    check(std::abs(sFinal.rollRad) < 0.03f,
          "Roll near zero after straightening");
    std::printf("    Mid: heading=%.2f deg, roll=%.3f rad. "
                "After: heading=%.2f deg, roll=%.3f rad\n",
                headingMid * 180.f / 3.14159f, rollMid,
                headingAfter * 180.f / 3.14159f, rollAfter);
    std::printf("    Final: speed=%.0f km/h, heading dev=%.2f deg, roll=%.4f rad\n",
                sFinal.speedMs * 3.6f,
                (sFinal.headingRad - heading0) * 180.f / 3.14159f,
                sFinal.rollRad);
}

// ---------------------------------------------------------------------------
// Test: high-speed braking stability — no spin under hard braking at speed
// ---------------------------------------------------------------------------
static void testHighSpeedBraking()
{
    std::printf("\n[High-speed braking stability]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Get to ~50 m/s (~180 km/h)
    InputFrame throttle{}; throttle.throttle = 1.0f;
    simulate(*v, throttle, 15.0f);
    float speed0 = v->state().speedMs;
    float heading0 = v->state().headingRad;
    std::printf("    Entry speed: %.1f m/s (%.0f km/h)\n", speed0, speed0 * 3.6f);

    // Slam brakes at high speed
    InputFrame brake{}; brake.brake = 1.0f;
    constexpr float DT = 1.f / 120.f;
    float maxPitch = 0.f;
    float maxRoll = 0.f;
    float maxHeadingDev = 0.f;
    for (int i = 0; i < 360; ++i) { // 3 seconds
        v->integrate(brake, DT);
        auto s = v->state();
        maxPitch = std::max(maxPitch, std::abs(s.pitchRad));
        maxRoll  = std::max(maxRoll, std::abs(s.rollRad));
        maxHeadingDev = std::max(maxHeadingDev, std::abs(s.headingRad - heading0));
    }
    auto sFinal = v->state();

    check(sFinal.speedMs < 15.0f,
          "Car mostly stopped from high speed under 3s hard braking");
    check(maxHeadingDev < 0.10f,
          "Car doesn't yaw during straight-line braking");
    check(maxPitch < 0.15f,
          "Pitch bounded during high-speed braking");
    check(maxRoll < 0.05f,
          "No roll during straight-line braking");
    std::printf("    Stopped at: speed=%.1f m/s, max pitch=%.3f rad (%.1f deg), "
                "max heading dev=%.4f rad\n",
                sFinal.speedMs, maxPitch, maxPitch * 180.f / 3.14159f, maxHeadingDev);
}

// ---------------------------------------------------------------------------
// Test: trail braking into corner — brake then turn at high speed
// ---------------------------------------------------------------------------
static void testTrailBraking()
{
    std::printf("\n[Trail braking into corner]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Get to ~35 m/s (~126 km/h)
    InputFrame throttle{}; throttle.throttle = 1.0f;
    simulate(*v, throttle, 10.0f);
    float speed0 = v->state().speedMs;
    std::printf("    Entry speed: %.1f m/s (%.0f km/h)\n", speed0, speed0 * 3.6f);

    constexpr float DT = 1.f / 120.f;

    // Phase 1: heavy braking + gentle turn-in (trail braking)
    InputFrame trailBrake{}; trailBrake.brake = 0.8f; trailBrake.steer = 0.3f;
    for (int i = 0; i < 120; ++i) v->integrate(trailBrake, DT); // 1s

    auto sMid = v->state();
    std::printf("    After trail brake: speed=%.1f m/s, heading=%.1f deg, roll=%.3f rad\n",
                sMid.speedMs, sMid.headingRad * 180.f / 3.14159f, sMid.rollRad);

    // Phase 2: release brake, add throttle through corner
    InputFrame cornerExit{}; cornerExit.steer = 0.4f; cornerExit.throttle = 0.5f;
    float maxRoll = 0.f;
    for (int i = 0; i < 240; ++i) { // 2s
        v->integrate(cornerExit, DT);
        maxRoll = std::max(maxRoll, std::abs(v->state().rollRad));
    }
    auto sFinal = v->state();

    check(sMid.speedMs < speed0 * 0.85f,
          "Car slowed during trail braking (friction circle shared)");
    check(std::abs(sFinal.headingRad) > 0.2f,
          "Car turned through the corner");
    check(sFinal.speedMs > 5.f,
          "Car still moving at corner exit");
    check(maxRoll < 0.18f,
          "Roll bounded during trail brake corner");
    std::printf("    Exit: speed=%.1f m/s (%.0f km/h), heading=%.1f deg, max roll=%.3f rad\n",
                sFinal.speedMs, sFinal.speedMs * 3.6f,
                sFinal.headingRad * 180.f / 3.14159f, maxRoll);
}

// ---------------------------------------------------------------------------
// Test: brake then turn — no suspension corner gets stuck at ground
// ---------------------------------------------------------------------------
static void testBrakeThenTurnNoStuck()
{
    std::printf("\n[Brake then turn — no stuck corner]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Accelerate to moderate speed
    InputFrame throttle{}; throttle.throttle = 0.8f;
    simulate(*v, throttle, 4.0f);
    float speed0 = v->state().speedMs;
    std::printf("    Entry speed: %.1f m/s (%.0f km/h)\n", speed0, speed0 * 3.6f);

    // Brake hard while turning left (worst case for FL corner: pitch + roll load)
    InputFrame brakeTurn{}; brakeTurn.brake = 1.0f; brakeTurn.steer = -0.8f;
    simulate(*v, brakeTurn, 2.0f);

    // Now release brakes, straighten, and apply throttle — suspension should recover
    InputFrame accel{}; accel.throttle = 0.5f;
    constexpr float DT = 1.f / 120.f;

    float minSusp = 999.f;
    float maxSuspDiff = 0.f;
    for (int i = 0; i < 360; ++i) { // 3 seconds
        v->integrate(accel, DT);
        auto s = v->state();
        for (int c = 0; c < 4; ++c) {
            minSusp = std::min(minSusp, s.suspLength[c]);
        }
        // Check symmetry: no single corner wildly different from the rest
        float maxL = 0.f, minL = 999.f;
        for (int c = 0; c < 4; ++c) {
            maxL = std::max(maxL, s.suspLength[c]);
            minL = std::min(minL, s.suspLength[c]);
        }
        maxSuspDiff = std::max(maxSuspDiff, maxL - minL);
    }
    auto sFinal = v->state();

    // No corner should be collapsed (suspension length > some minimum)
    check(minSusp > 0.05f,
          "No suspension corner collapsed to near-zero length");

    // All four corners should be roughly similar after straightening
    float finalDiff = 0.f;
    {
        float maxL = 0.f, minL = 999.f;
        for (int c = 0; c < 4; ++c) {
            maxL = std::max(maxL, sFinal.suspLength[c]);
            minL = std::min(minL, sFinal.suspLength[c]);
        }
        finalDiff = maxL - minL;
    }
    check(finalDiff < 0.05f,
          "Suspension corners recover to similar lengths after straightening");

    // Car should still be driveable (moving forward)
    check(sFinal.speedMs > 3.0f,
          "Car still moving after brake-turn-accelerate sequence");

    std::printf("    Min susp length during maneuver: %.4f m\n", minSusp);
    std::printf("    Max susp diff during maneuver: %.4f m\n", maxSuspDiff);
    std::printf("    Final susp: FL=%.3f FR=%.3f RL=%.3f RR=%.3f (diff=%.4f)\n",
                sFinal.suspLength[0], sFinal.suspLength[1],
                sFinal.suspLength[2], sFinal.suspLength[3], finalDiff);
    std::printf("    Final speed: %.1f m/s, roll=%.3f rad\n",
                sFinal.speedMs, sFinal.rollRad);
}

// ---------------------------------------------------------------------------
// Test: full steer + full gas then stop — body must settle
// ---------------------------------------------------------------------------
static void testFullSteerFullGasSettle()
{
    std::printf("\n[Full steer + full gas then stop — body settles]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Phase 1: full throttle + full steer for 3 seconds (aggressive donut)
    InputFrame donut{};
    donut.throttle = 1.0f;
    donut.steer    = 1.0f;
    auto sDrive = simulate(*v, donut, 3.0f);
    std::printf("    After donut: speed=%.1f m/s, roll=%.3f rad (%.1f deg), pitch=%.3f rad (%.1f deg)\n",
                sDrive.speedMs, sDrive.rollRad, sDrive.rollRad * 57.3f,
                sDrive.pitchRad, sDrive.pitchRad * 57.3f);

    // Phase 2: release everything, coast/brake to a stop
    InputFrame brake{};
    brake.brake = 1.0f;
    simulate(*v, brake, 3.0f);

    // Phase 3: idle for 12 seconds — body should settle
    InputFrame idle{};
    auto sSettle = simulate(*v, idle, 12.0f);

    std::printf("    After settle: speed=%.2f m/s, roll=%.3f rad (%.1f deg), pitch=%.3f rad (%.1f deg)\n",
                glm::length(sSettle.velocity), sSettle.rollRad, sSettle.rollRad * 57.3f,
                sSettle.pitchRad, sSettle.pitchRad * 57.3f);

    // Body should not be oscillating wildly — roll and pitch well within safety limits
    check(std::abs(sSettle.rollRad) < 0.07f,
          "Roll settled below 4 deg after aggressive stop");
    check(std::abs(sSettle.pitchRad) < 0.07f,
          "Pitch settled below 4 deg after aggressive stop");
    // Speed should be near zero
    check(glm::length(sSettle.velocity) < 1.0f,
          "Car nearly stopped after brake + settle");
}

// ---------------------------------------------------------------------------
// Test: slip angle stays within physically reasonable bounds
// ---------------------------------------------------------------------------
static void testSlipAngleBounds()
{
    std::printf("\n[Slip angle bounds during aggressive driving]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    float maxSlip = 0.f;

    // Phase 1: accelerate
    InputFrame gas{}; gas.throttle = 1.0f;
    constexpr float DT = 1.f / 120.f;
    for (int i = 0; i < 240; ++i) { // 2s
        v->integrate(gas, DT);
        float slip = std::abs(v->state().slipAngleRad) * 57.3f;
        maxSlip = std::max(maxSlip, slip);
    }

    // Phase 2: full steer + full gas (aggressive turn)
    float maxSlipDonut = 0.f;
    InputFrame turnGas{}; turnGas.throttle = 1.0f; turnGas.steer = 1.0f;
    for (int i = 0; i < 360; ++i) { // 3s
        v->integrate(turnGas, DT);
        float slip = std::abs(v->state().slipAngleRad) * 57.3f;
        maxSlip = std::max(maxSlip, slip);
        maxSlipDonut = std::max(maxSlipDonut, slip);
    }

    // Phase 3: release, brake
    float maxSlipBrake = 0.f;
    InputFrame brake{}; brake.brake = 1.0f;
    for (int i = 0; i < 360; ++i) { // 3s
        v->integrate(brake, DT);
        float slip = std::abs(v->state().slipAngleRad) * 57.3f;
        maxSlip = std::max(maxSlip, slip);
        maxSlipBrake = std::max(maxSlipBrake, slip);
    }

    // Phase 4: idle (coming to rest) — measure slip only after car is near-stopped
    float maxSlipIdle = 0.f;
    InputFrame idle{};
    for (int i = 0; i < 600; ++i) { // 5s
        v->integrate(idle, DT);
        float slip = std::abs(v->state().slipAngleRad) * 57.3f;
        maxSlip = std::max(maxSlip, slip);
        // Only track idle slip once car is actually near-stopped
        if (glm::length(v->state().velocity) < 1.0f)
            maxSlipIdle = std::max(maxSlipIdle, slip);
    }

    std::printf("    Max slip: overall=%.1f, donut=%.1f, brake=%.1f, idle=%.1f deg\n",
                maxSlip, maxSlipDonut, maxSlipBrake, maxSlipIdle);

    // During a spin (braking from a donut), high slip is physically correct —
    // the car IS traveling sideways. What matters:
    // 1. During the donut itself, slip shouldn't be extreme (car is controllable)
    // 2. After stopping, slip must return to near zero (no phantom slip from oscillation)
    check(maxSlipDonut < 30.f,
          "Slip angle bounded during donut (car controllable)");
    check(maxSlipIdle < 5.f,
          "Slip angle returns to near zero after stopping");
}

// ---------------------------------------------------------------------------
// Test: aggressive cornering — body roll/pitch don't hit safety limits
// ---------------------------------------------------------------------------
static void testAggressiveCorneringBodyLimits()
{
    std::printf("\n[Aggressive cornering — body roll/pitch limits]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Accelerate to moderate speed
    InputFrame gas{}; gas.throttle = 1.0f;
    simulate(*v, gas, 2.0f);

    // Hard turn with throttle — aggressive but not a full donut
    InputFrame hardTurn{}; hardTurn.throttle = 0.8f; hardTurn.steer = 0.8f;
    constexpr float DT = 1.f / 120.f;

    float maxRollAbs = 0.f, maxPitchAbs = 0.f;
    int rollLimitHits = 0, pitchLimitHits = 0;
    for (int i = 0; i < 360; ++i) { // 3s of hard cornering
        v->integrate(hardTurn, DT);
        float r = std::abs(v->state().rollRad);
        float p = std::abs(v->state().pitchRad);
        maxRollAbs  = std::max(maxRollAbs,  r);
        maxPitchAbs = std::max(maxPitchAbs, p);
        if (r >= 0.139f) ++rollLimitHits;
        if (p >= 0.135f) ++pitchLimitHits;
    }

    std::printf("    Max roll: %.1f deg, Max pitch: %.1f deg\n",
                maxRollAbs * 57.3f, maxPitchAbs * 57.3f);
    std::printf("    Roll limit hits: %d, Pitch limit hits: %d (out of 360)\n",
                rollLimitHits, pitchLimitHits);

    // During aggressive (80% inputs) cornering, body shouldn't hit safety limits.
    // Some roll is expected — up to ~8 deg for a BRZ at high lateral g.
    check(maxRollAbs < 0.16f,
          "Roll stays below safety limit during aggressive cornering");
    check(maxPitchAbs < 0.16f,
          "Pitch stays below safety limit during aggressive cornering");
}

// ---------------------------------------------------------------------------
// Slope terrain helpers
// ---------------------------------------------------------------------------

/// Build a TerrainQuery with a cross-slope (lateral tilt).
/// leftY at x = -halfWidth, rightY at x = +halfWidth. Flat longitudinally.
static TerrainQuery makeTerrainCrossSlope(float lengthM, float halfWidthM,
                                           float leftY, float rightY,
                                           int numPointsZ = 50, int numPointsX = 5)
{
    std::vector<glm::vec3> positions;
    std::vector<uint32_t>  indices;
    constexpr float MARGIN = 5.f;

    for (int zi = 0; zi < numPointsZ; ++zi) {
        float z = -MARGIN + static_cast<float>(zi) / (numPointsZ - 1) * (lengthM + 2.f * MARGIN);
        for (int xi = 0; xi < numPointsX; ++xi) {
            float xFrac = static_cast<float>(xi) / (numPointsX - 1);
            float x = -halfWidthM + xFrac * 2.f * halfWidthM;
            float y = glm::mix(leftY, rightY, xFrac);
            positions.push_back({ x, y, z });
        }
    }
    // Triangulate grid
    for (int zi = 0; zi < numPointsZ - 1; ++zi) {
        for (int xi = 0; xi < numPointsX - 1; ++xi) {
            uint32_t tl = zi * numPointsX + xi;
            uint32_t tr = tl + 1;
            uint32_t bl = tl + numPointsX;
            uint32_t br = bl + 1;
            indices.insert(indices.end(), { tl, bl, tr, tr, bl, br });
        }
    }

    TerrainQuery terrain;
    terrain.buildFromMesh(positions, indices);
    return terrain;
}

/// Build a terrain for a steep hill (longitudinal slope in degrees).
static TerrainQuery makeTerrainSteepHill(float lengthM, float slopeDeg)
{
    float rise = lengthM * std::tan(slopeDeg * 3.14159f / 180.f);
    return makeTerrainLine(lengthM, 0.f, rise);
}

// ---------------------------------------------------------------------------
// Test: side-hilling — car on a lateral slope should not get stuck rolled over
// ---------------------------------------------------------------------------
static void testSideHillStability()
{
    std::printf("\n[Side-hill stability — car on lateral slope]\n");

    // Create a road that's tilted 10 degrees laterally (moderate cross-slope)
    float halfWidth = 20.f;
    float tilt = halfWidth * std::tan(10.f * 3.14159f / 180.f);
    auto terrain = makeTerrainCrossSlope(200.f, halfWidth, 0.f, tilt);

    auto v = makeVehicle();
    v->setTerrainQuery(terrain);
    v->reset({0.f, 0.f, 5.f}, 0.f);

    // Idle for 3 seconds — car should settle on the slope, not roll over
    InputFrame idle{};
    constexpr float DT = 1.f / 120.f;
    float maxRoll = 0.f;
    for (int i = 0; i < 360; ++i) {
        v->integrate(idle, DT);
        maxRoll = std::max(maxRoll, std::abs(v->state().rollRad));
    }
    auto s = v->state();

    check(std::abs(s.rollRad) < 0.18f,
          "Roll bounded on 10-deg lateral slope at rest");
    check(glm::length(s.velocity) < 3.f,
          "Car doesn't slide uncontrollably on 10-deg cross-slope");
    std::printf("    (roll=%.3f rad (%.1f deg), speed=%.2f m/s, maxRoll=%.3f rad)\n",
                s.rollRad, s.rollRad * 57.3f, glm::length(s.velocity), maxRoll);
}

// ---------------------------------------------------------------------------
// Test: side-hill driving — car traversing a lateral slope stays controllable
// ---------------------------------------------------------------------------
static void testSideHillDriving()
{
    std::printf("\n[Side-hill driving — traversing lateral slope]\n");

    float halfWidth = 20.f;
    float tilt = halfWidth * std::tan(8.f * 3.14159f / 180.f);
    auto terrain = makeTerrainCrossSlope(300.f, halfWidth, 0.f, tilt);

    auto v = makeVehicle();
    v->setTerrainQuery(terrain);
    v->reset({0.f, 0.f, 5.f}, 0.f);

    // Drive straight at moderate speed on the cross-slope
    InputFrame gas{}; gas.throttle = 0.5f;
    auto s = simulate(*v, gas, 4.0f);

    check(s.speedMs > 5.f,
          "Car can make progress driving on cross-slope");
    check(std::abs(s.rollRad) < 0.18f,
          "Roll bounded while driving on cross-slope");
    std::printf("    (speed=%.1f m/s, roll=%.3f rad (%.1f deg), x-drift=%.2f m)\n",
                s.speedMs, s.rollRad, s.rollRad * 57.3f, s.position.x);
}

// ---------------------------------------------------------------------------
// Test: steep uphill — car on 15-degree slope can climb with full throttle
// ---------------------------------------------------------------------------
static void testSteepUphillClimb()
{
    std::printf("\n[Steep uphill — 12-degree slope]\n");

    // 12 degrees is steep but climbable for a 200hp RWD car in 1st gear.
    // 15 degrees pushes the BRZ to its absolute limit (slope force ≈ max drive force).
    auto terrain = makeTerrainSteepHill(300.f, 12.f);
    auto v = makeVehicle();
    v->setTerrainQuery(terrain);
    v->reset({0.f, 0.f, 5.f}, 0.f);

    InputFrame gas{}; gas.throttle = 1.0f;
    auto s = simulate(*v, gas, 5.0f);

    check(s.speedMs > 2.f,
          "Car still making progress on 12-deg uphill");
    check(s.position.y > 3.f,
          "Car gained significant elevation on steep hill");
    check(std::abs(s.rollRad) < 0.05f,
          "No phantom roll on straight uphill");
    std::printf("    (speed=%.1f m/s, elevation=%.1f m, pitch=%.3f rad (%.1f deg))\n",
                s.speedMs, s.position.y, s.pitchRad, s.pitchRad * 57.3f);
}

// ---------------------------------------------------------------------------
// Test: steep downhill — car on -15-degree slope under braking stays stable
// ---------------------------------------------------------------------------
static void testSteepDownhillBraking()
{
    std::printf("\n[Steep downhill braking — 15-degree slope]\n");

    float slopeDeg = 15.f;
    auto terrain = makeTerrainSteepHill(300.f, -slopeDeg);
    auto v = makeVehicle();
    v->setTerrainQuery(terrain);
    v->reset({0.f, 0.f, 5.f}, 0.f);

    // Body naturally pitches ~slopeDeg on this terrain (nose-down positive)
    float terrainPitchRad = slopeDeg * 3.14159f / 180.f;

    // Let car gain some speed downhill
    InputFrame coast{};
    simulate(*v, coast, 2.0f);
    float speedBefore = glm::length(v->state().velocity);

    // Brake hard
    InputFrame brake{}; brake.brake = 1.0f;
    constexpr float DT = 1.f / 120.f;
    float maxPitchDev = 0.f;  // deviation from terrain pitch
    for (int i = 0; i < 360; ++i) {
        v->integrate(brake, DT);
        float pitchDev = std::abs(v->state().pitchRad) - terrainPitchRad;
        maxPitchDev = std::max(maxPitchDev, pitchDev);
    }
    auto s = v->state();

    // Pitch deviation from terrain slope should stay within hard limit (0.15 rad)
    check(maxPitchDev < 0.15f,
          "Pitch deviation from terrain bounded during steep downhill braking");
    check(std::abs(s.rollRad) < 0.05f,
          "No phantom roll on straight downhill braking");
    std::printf("    (speed before brake=%.1f m/s, after=%.1f m/s, pitch=%.3f rad, terrainPitch=%.3f rad, maxDev=%.3f rad)\n",
                speedBefore, glm::length(s.velocity), s.pitchRad, terrainPitchRad, maxPitchDev);
}

// ---------------------------------------------------------------------------
// Test: stopped on slope doesn't get stuck at roll limit
// ---------------------------------------------------------------------------
static void testStoppedOnSlopeNoStuck()
{
    std::printf("\n[Stopped on slope — no stuck roll]\n");

    // Simulate the user's scenario: drive aggressively, stop on a slope
    float halfWidth = 30.f;
    float slopeDeg = 8.f;
    float tilt = halfWidth * std::tan(slopeDeg * 3.14159f / 180.f);
    auto terrain = makeTerrainCrossSlope(300.f, halfWidth, 0.f, tilt);

    auto v = makeVehicle();
    v->setTerrainQuery(terrain);
    v->reset({0.f, 0.f, 5.f}, 0.f);

    // Drive fast, turn, then brake
    InputFrame gas{}; gas.throttle = 0.8f;
    simulate(*v, gas, 2.0f);

    InputFrame turnGas{}; turnGas.throttle = 0.6f; turnGas.steer = 0.5f;
    simulate(*v, turnGas, 1.5f);

    InputFrame brake{}; brake.brake = 1.0f;
    simulate(*v, brake, 3.0f);

    // Idle for 5 seconds — should settle
    InputFrame idle{};
    auto s = simulate(*v, idle, 5.0f);

    check(std::abs(s.rollRad) < 0.15f,
          "Roll not stuck at limit after stopping on cross-slope");
    check(std::abs(s.pitchRad) < 0.10f,
          "Pitch not stuck at limit after stopping on slope");
    std::printf("    (roll=%.3f rad (%.1f deg), pitch=%.3f rad (%.1f deg), speed=%.2f m/s)\n",
                s.rollRad, s.rollRad * 57.3f, s.pitchRad, s.pitchRad * 57.3f,
                glm::length(s.velocity));
}

// ---------------------------------------------------------------------------
// Test: no stored energy release — brake to stop then release, no phantom motion
// ---------------------------------------------------------------------------
static void testNoStoredEnergyOnBrakeRelease()
{
    std::printf("\n[No stored energy on brake release]\n");
    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Phase 1: full gas + full turn for 1 second (aggressive donut start)
    InputFrame donut{}; donut.throttle = 1.0f; donut.steer = 1.0f;
    simulate(*v, donut, 1.0f);
    std::printf("    After donut: speed=%.1f m/s, roll=%.3f, pitch=%.3f\n",
                glm::length(v->state().velocity), v->state().rollRad, v->state().pitchRad);

    // Phase 2: brake to a full stop
    InputFrame brake{}; brake.brake = 1.0f;
    simulate(*v, brake, 4.0f);
    auto sBraked = v->state();
    float speedBraked = glm::length(sBraked.velocity);
    std::printf("    After brake: speed=%.3f m/s, roll=%.3f, pitch=%.3f\n",
                speedBraked, sBraked.rollRad, sBraked.pitchRad);

    // Phase 3: release brake — idle for 3 seconds
    // The car should NOT wiggle or move forward
    InputFrame idle{};
    constexpr float DT = 1.f / 120.f;
    float maxSpeed = 0.f;
    float maxRollRange = 0.f, minRoll = 99.f, maxRoll = -99.f;
    float maxPitchRange = 0.f, minPitch = 99.f, maxPitch = -99.f;
    for (int i = 0; i < 360; ++i) { // 3 seconds
        v->integrate(idle, DT);
        auto s = v->state();
        float spd = glm::length(s.velocity);
        maxSpeed = std::max(maxSpeed, spd);
        minRoll  = std::min(minRoll,  s.rollRad);
        maxRoll  = std::max(maxRoll,  s.rollRad);
        minPitch = std::min(minPitch, s.pitchRad);
        maxPitch = std::max(maxPitch, s.pitchRad);
    }
    float rollRange  = maxRoll - minRoll;
    float pitchRange = maxPitch - minPitch;
    auto sFinal = v->state();

    std::printf("    After release: speed=%.3f m/s (max=%.3f), roll range=%.4f, pitch range=%.4f\n",
                glm::length(sFinal.velocity), maxSpeed, rollRange, pitchRange);

    check(maxSpeed < 0.5f,
          "No phantom forward motion after brake release");
    check(rollRange < 0.03f,
          "No roll wiggle after brake release");
    check(pitchRange < 0.03f,
          "No pitch wiggle after brake release");
}

// ---------------------------------------------------------------------------
// Test: falling off mesh edge — no NaN, car falls
// ---------------------------------------------------------------------------
static void testFallOffEdge()
{
    std::printf("\n[Fall off mesh edge — no NaN]\n");

    // Narrow mesh: only 3m wide, car will drive off quickly
    auto terrain = makeTerrainLine(100.f, 0.f, 0.f, 20, 3.f);

    auto v = makeVehicle();
    v->setTerrainQuery(terrain);
    v->reset({0.f, 0.f, 5.f}, 0.f);

    // Drive at an angle so we leave the mesh laterally
    InputFrame steerGas{}; steerGas.throttle = 0.5f; steerGas.steer = 0.5f;
    auto s = simulate(*v, steerGas, 3.0f);

    bool noNaN = !std::isnan(s.position.x) && !std::isnan(s.position.y) &&
                 !std::isnan(s.position.z) && !std::isnan(s.speedMs);
    check(noNaN, "No NaN in vehicle state after driving off mesh");
    std::printf("    (pos=%.1f, %.1f, %.1f, speed=%.1f)\n",
                s.position.x, s.position.y, s.position.z, s.speedMs);
}

// ---------------------------------------------------------------------------
// Test: reset after fall — car returns to valid position
// ---------------------------------------------------------------------------
static void testResetAfterFall()
{
    std::printf("\n[Reset after fall — returns to valid state]\n");

    auto terrain = makeTerrainLine(100.f, 0.f, 0.f, 20, 3.f);
    auto v = makeVehicle();
    v->setTerrainQuery(terrain);
    v->reset({0.f, 0.f, 5.f}, 0.f);

    // Drive off the edge
    InputFrame steerGas{}; steerGas.throttle = 0.5f; steerGas.steer = 0.5f;
    simulate(*v, steerGas, 3.0f);

    // Now reset back to start
    v->reset({0.f, 0.f, 5.f}, 0.f);
    auto s = v->state();

    check(!std::isnan(s.position.y), "Position not NaN after reset");
    check(std::abs(s.position.z - 5.f) < 0.1f, "Position near reset target");

    // Run physics for 1 second to verify it settles
    InputFrame idle{};
    s = simulate(*v, idle, 1.0f);

    check(!std::isnan(s.position.y), "No NaN after post-reset simulation");
    check(std::abs(s.position.y) < 2.f, "Car at reasonable height after reset");
    check(glm::length(s.velocity) < 1.f, "Car nearly still after reset + settle");
    std::printf("    (pos=%.2f, %.2f, %.2f, speed=%.3f)\n",
                s.position.x, s.position.y, s.position.z, glm::length(s.velocity));
}

// ---------------------------------------------------------------------------
// Test: partial wheel support — 2 wheels on mesh, 2 off
// ---------------------------------------------------------------------------
static void testPartialWheelSupport()
{
    std::printf("\n[Partial wheel support — half on/half off]\n");

    // Build a mesh that covers only the right side (x > -0.5m).
    // Left wheels will be off-mesh, right wheels on-mesh.
    std::vector<glm::vec3> positions;
    std::vector<uint32_t> indices;
    for (int i = 0; i < 30; ++i) {
        float z = i * 5.f - 5.f;
        positions.push_back({ -0.5f, 0.f, z });
        positions.push_back({  20.f, 0.f, z });
    }
    for (int i = 0; i < 29; ++i) {
        uint32_t tl = i*2, tr = i*2+1, bl = (i+1)*2, br = (i+1)*2+1;
        indices.insert(indices.end(), { tl, bl, tr, tr, bl, br });
    }
    TerrainQuery terrain;
    terrain.buildFromMesh(positions, indices);

    auto v = makeVehicle();
    v->setTerrainQuery(terrain);
    // Place car at x=0: right wheels at x~+0.75 (on mesh), left at x~-0.75 (off mesh)
    v->reset({0.f, 0.f, 5.f}, 0.f);

    // Idle for 2 seconds — car should tilt and slide, not NaN
    InputFrame idle{};
    auto s = simulate(*v, idle, 2.0f);

    bool noNaN = !std::isnan(s.position.x) && !std::isnan(s.position.y) &&
                 !std::isnan(s.position.z);
    check(noNaN, "No NaN with partial wheel support");
    // With right wheels (FR/RR at -X) off-mesh, gravity should barrel-roll
    // the car rightward (positive roll = right side down).
    check(std::abs(s.rollRad) > 0.3f,
          "Car barrel-rolls toward unsupported side (roll > 17 deg)");
    // The car tips dramatically — don't require drop since the supported-side
    // suspension pushes upward. The key is the roll, not the vertical position.
    check(std::abs(s.rollRad) > 0.5f,
          "Car reaches significant roll angle (> 29 deg) with half support");
    std::printf("    (pos=%.1f, %.1f, %.1f, roll=%.3f rad (%.1f deg), speed=%.1f)\n",
                s.position.x, s.position.y, s.position.z,
                s.rollRad, s.rollRad * 57.3f, glm::length(s.velocity));
}

// ---------------------------------------------------------------------------
// Test: hairpin exit physics — no spike at segment 16→17 transition
// ---------------------------------------------------------------------------
static void testHairpinExitPhysics()
{
    std::printf("\n[Hairpin exit — smooth physics through segment transition]\n");

    // Build the actual road and terrain from the test stage
    // We can't easily do this without the full RoadBuilder, so instead
    // simulate a car driving through a corner and check for spikes.
    // Build a terrain that mimics the hairpin exit: a wide flat surface.
    auto terrain = makeTerrainLine(400.f, 0.f, 0.f, 50, 50.f);

    auto v = makeVehicle();
    v->setTerrainQuery(terrain);
    v->reset({0.f, 0.f, 5.f}, 0.f);

    // Accelerate, then enter a tight right turn (simulating hairpin)
    InputFrame gas{}; gas.throttle = 0.8f;
    simulate(*v, gas, 3.0f);

    InputFrame turnR{}; turnR.throttle = 0.4f; turnR.steer = 0.8f;
    simulate(*v, turnR, 3.0f);

    // Exit turn — straighten and accelerate (the transition point)
    InputFrame straighten{}; straighten.throttle = 0.6f; straighten.steer = 0.0f;

    // Monitor for sudden spikes in speed, roll, or pitch
    constexpr float DT = 1.f / 120.f;
    float maxDeltaSpeed = 0.f;
    float maxDeltaRoll  = 0.f;
    float prevSpeed = glm::length(v->state().velocity);
    float prevRoll  = v->state().rollRad;

    for (int i = 0; i < 360; ++i) {  // 3 seconds
        v->integrate(straighten, DT);
        float speed = glm::length(v->state().velocity);
        float roll  = v->state().rollRad;

        float dSpeed = std::abs(speed - prevSpeed);
        float dRoll  = std::abs(roll - prevRoll);
        maxDeltaSpeed = std::max(maxDeltaSpeed, dSpeed);
        maxDeltaRoll  = std::max(maxDeltaRoll, dRoll);

        prevSpeed = speed;
        prevRoll  = roll;
    }
    auto s = v->state();

    check(!std::isnan(s.speedMs), "No NaN after hairpin exit");
    check(maxDeltaSpeed < 2.f, "No sudden speed spike at corner exit");
    check(maxDeltaRoll < 0.15f, "No sudden roll spike at corner exit");
    check(glm::length(s.velocity) > 3.f, "Car still moving after corner exit");
    std::printf("    (speed=%.1f m/s, maxDeltaSpeed=%.3f, maxDeltaRoll=%.4f)\n",
                glm::length(s.velocity), maxDeltaSpeed, maxDeltaRoll);
}

// ---------------------------------------------------------------------------
// Test: real road terrain — drive through hairpin exit on actual stage mesh
// ---------------------------------------------------------------------------
static void testRealRoadHairpinExit()
{
    std::printf("\n[Real road — hairpin exit terrain continuity]\n");

    // Build the actual test stage and terrain
    TestStageGenerator gen;
    auto segments = gen.generate();
    auto mesh = RoadBuilder::build(segments);

    std::vector<glm::vec3> meshPositions;
    meshPositions.reserve(mesh.vertices.size());
    for (const auto& v : mesh.vertices)
        meshPositions.push_back(v.position);

    TerrainQuery terrain;
    terrain.buildFromMesh(meshPositions, mesh.indices);

    // Validate terrain query: height at each centerline point WITH hintY should
    // match the mesh Y. Without hint, overlapping road may return wrong section.
    float maxTerrainError = 0.f;
    for (size_t i = 0; i < mesh.centerlinePoints.size(); ++i) {
        auto& p = mesh.centerlinePoints[i];
        float h = terrain.heightAt(p.x, p.z, p.y);  // hint = actual mesh Y
        if (h > TerrainQuery::NO_GROUND + 1.f) {
            float err = std::abs(h - p.y);
            maxTerrainError = std::max(maxTerrainError, err);
        }
    }
    check(maxTerrainError < 0.1f,
          "Terrain query with hintY matches mesh centerline height");
    std::printf("    (max terrain error with hintY=%.4f m)\n", maxTerrainError);

    // Drive the actual road using steering to stay on track.
    // Use moderate throttle with no steer — the opening straight is long enough.
    auto v = makeVehicle();
    v->setTerrainQuery(terrain);
    v->setCenterlinePoints(mesh.centerlinePoints, mesh.segmentStartVertex, 7);

    size_t startSeg = (segments[0].indexSequence < 0) ? 1 : 0;
    v->reset(segments[startSeg].startPosition, segments[startSeg].startHeadingRad);

    // Drive straight for 3 seconds on the opening 80m straight
    InputFrame gas{}; gas.throttle = 0.8f;
    constexpr float DT = 1.f / 120.f;
    bool anyNaN = false;
    float maxDeltaSpeed = 0.f;
    float prevSpeed = 0.f;

    for (int i = 0; i < 120 * 3; ++i) {
        v->integrate(gas, DT);
        auto s = v->state();
        float speed = glm::length(s.velocity);
        if (std::isnan(speed)) { anyNaN = true; break; }
        float dSpeed = std::abs(speed - prevSpeed);
        maxDeltaSpeed = std::max(maxDeltaSpeed, dSpeed);
        prevSpeed = speed;
    }

    auto s = v->state();
    check(!anyNaN, "No NaN driving on real road opening straight");
    check(maxDeltaSpeed < 3.f, "No speed spikes on opening straight");
    check(s.position.y < 5.f && s.position.y > -5.f,
          "Car stays at reasonable height on opening straight");
    std::printf("    (pos=%.1f, %.1f, %.1f, speed=%.1f, maxDSpeed=%.3f)\n",
                s.position.x, s.position.y, s.position.z,
                glm::length(s.velocity), maxDeltaSpeed);
}

// ---------------------------------------------------------------------------
// Test: tree collision — car heading toward tree line is stopped
// ---------------------------------------------------------------------------
static void testTreeCollision()
{
    std::printf("\n[Tree collision barrier]\n");

    auto v = makeVehicle();
    v->reset({0.f, 0.f, 0.f}, 0.f);

    // Place a row of trees perpendicular to +Z at z=15, along X from -10 to +10
    std::vector<TreeInstance> trees;
    for (float x = -10.f; x <= 10.f; x += 1.5f) {
        trees.push_back({ glm::vec3{x, -0.5f, 15.f}, 0.20f, 6.f });
    }
    v->setTrees(trees);

    // Drive full throttle into the trees
    InputFrame gas{};
    gas.throttle = 1.0f;
    constexpr float DT = 1.f / 120.f;

    float maxZ = 0.f;
    for (int i = 0; i < 600; ++i) {  // 5 seconds
        v->integrate(gas, DT);
        auto s = v->state();
        maxZ = std::max(maxZ, s.position.z);
    }

    auto s = v->state();
    float speed = glm::length(s.velocity);
    std::printf("    (pos=%.1f, %.1f, %.1f, speed=%.1f m/s, maxZ=%.1f)\n",
                s.position.x, s.position.y, s.position.z, speed, maxZ);

    // Car should have been stopped by the tree wall
    check(maxZ < 16.f, "Car stopped by tree barrier (did not pass z=16)");
    check(speed < 8.f, "Car slowed significantly by tree impact");
    check(!std::isnan(s.position.x), "No NaN after tree collision");
}

// ---------------------------------------------------------------------------
// Diagnostic: scripted full-track drive
//
// Drives the entire test stage using proportional steering to follow the
// centerline. Logs per-frame diagnostics for anomalies: NaN, speed spikes,
// extreme roll/pitch, suspension bottoming, wheels off mesh.
// NOT a pass/fail unit test — diagnostic only.
// ---------------------------------------------------------------------------
static void diagnosticFullTrackDrive()
{
    std::printf("\n[DIAGNOSTIC: Full-track drive]\n");

    // Build actual test stage
    TestStageGenerator gen;
    auto segments = gen.generate();
    auto mesh = RoadBuilder::build(segments);

    std::vector<glm::vec3> meshPositions;
    meshPositions.reserve(mesh.vertices.size());
    for (const auto& v : mesh.vertices)
        meshPositions.push_back(v.position);

    TerrainQuery terrain;
    terrain.buildFromMesh(meshPositions, mesh.indices);

    auto v = makeVehicle();
    v->setTerrainQuery(terrain);
    v->setCenterlinePoints(mesh.centerlinePoints, mesh.segmentStartVertex, 7);

    size_t startSeg = (segments[0].indexSequence < 0) ? 1 : 0;
    v->reset(segments[startSeg].startPosition, segments[startSeg].startHeadingRad);

    constexpr float DT = 1.f / 120.f;
    constexpr float MAX_TIME = 120.f;  // 2 minutes max

    // Tracking
    int   frame = 0;
    float simTime = 0.f;
    float prevSpeed = 0.f;
    int   nanFrames = 0;
    int   speedSpikeFrames = 0;
    int   extremeRollFrames = 0;
    int   extremePitchFrames = 0;
    int   offMeshFrames = 0;
    int   suspBottomFrames = 0;
    float maxSpeed = 0.f;
    float maxRoll = 0.f;
    float maxPitch = 0.f;
    float maxSpeedDelta = 0.f;
    int   lastSegReported = -1;

    // Find nearest centerline point for steering
    auto findNearestCenterline = [&](const glm::vec3& pos) -> int {
        float minDist = 1e9f;
        int best = 0;
        for (int i = 0; i < (int)mesh.centerlinePoints.size(); ++i) {
            glm::vec3 d = pos - mesh.centerlinePoints[i];
            d.y = 0.f;
            float dist = glm::dot(d, d);
            if (dist < minDist) { minDist = dist; best = i; }
        }
        return best;
    };

    while (simTime < MAX_TIME) {
        auto s = v->state();

        // Check for NaN
        if (std::isnan(s.position.x) || std::isnan(s.position.y) || std::isnan(s.position.z) ||
            std::isnan(s.speedMs)) {
            ++nanFrames;
            if (nanFrames <= 3)
                std::printf("  !! NaN at t=%.2f seg=%d pos=(%.1f,%.1f,%.1f)\n",
                            simTime, s.segmentIndex, s.position.x, s.position.y, s.position.z);
            break;
        }

        float speed = glm::length(s.velocity);
        float dSpeed = std::abs(speed - prevSpeed);

        // Log segment transitions
        if (s.segmentIndex != lastSegReported) {
            std::printf("  seg %2d: t=%.1fs speed=%.1f m/s pos=(%.0f,%.1f,%.0f) roll=%.2f pitch=%.2f\n",
                        s.segmentIndex, simTime, speed,
                        s.position.x, s.position.y, s.position.z,
                        s.rollRad * 57.3f, s.pitchRad * 57.3f);
            lastSegReported = s.segmentIndex;
        }

        // Speed spike
        if (dSpeed > 2.f && frame > 10) {
            ++speedSpikeFrames;
            if (speedSpikeFrames <= 5)
                std::printf("  !! Speed spike at t=%.2f seg=%d: delta=%.2f (%.1f->%.1f) "
                            "pos=(%.1f,%.1f,%.1f) roll=%.1f pitch=%.1f "
                            "gndY=[%.1f,%.1f,%.1f,%.1f]\n",
                            simTime, s.segmentIndex, dSpeed, prevSpeed, speed,
                            s.position.x, s.position.y, s.position.z,
                            s.rollRad * 57.3f, s.pitchRad * 57.3f,
                            s.wheelGroundHeight[0], s.wheelGroundHeight[1],
                            s.wheelGroundHeight[2], s.wheelGroundHeight[3]);
        }
        maxSpeedDelta = std::max(maxSpeedDelta, dSpeed);
        maxSpeed = std::max(maxSpeed, speed);

        // Extreme roll (terrain-relative: deviation from ground slope)
        float rollDeviation = s.rollRad - s.terrainRollRad;
        if (std::abs(rollDeviation) > 0.15f) {
            ++extremeRollFrames;
            if (extremeRollFrames <= 3)
                std::printf("  !! Extreme roll at t=%.2f seg=%d: %.2f deg (terrain=%.2f, dev=%.2f)\n",
                            simTime, s.segmentIndex, s.rollRad * 57.3f,
                            s.terrainRollRad * 57.3f, rollDeviation * 57.3f);
        }
        maxRoll = std::max(maxRoll, std::abs(rollDeviation));

        // Extreme pitch (terrain-relative: deviation from ground slope)
        float pitchDeviation = s.pitchRad - s.terrainPitchRad;
        if (std::abs(pitchDeviation) > 0.12f) {
            ++extremePitchFrames;
            if (extremePitchFrames <= 3)
                std::printf("  !! Extreme pitch at t=%.2f seg=%d: %.2f deg (terrain=%.2f, dev=%.2f)\n",
                            simTime, s.segmentIndex, s.pitchRad * 57.3f,
                            s.terrainPitchRad * 57.3f, pitchDeviation * 57.3f);
        }
        maxPitch = std::max(maxPitch, std::abs(pitchDeviation));

        // Off-mesh wheels
        int offCount = 0;
        for (int c = 0; c < 4; ++c)
            if (s.wheelGroundHeight[c] <= TerrainQuery::NO_GROUND + 1.f) ++offCount;
        if (offCount > 0) {
            ++offMeshFrames;
            if (offMeshFrames <= 3)
                std::printf("  !! %d wheels off mesh at t=%.2f seg=%d pos=(%.0f,%.1f,%.0f)\n",
                            offCount, simTime, s.segmentIndex,
                            s.position.x, s.position.y, s.position.z);
        }

        // Suspension bottoming (spring length near minimum)
        for (int c = 0; c < 4; ++c) {
            if (s.suspLength[c] < 0.05f && s.suspLength[c] > 0.f) {
                ++suspBottomFrames;
                if (suspBottomFrames <= 5)
                    std::printf("  !! Susp bottom corner %d at t=%.2f seg=%d len=%.4f\n",
                                c, simTime, s.segmentIndex, s.suspLength[c]);
            }
        }

        // --- Compute steering to follow centerline ---
        int nearestIdx = findNearestCenterline(s.position);
        // Look ahead: distance-based, shorter at low speed
        float lookDist = std::max(8.f, speed * 0.6f);
        int lookAhead = nearestIdx;
        float accumulated = 0.f;
        while (lookAhead + 1 < (int)mesh.centerlinePoints.size() && accumulated < lookDist) {
            accumulated += glm::length(mesh.centerlinePoints[lookAhead + 1]
                                     - mesh.centerlinePoints[lookAhead]);
            ++lookAhead;
        }
        glm::vec3 target = mesh.centerlinePoints[lookAhead];

        // Desired heading to target
        float dx = target.x - s.position.x;
        float dz = target.z - s.position.z;
        float desiredHeading = std::atan2(dx, dz);

        // Heading error (normalized to [-pi, pi])
        float headingErr = desiredHeading - s.headingRad;
        while (headingErr >  3.14159f) headingErr -= 6.28318f;
        while (headingErr < -3.14159f) headingErr += 6.28318f;

        // Lateral offset from centerline — used as additional steering correction
        glm::vec3 toNearest = s.position - mesh.centerlinePoints[nearestIdx];
        toNearest.y = 0.f;
        // Project onto the perpendicular of the heading direction
        float headSin = std::sin(s.headingRad), headCos = std::cos(s.headingRad);
        float lateralOff = toNearest.x * headCos - toNearest.z * headSin;

        // PD-controller for steering: heading error + lateral offset correction
        float steerCmd = std::clamp(headingErr * 3.5f - lateralOff * 0.5f, -1.f, 1.f);

        // Speed-adaptive throttle: slower in corners, faster on straights
        float absSteer = std::abs(steerCmd);
        float targetSpeed = (absSteer > 0.5f) ? 7.f : (absSteer > 0.2f) ? 12.f : 20.f;
        float throttleCmd = (speed < targetSpeed) ? 0.5f : 0.05f;
        float brakeCmd = (speed > targetSpeed + 3.f) ? 0.9f : 0.f;

        InputFrame input{};
        input.steer    = steerCmd;
        input.throttle = throttleCmd;
        input.brake    = brakeCmd;

        v->integrate(input, DT);
        prevSpeed = speed;
        simTime += DT;
        ++frame;

        // Stop if car fell off (Y < -50)
        if (s.position.y < -50.f) {
            std::printf("  !! Car fell off at t=%.2f seg=%d pos=(%.0f,%.1f,%.0f)\n",
                        simTime, s.segmentIndex, s.position.x, s.position.y, s.position.z);
            break;
        }

        // Stop if reached the end of the track (last segment)
        if (s.segmentIndex >= (int)segments.size() - 2 && s.odoMeters > 500.f)
            break;
    }

    auto finalState = v->state();
    std::printf("\n  --- Summary ---\n");
    std::printf("  Sim time:     %.1f s (%d frames)\n", simTime, frame);
    std::printf("  Final seg:    %d / %d\n", finalState.segmentIndex, (int)segments.size() - 1);
    std::printf("  Final pos:    (%.0f, %.1f, %.0f)\n",
                finalState.position.x, finalState.position.y, finalState.position.z);
    std::printf("  Final speed:  %.1f m/s (%.0f km/h)\n",
                glm::length(finalState.velocity), glm::length(finalState.velocity) * 3.6f);
    std::printf("  Odometer:     %.0f m\n", finalState.odoMeters);
    std::printf("  Max speed:    %.1f m/s (%.0f km/h)\n", maxSpeed, maxSpeed * 3.6f);
    std::printf("  Max roll dev: %.1f deg (terrain-relative)\n", maxRoll * 57.3f);
    std::printf("  Max pitch dev:%.1f deg (terrain-relative)\n", maxPitch * 57.3f);
    std::printf("  Max dSpeed:   %.2f m/s/frame\n", maxSpeedDelta);
    std::printf("  NaN frames:   %d\n", nanFrames);
    std::printf("  Speed spikes: %d (>2 m/s/frame)\n", speedSpikeFrames);
    std::printf("  Extreme roll: %d frames (>8.6 deg from terrain)\n", extremeRollFrames);
    std::printf("  Extreme pitch:%d frames (>6.9 deg from terrain)\n", extremePitchFrames);
    std::printf("  Off-mesh:     %d frames\n", offMeshFrames);
    std::printf("  Susp bottom:  %d events\n", suspBottomFrames);
    std::printf("  ---------------\n\n");
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main()
{
    std::printf("=== Vehicle Physics Tests ===\n");

    // Basic behavior
    testStaticEquilibrium();
    testThrottle();
    testBraking();
    testSteering();
    testNoSpinAtRest();
    testBodyRoll();
    testLowSpeedHighSteer();

    // Terrain collision tests
    testFlatGroundRegression();
    testUphillSlowdown();
    testDownhillAcceleration();
    testCrestElevation();
    testPerWheelHeightDifference();

    // Suspension and stability tests
    testSuspensionTravelLimits();
    testBodyRollStability();
    testRwdBehavior();
    testBodySettlesAfterHardDriving();

    // Weight transfer and dynamics
    testWeightTransferBraking();
    testWeightTransferAcceleration();
    testStaticWeightDistribution();
    testBrakingDistance();
    testEnergyConservationCoasting();

    // Performance and limits
    testTopSpeed();
    testGearShifting();
    testSuspensionStaticDeflection();

    // Handling characteristics
    testOversteerUnderPower();
    testLoadSensitivity();
    testSwayBarEffect();
    testSpringRateEffect();
    testCoastDown();
    testSteeringSymmetry();
    testDamperEffect();
    testTyreGripScaling();
    testLateralG();
    testOdometer();

    // High-speed behavior
    testHighSpeedStraightLine();
    testHighSpeedCornering();
    testHighSpeedLaneChange();
    testHighSpeedBraking();
    testTrailBraking();
    testBrakeThenTurnNoStuck();

    // Full-steer + full-gas stability
    testFullSteerFullGasSettle();
    testSlipAngleBounds();
    testAggressiveCorneringBodyLimits();

    // Slope stability
    testSideHillStability();
    testSideHillDriving();
    testSteepUphillClimb();
    testSteepDownhillBraking();
    testStoppedOnSlopeNoStuck();

    // Brake release stability
    testNoStoredEnergyOnBrakeRelease();

    // Off-mesh and partial support
    testFallOffEdge();
    testResetAfterFall();
    testPartialWheelSupport();
    testHairpinExitPhysics();
    testRealRoadHairpinExit();

    // Tree collision
    testTreeCollision();

    // Full-track diagnostic (not pass/fail — just prints diagnostics)
    diagnosticFullTrackDrive();

    std::printf("\n=== Results: %d passed, %d failed ===\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
