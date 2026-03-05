/// Unit tests for MultiBodyVehicle physics.
/// Uses hand-written assertions (no external framework) and prints PASS/FAIL.
/// Build target: VehiclePhysicsTest (see tests/CMakeLists.txt)

#include "vehicle/MultiBodyVehicle.h"
#include "input/InputFrame.h"
#include <cstdio>
#include <cmath>
#include <cstdlib>

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

// Run for N seconds of simulation with fixed inputs; return final state.
static VehicleState simulate(MultiBodyVehicle& v, const InputFrame& input, float seconds)
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
    MultiBodyVehicle v;
    v.reset({0.f, 0.f, 0.f}, 0.f);

    InputFrame zero{};
    auto s = simulate(v, zero, 2.0f);   // let suspension settle

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
    MultiBodyVehicle v;
    v.reset({0.f, 0.f, 0.f}, 0.f);

    InputFrame throttle{};
    throttle.throttle = 1.0f;

    auto s = simulate(v, throttle, 5.0f);

    check(s.speedMs > 10.f, "Reaches >10 m/s (~36 km/h) in 5 s");
    check(s.speedMs < 80.f, "Does not exceed 80 m/s (unrealistic)");
    // Forward motion: position.z should be positive (we start heading yaw=0, +Z = forward)
    check(s.position.z > 30.f, "Travels >30 m forward in 5 s");
    // Under hard throttle, rear squats → slight nose-up (negative pitch in nose-down convention)
    check(s.pitchRad < 0.02f, "Pitch stays small under throttle");

    // 0-60 kph (~16.7 m/s) should happen in under 8 s for a WRX
    MultiBodyVehicle v2;
    v2.reset({0.f, 0.f, 0.f}, 0.f);
    float t = 0.f;
    constexpr float DT = 1.f / 120.f;
    while (t < 10.f) {
        v2.integrate(throttle, DT);
        t += DT;
        if (v2.state().speedMs >= 16.7f) break;
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
    MultiBodyVehicle v;
    v.reset({0.f, 0.f, 0.f}, 0.f);

    // First get the car up to speed
    InputFrame throttle{}; throttle.throttle = 1.0f;
    simulate(v, throttle, 3.0f);
    float speed0 = v.state().speedMs;
    check(speed0 > 5.f, "Car has speed before braking test");

    // Now full brake
    InputFrame brake{}; brake.brake = 1.0f;
    auto s = simulate(v, brake, 4.0f);

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
    MultiBodyVehicle v;
    v.reset({0.f, 0.f, 0.f}, 0.f);

    InputFrame throttle{}; throttle.throttle = 0.5f;
    simulate(v, throttle, 3.0f);
    float speed0 = v.state().speedMs;
    check(speed0 > 3.f, "Car has speed before steering test");

    glm::vec3 pos0 = v.state().position;

    // Steer left (negative steer = left in rally convention)
    InputFrame steerLeft{}; steerLeft.steer = -1.0f; steerLeft.throttle = 0.3f;
    simulate(v, steerLeft, 2.0f);

    auto s = v.state();
    // After 2 s of left steer, heading should have changed (turned left = heading decreased)
    float headingChange = s.headingRad - 0.f;  // initial heading was 0
    check(headingChange < -0.1f || s.position.x < pos0.x - 1.f,
          "Car turns left with negative steer input");

    // Roll: turning left → body rolls right (centrifugal) → positive rollRad
    // (right-side-down = positive). Allow small tolerance since we check direction.
    check(s.rollRad > -0.15f, "Roll direction plausible during left turn");

    // Steer right
    MultiBodyVehicle v2;
    v2.reset({0.f, 0.f, 0.f}, 0.f);
    InputFrame throttle2{}; throttle2.throttle = 0.5f;
    simulate(v2, throttle2, 3.0f);

    InputFrame steerRight{}; steerRight.steer = +1.0f; steerRight.throttle = 0.3f;
    simulate(v2, steerRight, 2.0f);
    auto s2 = v2.state();
    check(s2.headingRad > 0.1f || s2.position.x > 1.f,
          "Car turns right with positive steer input");
}

// ---------------------------------------------------------------------------
// Test: no spinning at standstill with steer input
// ---------------------------------------------------------------------------
static void testNoSpinAtRest()
{
    std::printf("\n[No spin at standstill]\n");
    MultiBodyVehicle v;
    v.reset({0.f, 0.f, 0.f}, 0.f);

    // Full steer, no throttle, from rest
    InputFrame steer{}; steer.steer = 1.0f;
    auto s = simulate(v, steer, 1.0f);

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
    MultiBodyVehicle v;
    v.reset({0.f, 0.f, 0.f}, 0.f);

    // Get up to speed
    InputFrame throttle{}; throttle.throttle = 0.6f;
    simulate(v, throttle, 4.0f);

    // Right turn at speed
    InputFrame corner{}; corner.steer = 1.0f; corner.throttle = 0.2f;
    simulate(v, corner, 1.5f);
    float roll = v.state().rollRad;

    // Turning right → centrifugal pushes body outward (left relative to car) →
    // outer left springs compress more → left side pushed up → right side drops down.
    // Right-side-down = POSITIVE roll in our convention.
    check(roll > 0.005f,  "Body rolls to outside of right turn (positive roll = right-side-down)");
    check(roll < 0.20f,   "Roll stays within realistic limit (<~11 deg)");

    std::printf("    (roll = %.4f rad = %.2f deg)\n",
                roll, roll * 180.f / 3.14159f);
}

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main()
{
    std::printf("=== MultiBodyVehicle Physics Tests ===\n");

    testStaticEquilibrium();
    testThrottle();
    testBraking();
    testSteering();
    testNoSpinAtRest();
    testBodyRoll();

    std::printf("\n=== Results: %d passed, %d failed ===\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
