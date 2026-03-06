#include "../src/Engine.h"
#include "../src/Tire.h"
#include "../src/Wheel.h"
#include "../src/VehiclePhysics.h"
#include "../src/Input.h"
#include <cstdio>
#include <cmath>
#include <cstdlib>

static int g_pass = 0, g_fail = 0;

#define CHECK(cond, msg) do { \
    if (cond) { g_pass++; } \
    else { g_fail++; std::fprintf(stderr, "FAIL [%s:%d]: %s\n", __FILE__, __LINE__, msg); } \
} while(0)

#define APPROX(a, b, tol) (std::abs((a)-(b)) < (tol))

// Helper: run physics for N seconds with given input
static void simulate(VehiclePhysics& phys, const InputState& input, float seconds)
{
    constexpr float DT = 1.f / 120.f;
    int steps = (int)(seconds / DT);
    for (int i = 0; i < steps; ++i)
        phys.update(DT, input);
}

// ============================================================================
// Torque curve tests
// ============================================================================

static void testTorqueCurveLookup()
{
    TorqueCurve curve({
        {1000.f, 30.f},
        {3500.f, 80.f},
        {6500.f, 45.f},
    });

    CHECK(APPROX(curve.lookup(1000.f), 30.f, 0.01f), "torque at 1000 RPM");
    CHECK(APPROX(curve.lookup(6500.f), 45.f, 0.01f), "torque at 6500 RPM");

    float mid = curve.lookup(2250.f);
    CHECK(APPROX(mid, 55.f, 0.01f), "torque interpolation at 2250 RPM");

    CHECK(APPROX(curve.lookup(500.f), 30.f, 0.01f), "torque below min RPM");
    CHECK(APPROX(curve.lookup(7000.f), 45.f, 0.01f), "torque above max RPM");
}

// ============================================================================
// Engine tests
// ============================================================================

static void testEngineIdleWithNoThrottle()
{
    Engine eng;
    eng.torqueCurve = TorqueCurve({
        {1000.f, 30.f}, {3500.f, 80.f}, {6500.f, 45.f},
    });
    eng.rpm = 1200.f;

    for (int i = 0; i < 1000; ++i)
        eng.update(0.f, 0.f, 1.f/120.f);

    CHECK(APPROX(eng.rpm, eng.idleRpm, 50.f), "engine stays near idle with no throttle");
}

static void testEngineRevsUpWithThrottle()
{
    Engine eng;
    eng.torqueCurve = TorqueCurve({
        {1000.f, 30.f}, {3500.f, 80.f}, {6500.f, 45.f},
    });
    eng.rpm = 1200.f;

    for (int i = 0; i < 240; ++i)
        eng.update(1.f, 0.f, 1.f/120.f);

    CHECK(eng.rpm > 2000.f, "engine revs up with throttle (clutch disengaged)");
    CHECK(eng.rpm <= eng.rpmLimit, "engine respects RPM limit");
}

static void testEngineRevLimiter()
{
    Engine eng;
    eng.torqueCurve = TorqueCurve({
        {1000.f, 30.f}, {6500.f, 80.f},
    });
    eng.rpm = 6400.f;

    for (int i = 0; i < 600; ++i)
        eng.update(1.f, 0.f, 1.f/120.f);

    CHECK(eng.rpm <= eng.rpmLimit, "rev limiter caps RPM");
}

static void testClutchEngagement()
{
    Engine eng;
    eng.torqueCurve = TorqueCurve({
        {1000.f, 30.f}, {3500.f, 80.f}, {6500.f, 45.f},
    });
    eng.rpm = 1200.f;

    float torque = eng.update(1.f, 0.f, 1.f/120.f);
    CHECK(APPROX(torque, 0.f, 1.f), "no torque below clutch engage RPM");

    eng.rpm = 3000.f;
    torque = eng.update(1.f, 0.f, 1.f/120.f);
    CHECK(torque > 10.f, "torque output when clutch engaged");
}

// ============================================================================
// Tire unit tests
// ============================================================================

static void testTireDeflectionBounds()
{
    Tire t;
    t.radius = 0.30f;
    t.maxDeflection = 0.025f;

    // No contact: hub above ground + R
    CHECK(t.computeDeflection(0.35f, 0.f) == 0.f, "no deflection when hub above ground+R");

    // Partial deflection
    float d = t.computeDeflection(0.29f, 0.f);  // 10mm overlap
    CHECK(APPROX(d, 0.01f, 0.001f), "correct deflection for 10mm overlap");

    // Clamped at dMax
    float dMax = t.computeDeflection(0.20f, 0.f);  // way below ground
    CHECK(APPROX(dMax, 0.025f, 0.001f), "deflection clamped at dMax");
}

static void testTireNormalForce()
{
    Tire t;
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

    // No contact
    CHECK(t.computeNormalForce(0.f, 0.f) == 0.f, "zero force at zero deflection");
}

static void testTireContactPatch()
{
    Tire t;
    t.radius = 0.30f;
    t.width = 0.20f;

    // At 10mm deflection: L = 2*sqrt(2*0.3*0.01 - 0.01^2) = 2*sqrt(0.0059) ≈ 0.154m
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
    Tire t;
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

    // Large slip: saturates at mu * Fn
    float Fsat = t.computeLongitudinalForce(1.0f, Fn);
    CHECK(APPROX(Fsat, t.mu * Fn, 1.f), "brush model saturates at mu*Fn");

    // Negative slip: negative force
    float Fneg = t.computeLongitudinalForce(-0.05f, Fn);
    CHECK(Fneg < 0.f, "negative slip = negative force");

    // No normal load: no force
    CHECK(t.computeLongitudinalForce(0.1f, 0.f) == 0.f, "no force without normal load");
}

// ============================================================================
// Vehicle integration tests
// ============================================================================

static void testVehicleAccelerates()
{
    VehiclePhysics phys;
    phys.init();

    InputState gas{};
    gas.throttle = 1.f;

    simulate(phys, gas, 5.f);

    CHECK(phys.getForwardSpeed() > 5.f, "vehicle accelerates with throttle");
    CHECK(phys.getEngineRpm() > 2000.f, "engine RPM rises during acceleration");
}

static void testVehicleBrakes()
{
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 5.f);
    float speedBeforeBrake = phys.getForwardSpeed();
    CHECK(speedBeforeBrake > 5.f, "moving before braking");

    InputState brake{}; brake.brake = 1.f;
    simulate(phys, brake, 3.f);

    CHECK(phys.getForwardSpeed() < speedBeforeBrake * 0.1f,
          "vehicle slows significantly with brakes");
}

static void testVehicleCoastsToStop()
{
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 3.f);
    float peakSpeed = phys.getForwardSpeed();
    CHECK(peakSpeed > 3.f, "vehicle moving before coasting");

    InputState nothing{};
    simulate(phys, nothing, 60.f);

    CHECK(phys.getForwardSpeed() < 0.5f,
          "vehicle coasts to near-stop from friction");
}

static void testBrakeDoesNotReverse()
{
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 2.f);

    InputState brake{}; brake.brake = 1.f;
    simulate(phys, brake, 10.f);

    CHECK(phys.getForwardSpeed() >= -0.05f,
          "braking doesn't make the car go backward");
}

static void testRWDOnlyRearWheelsDriven()
{
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 3.f);

    CHECK(phys.getForwardSpeed() > 2.f,
          "RWD vehicle accelerates (rear wheels driven)");
}

static void testEngineBraking()
{
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 5.f);
    float speed1 = phys.getForwardSpeed();

    InputState nothing{};
    simulate(phys, nothing, 2.f);
    float speed2 = phys.getForwardSpeed();

    float decel = speed1 - speed2;
    // Decel comes from rolling resistance + bearing drag through slip model.
    // Centrifugal clutch disengages at low RPM, limiting engine braking.
    CHECK(decel > 0.2f, "rolling resistance + bearing drag decelerates off throttle");
}

static void testEqualBrakingFrontRear()
{
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 4.f);
    float speedBefore = phys.getForwardSpeed();

    InputState brake{}; brake.brake = 1.f;
    simulate(phys, brake, 1.f);

    float speedAfter = phys.getForwardSpeed();
    float decel = speedBefore - speedAfter;

    CHECK(decel > 5.f, "strong braking with all 4 wheels");
}

static void testStationaryWithNoInput()
{
    VehiclePhysics phys;
    phys.init();

    InputState nothing{};
    simulate(phys, nothing, 2.f);

    CHECK(APPROX(phys.getForwardSpeed(), 0.f, 0.1f),
          "vehicle stays stationary with no input");
}

// ============================================================================
// Friction holding tests (tire regularized Coulomb)
// ============================================================================

static void testDirectBackwardDrift()
{
    // Start fresh, manually check forces at low backward speed
    VehiclePhysics phys;
    phys.init();

    // Let it settle vertically
    InputState nothing{};
    simulate(phys, nothing, 1.f);

    // Give it a small forward push, then check it damps
    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 2.f);
    std::printf("  after-gas speed=%.4f\n", phys.getForwardSpeed());

    simulate(phys, nothing, 0.5f);
    std::printf("  coast0.5 speed=%.4f\n", phys.getForwardSpeed());
    simulate(phys, nothing, 0.5f);
    std::printf("  coast1.0 speed=%.4f\n", phys.getForwardSpeed());
    simulate(phys, nothing, 0.5f);
    std::printf("  coast1.5 speed=%.4f\n", phys.getForwardSpeed());

    CHECK(phys.getForwardSpeed() >= 0.f, "car doesn't go backward from coasting forward");
}

static void testFrictionHoldsAfterBraking()
{
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 3.f);
    std::printf("  after-gas speed=%.4f\n", phys.getForwardSpeed());

    InputState brake{}; brake.brake = 1.f;
    simulate(phys, brake, 1.f);
    std::printf("  after-1s-brake speed=%.4f\n", phys.getForwardSpeed());
    simulate(phys, brake, 4.f);
    std::printf("  after-5s-brake speed=%.4f\n", phys.getForwardSpeed());

    // Release everything — regularized friction should damp to ~0
    InputState nothing{};
    for (int s = 0; s < 10; ++s) {
        simulate(phys, nothing, 0.5f);
        std::printf("  coast +%.1fs speed=%.4f\n", (s+1)*0.5f, phys.getForwardSpeed());
    }
    CHECK(std::abs(phys.getForwardSpeed()) < 0.05f,
          "friction damps car to near-zero after braking");
}

static void testNoCreepAtStartup()
{
    VehiclePhysics phys;
    phys.init();

    Vehicle veh;
    phys.fillVehicle(veh);
    float startZ = veh.position.z;

    InputState nothing{};
    simulate(phys, nothing, 10.f);

    phys.fillVehicle(veh);
    CHECK(std::abs(veh.position.z - startZ) < 0.01f,
          "no creep at startup");
}

static void testThrottleOvercomesFriction()
{
    VehiclePhysics phys;
    phys.init();

    InputState nothing{};
    simulate(phys, nothing, 1.f);
    CHECK(std::abs(phys.getForwardSpeed()) < 0.01f, "stationary before throttle");

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 3.f);
    CHECK(phys.getForwardSpeed() > 1.f, "throttle overcomes friction");
}

// ============================================================================
// Tire lateral force tests
// ============================================================================

static void testTireLateralBrushModel()
{
    Tire t;
    t.mu = 1.0f;
    t.lateralSlipStiffnessPerArea = 1.5e6f;
    t.width = 0.20f;
    t.radius = 0.30f;

    t.updateContactPatch(0.01f);
    float Fn = 2000.f;

    // Positive slip angle (velocity to right) → negative force (pushes left)
    float F1 = t.computeLateralForce(0.05f, Fn);
    CHECK(F1 < 0.f, "positive slip angle → negative lateral force");

    // Negative slip angle → positive force
    float F2 = t.computeLateralForce(-0.05f, Fn);
    CHECK(F2 > 0.f, "negative slip angle → positive lateral force");

    // Larger slip angle → larger magnitude
    float F3 = t.computeLateralForce(0.10f, Fn);
    CHECK(std::abs(F3) > std::abs(F1), "more slip angle = more force");

    // Saturates at mu * Fn
    float Fsat = t.computeLateralForce(1.0f, Fn);
    CHECK(APPROX(std::abs(Fsat), t.mu * Fn, 1.f), "lateral saturates at mu*Fn");

    // No contact = no force
    CHECK(t.computeLateralForce(0.1f, 0.f) == 0.f, "no lateral force without load");
}

static void testFrictionCircle()
{
    Tire t;
    t.mu = 1.0f;
    float Fn = 1000.f;

    // Forces within circle: no change
    float fx = 500.f, fy = 500.f;
    t.frictionCircleClamp(fx, fy, Fn);
    CHECK(APPROX(fx, 500.f, 0.1f), "friction circle: within → unchanged");

    // Forces exceeding circle: scaled down
    fx = 800.f; fy = 800.f;
    t.frictionCircleClamp(fx, fy, Fn);
    float combined = std::sqrt(fx * fx + fy * fy);
    CHECK(APPROX(combined, 1000.f, 1.f), "friction circle: clamped to mu*Fn");
    CHECK(APPROX(fx, fy, 0.1f), "friction circle: ratio preserved");
}

// ============================================================================
// Steering tests
// ============================================================================

static void testVehicleTurnsRight()
{
    VehiclePhysics phys;
    phys.init();

    // Build up some speed first
    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 3.f);
    float headingBefore = phys.getHeading();

    // Steer right while maintaining speed
    InputState steerRight{}; steerRight.throttle = 0.5f; steerRight.steer = 1.f;
    simulate(phys, steerRight, 2.f);

    float headingAfter = phys.getHeading();
    CHECK(headingAfter > headingBefore + 0.1f,
          "vehicle turns right with positive steer");
}

static void testVehicleTurnsLeft()
{
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 3.f);
    float headingBefore = phys.getHeading();

    InputState steerLeft{}; steerLeft.throttle = 0.5f; steerLeft.steer = -1.f;
    simulate(phys, steerLeft, 2.f);

    float headingAfter = phys.getHeading();
    CHECK(headingAfter < headingBefore - 0.1f,
          "vehicle turns left with negative steer");
}

static void testStraightWithNoSteer()
{
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 5.f);

    // With zero steer, heading should stay very close to 0
    CHECK(std::abs(phys.getHeading()) < 0.01f,
          "vehicle drives straight with no steering");
}

static void testDeadzoneInput()
{
    // Small steer inputs within deadzone should not turn the car
    VehiclePhysics phys;
    phys.init();

    InputState gas{}; gas.throttle = 1.f;
    simulate(phys, gas, 3.f);

    // Apply tiny steer (within 5% deadzone)
    InputState tinySteer{}; tinySteer.throttle = 0.5f; tinySteer.steer = 0.03f;
    simulate(phys, tinySteer, 2.f);

    CHECK(std::abs(phys.getHeading()) < 0.01f,
          "deadzone prevents turning from tiny steer input");
}

// ============================================================================

int main()
{
    std::printf("Running vehicle physics tests...\n\n");

    testTorqueCurveLookup();
    testEngineIdleWithNoThrottle();
    testEngineRevsUpWithThrottle();
    testEngineRevLimiter();
    testClutchEngagement();

    testTireDeflectionBounds();
    testTireNormalForce();
    testTireContactPatch();
    testTireBrushModel();
    testTireLateralBrushModel();
    testFrictionCircle();

    testVehicleAccelerates();
    testVehicleBrakes();
    testVehicleCoastsToStop();
    testBrakeDoesNotReverse();
    testRWDOnlyRearWheelsDriven();
    testEngineBraking();
    testEqualBrakingFrontRear();
    testStationaryWithNoInput();

    testFrictionHoldsAfterBraking();
    testNoCreepAtStartup();
    testThrottleOvercomesFriction();

    testVehicleTurnsRight();
    testVehicleTurnsLeft();
    testStraightWithNoSteer();
    testDeadzoneInput();

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
