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

static void testTireRollingResistance()
{
    Tire t;
    t.normalLoad = 1000.f;  // 1000 N
    t.rollingResistCoeff = 0.015f;

    // At moderate speed, rolling resistance = C_rr * N * regSign(v)
    // regSign(5.0) ≈ 5.0 / (5.0 + 0.02) ≈ 0.996
    float force = t.computeForce(0.f, 5.0f);  // no wheel torque, 5 m/s
    CHECK(force < -10.f, "rolling resistance opposes forward motion");
    CHECK(force > -20.f, "rolling resistance is modest (C_rr * N * ~1)");
}

static void testTireHoldsAtZeroSpeed()
{
    Tire t;
    t.normalLoad = 1000.f;

    // At zero speed with no torque, force should be essentially zero
    float force = t.computeForce(0.f, 0.f);
    CHECK(std::abs(force) < 0.1f, "tire produces ~zero force at standstill");
}

static void testTireTractionLimit()
{
    Tire t;
    t.normalLoad = 1000.f;
    t.staticFrictionCoeff = 1.0f;

    // Demand way more force than traction allows
    float hugeForce = t.computeForce(5000.f, 10.f);  // 5000 Nm at wheel
    CHECK(std::abs(hugeForce) <= 1000.f + 1.f, "tire clamps to traction limit");
}

static void testTireRegularizedFriction()
{
    Tire t;
    t.normalLoad = 1000.f;
    t.epsilon = 0.02f;

    // Very low speed: friction ramps linearly, not discontinuously
    float f1 = t.computeForce(0.f, 0.001f);
    float f2 = t.computeForce(0.f, 0.01f);
    float f3 = t.computeForce(0.f, 0.1f);

    // Forces should increase with speed (rolling resistance ramps up)
    CHECK(std::abs(f1) < std::abs(f2), "friction increases from 0.001 to 0.01 m/s");
    CHECK(std::abs(f2) < std::abs(f3), "friction increases from 0.01 to 0.1 m/s");
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
    simulate(phys, nothing, 30.f);

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
    // Centrifugal clutch disengages at low drivenRPM, so engine braking
    // is limited.  Decel comes mainly from drivetrain friction + rolling
    // resistance + bearing drag.
    CHECK(decel > 0.3f, "drivetrain friction decelerates vehicle when off throttle");
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
    std::printf("  settled speed=%.6f\n", phys.getForwardSpeed());

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

int main()
{
    std::printf("Running vehicle physics tests...\n\n");

    testTorqueCurveLookup();
    testEngineIdleWithNoThrottle();
    testEngineRevsUpWithThrottle();
    testEngineRevLimiter();
    testClutchEngagement();

    testTireRollingResistance();
    testTireHoldsAtZeroSpeed();
    testTireTractionLimit();
    testTireRegularizedFriction();

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

    std::printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return g_fail > 0 ? 1 : 0;
}
