#include "test_common.hpp"
#include "VehiclePhysics.hpp"
#include "Terrain.hpp"
#include <cmath>
#include <algorithm>

// ---------------------------------------------------------------------------
// Helpers — replicate the terrain interaction loop from main.cpp
// ---------------------------------------------------------------------------
static constexpr float DT = 1.f / 120.f;

struct WheelInfo {
    Subframe* subframe;
    SuspensionCorner* corner;
};

static glm::vec3 wheelLocalPos(const WheelInfo& w) {
    return w.subframe->attachmentPoint()
         + w.corner->attachmentPoint()
         + w.corner->tireOffset;
}

static void getWheels(VehiclePhysics& v, WheelInfo out[4]) {
    out[0] = { v.frontSubframe(), v.frontSubframe()->left()  };
    out[1] = { v.frontSubframe(), v.frontSubframe()->right() };
    out[2] = { v.rearSubframe(),  v.rearSubframe()->left()   };
    out[3] = { v.rearSubframe(),  v.rearSubframe()->right()  };
}

static void updateSprings(VehiclePhysics& v, const Terrain& t,
                           float prevComp[4]) {
    const auto& st = v.state();
    WheelInfo wheels[4];
    getWheels(v, wheels);

    for (int i = 0; i < 4; ++i) {
        glm::vec3 local = wheelLocalPos(wheels[i]);
        glm::vec3 world = st.position + st.bodyRotation * local;
        float groundH = t.heightAt(world.x, world.z);
        float comp = std::max(0.f, (groundH + wheels[i].corner->tire()->radius) - world.y);
        float compVel = (comp - prevComp[i]) / DT;
        prevComp[i] = comp;
        wheels[i].corner->spring()->setCompression(comp, compVel);
    }
}

static std::vector<CollisionContact> checkBodyCollisions(
    VehiclePhysics& v, const Terrain& t) {
    std::vector<CollisionContact> contacts;
    auto* body = v.body();
    if (!body) return contacts;
    const auto& st = v.state();
    for (auto& local : body->colliderCorners()) {
        glm::vec3 world = st.position + st.bodyRotation * (body->attachmentPoint() + local);
        TerrainContact tc = t.contactAt(world);
        if (tc.penetration > 0.f)
            contacts.push_back({ world, tc.normal, tc.penetration });
    }
    return contacts;
}

// Full physics step: springs -> ground normal -> update -> collisions
static void physicsStep(VehiclePhysics& v, const Terrain& t,
                         const InputState& input, float prevComp[4]) {
    updateSprings(v, t, prevComp);
    glm::vec3 gn = t.normalAt(v.state().position.x, v.state().position.z);
    v.setGroundNormal(gn);
    v.update(DT, input);
    auto contacts = checkBodyCollisions(v, t);
    if (!contacts.empty())
        v.applyCollisions(contacts, DT);
}

// Simulate for a duration with full terrain interaction
static void simulate(VehiclePhysics& v, const Terrain& t,
                      const InputState& input, float seconds, float prevComp[4]) {
    for (float elapsed = 0.f; elapsed < seconds; elapsed += DT)
        physicsStep(v, t, input, prevComp);
}

static float forwardSpeed(const VehiclePhysics& v) {
    glm::vec3 bodyVel = glm::transpose(v.state().bodyRotation) * v.state().velocity;
    return bodyVel.z;
}

// Initialize at equilibrium height on flat terrain
static void initOnTerrain(VehiclePhysics& v, const Terrain& t) {
    float groundH = t.heightAt(0.f, 0.f);
    float tireR = 0.31f;
    float mass = 1300.f;
    float springRate = 35000.f;
    float eqComp = mass * 9.81f / (4.f * springRate);
    v.mutableState().position.y = groundH + tireR - eqComp;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

static void testEquilibriumOnFlatGround() {
    std::printf("  testEquilibriumOnFlatGround\n");
    VehiclePhysics v; v.init();
    Terrain t;
    initOnTerrain(v, t);
    float prevComp[4] = {};
    InputState idle{};

    simulate(v, t, idle, 2.f, prevComp);

    float y = v.state().position.y;
    CHECK(y > 0.10f && y < 0.40f, "vehicle height reasonable");

    float speed = glm::length(v.state().velocity);
    CHECK(speed < 0.5f, "vehicle nearly stationary");
}

static void testNormalLoadAtRest() {
    std::printf("  testNormalLoadAtRest\n");
    VehiclePhysics v; v.init();
    Terrain t;
    initOnTerrain(v, t);
    float prevComp[4] = {};
    InputState idle{};

    simulate(v, t, idle, 1.f, prevComp);

    WheelInfo wheels[4];
    getWheels(v, wheels);
    for (int i = 0; i < 4; ++i) {
        float load = wheels[i].corner->tire()->tireOutput().normalLoadN;
        CHECK(load > 1000.f, "tire has substantial normal load (>1000N)");
    }
}

static void testThrottleAccelerates() {
    std::printf("  testThrottleAccelerates\n");
    VehiclePhysics v; v.init();
    Terrain t;
    initOnTerrain(v, t);
    float prevComp[4] = {};

    InputState idle{};
    simulate(v, t, idle, 0.5f, prevComp);

    InputState gas{}; gas.throttle = 1.f;
    simulate(v, t, gas, 2.f, prevComp);

    float fwd = forwardSpeed(v);
    CHECK(fwd > 5.f, "car moving forward >5 m/s after 2s WOT");
}

static void testZeroToSixty() {
    std::printf("  testZeroToSixty\n");
    VehiclePhysics v; v.init();
    Terrain t;
    initOnTerrain(v, t);
    float prevComp[4] = {};

    InputState idle{};
    simulate(v, t, idle, 0.5f, prevComp);

    InputState gas{}; gas.throttle = 1.f;
    float target = 26.8f; // 60 mph in m/s
    float elapsed = 0.f;
    bool reached = false;

    while (elapsed < 10.f) {
        physicsStep(v, t, gas, prevComp);
        elapsed += DT;
        if (forwardSpeed(v) >= target) { reached = true; break; }
    }

    CHECK(reached, "reached 60 mph within 10 seconds");
    CHECK(elapsed < 10.f, "0-60 time under 10s");
}

static void testBrakeStopsVehicle() {
    std::printf("  testBrakeStopsVehicle\n");
    VehiclePhysics v; v.init();
    Terrain t;
    initOnTerrain(v, t);
    float prevComp[4] = {};

    InputState idle{};
    simulate(v, t, idle, 0.5f, prevComp);
    InputState gas{}; gas.throttle = 1.f;
    simulate(v, t, gas, 3.f, prevComp);

    float speedBefore = forwardSpeed(v);
    CHECK(speedBefore > 10.f, "car is moving before braking");

    InputState brake{}; brake.brake = 1.f;
    simulate(v, t, brake, 5.f, prevComp);

    float speedAfter = std::abs(forwardSpeed(v));
    CHECK(speedAfter < 1.f, "car nearly stopped after 5s of braking");
}

static void testBrakeDeceleration() {
    std::printf("  testBrakeDeceleration\n");
    VehiclePhysics v; v.init();
    Terrain t;
    initOnTerrain(v, t);
    float prevComp[4] = {};

    InputState idle{};
    simulate(v, t, idle, 0.5f, prevComp);
    InputState gas{}; gas.throttle = 1.f;
    simulate(v, t, gas, 3.f, prevComp);

    float speedBefore = forwardSpeed(v);

    InputState brake{}; brake.brake = 1.f;
    simulate(v, t, brake, 1.f, prevComp);

    float speedAfter = forwardSpeed(v);
    float decel = (speedBefore - speedAfter) / 1.f;
    CHECK(decel > 4.9f, "braking deceleration > 0.5g");
}

static void testHandBrakeSlows() {
    std::printf("  testHandBrakeSlows\n");
    VehiclePhysics v; v.init();
    Terrain t;
    initOnTerrain(v, t);
    float prevComp[4] = {};

    InputState idle{};
    simulate(v, t, idle, 0.5f, prevComp);
    InputState gas{}; gas.throttle = 1.f;
    simulate(v, t, gas, 2.f, prevComp);

    float speedBefore = forwardSpeed(v);
    CHECK(speedBefore > 5.f, "moving before handbrake");

    InputState hb{}; hb.handBrake = 1.f;
    simulate(v, t, hb, 2.f, prevComp);

    float speedAfter = forwardSpeed(v);
    CHECK(speedAfter < speedBefore, "handbrake slows vehicle");
}

static void testSteeringProducesYaw() {
    std::printf("  testSteeringProducesYaw\n");
    VehiclePhysics v; v.init();
    Terrain t;
    initOnTerrain(v, t);
    float prevComp[4] = {};

    InputState idle{};
    simulate(v, t, idle, 0.5f, prevComp);

    InputState gas{}; gas.throttle = 1.f;
    simulate(v, t, gas, 2.f, prevComp);

    float headingBefore = v.state().heading;

    InputState steerLeft{}; steerLeft.throttle = 0.5f; steerLeft.steer = -1.f;
    simulate(v, t, steerLeft, 2.f, prevComp);

    float headingChange = std::abs(v.state().heading - headingBefore);
    CHECK(headingChange > 0.1f, "steering produces heading change >0.1 rad");
}

static void testSteeringProducesLateralMotion() {
    std::printf("  testSteeringProducesLateralMotion\n");
    VehiclePhysics v; v.init();
    Terrain t;
    initOnTerrain(v, t);
    float prevComp[4] = {};

    InputState idle{};
    simulate(v, t, idle, 0.5f, prevComp);

    InputState gas{}; gas.throttle = 1.f;
    simulate(v, t, gas, 2.f, prevComp);

    float xBefore = v.state().position.x;

    InputState steerRight{}; steerRight.throttle = 0.5f; steerRight.steer = 1.f;
    simulate(v, t, steerRight, 2.f, prevComp);

    float xDisplacement = std::abs(v.state().position.x - xBefore);
    CHECK(xDisplacement > 1.f, "lateral displacement > 1m after 2s of steering");
}

static void testNoSteeringAtRest() {
    std::printf("  testNoSteeringAtRest\n");
    VehiclePhysics v; v.init();
    Terrain t;
    initOnTerrain(v, t);
    float prevComp[4] = {};

    InputState idle{};
    simulate(v, t, idle, 0.5f, prevComp);

    float headingBefore = v.state().heading;

    InputState steerOnly{}; steerOnly.steer = 1.f;
    simulate(v, t, steerOnly, 1.f, prevComp);

    float headingChange = std::abs(v.state().heading - headingBefore);
    CHECK(headingChange < 0.05f, "no significant yaw without forward speed");
}

static void testDriveAndTurn() {
    std::printf("  testDriveAndTurn\n");
    VehiclePhysics v; v.init();
    Terrain t;
    initOnTerrain(v, t);
    float prevComp[4] = {};

    InputState idle{};
    simulate(v, t, idle, 0.5f, prevComp);

    InputState circle{}; circle.throttle = 0.7f; circle.steer = 0.3f;
    simulate(v, t, circle, 5.f, prevComp);

    float heading = std::abs(v.state().heading);
    CHECK(heading > 0.3f, "completed significant turn in 5 seconds");

    float speed = glm::length(v.state().velocity);
    CHECK(speed > 3.f, "still moving at >3 m/s");
}

static void testAccelBrakeStaysOnGround() {
    std::printf("  testAccelBrakeStaysOnGround\n");
    VehiclePhysics v; v.init();
    Terrain t;
    initOnTerrain(v, t);
    float prevComp[4] = {};

    InputState idle{};
    simulate(v, t, idle, 0.5f, prevComp);

    InputState gas{}; gas.throttle = 1.f;
    simulate(v, t, gas, 2.f, prevComp);

    InputState brake{}; brake.brake = 1.f;
    simulate(v, t, brake, 3.f, prevComp);

    float y = v.state().position.y;
    float groundH = t.heightAt(v.state().position.x, v.state().position.z);
    CHECK(y - groundH < 0.5f, "vehicle near ground after stop");
    CHECK(y - groundH > -0.2f, "vehicle not under ground");
}

static void testGearShiftsUp() {
    std::printf("  testGearShiftsUp\n");
    VehiclePhysics v; v.init();
    Terrain t;
    initOnTerrain(v, t);
    float prevComp[4] = {};

    InputState idle{};
    simulate(v, t, idle, 0.5f, prevComp);

    CHECK(v.drivetrain()->currentGear == 0, "starts in first gear");

    InputState gas{}; gas.throttle = 1.f;
    simulate(v, t, gas, 5.f, prevComp);

    CHECK(v.drivetrain()->currentGear > 0, "shifted up from first gear");
}

static void testEngineRpmTracksSpeed() {
    std::printf("  testEngineRpmTracksSpeed\n");
    VehiclePhysics v; v.init();
    Terrain t;
    initOnTerrain(v, t);
    float prevComp[4] = {};

    InputState idle{};
    simulate(v, t, idle, 0.5f, prevComp);
    float idleRpm = v.drivetrain()->engineRpm;

    InputState gas{}; gas.throttle = 1.f;
    simulate(v, t, gas, 3.f, prevComp);
    float drivingRpm = v.drivetrain()->engineRpm;

    CHECK(drivingRpm > idleRpm + 500.f, "RPM increased significantly under load");
}

static void testFullBrakeLockup() {
    std::printf("  testFullBrakeLockup\n");
    VehiclePhysics v; v.init();
    Terrain t;
    initOnTerrain(v, t);
    float prevComp[4] = {};

    InputState idle{};
    simulate(v, t, idle, 0.5f, prevComp);

    // Build speed
    InputState gas{}; gas.throttle = 1.f;
    simulate(v, t, gas, 3.f, prevComp);

    float speedBefore = forwardSpeed(v);
    CHECK(speedBefore > 10.f, "moving before brake lockup test");

    // Full brake — should lock wheels (tire reports sliding)
    InputState brake{}; brake.brake = 1.f;
    WheelInfo wheels[4];
    getWheels(v, wheels);

    bool anySliding = false;
    for (float elapsed = 0.f; elapsed < 1.f; elapsed += DT) {
        physicsStep(v, t, brake, prevComp);
        for (int i = 0; i < 4; ++i)
            if (wheels[i].corner->tire()->tireOutput().sliding)
                anySliding = true;
    }

    CHECK(anySliding, "full brake causes wheel lockup (sliding)");

    // With locked wheels, braking is less effective than threshold braking.
    // Verify the car still decelerates (doesn't glitch out).
    float speedAfter = forwardSpeed(v);
    CHECK(speedAfter < speedBefore, "car decelerates under locked brakes");
}

static void testHandBrakeLockup() {
    std::printf("  testHandBrakeLockup\n");
    VehiclePhysics v; v.init();
    Terrain t;
    initOnTerrain(v, t);
    float prevComp[4] = {};

    InputState idle{};
    simulate(v, t, idle, 0.5f, prevComp);

    InputState gas{}; gas.throttle = 1.f;
    simulate(v, t, gas, 3.f, prevComp);

    float speedBefore = forwardSpeed(v);
    CHECK(speedBefore > 10.f, "moving before handbrake test");

    // Handbrake only — should lock rear wheels
    InputState hb{}; hb.handBrake = 1.f;
    WheelInfo wheels[4];
    getWheels(v, wheels);

    bool rearSliding = false;
    for (float elapsed = 0.f; elapsed < 1.f; elapsed += DT) {
        physicsStep(v, t, hb, prevComp);
        // Rear tires are indices 2 and 3
        for (int i = 2; i < 4; ++i)
            if (wheels[i].corner->tire()->tireOutput().sliding)
                rearSliding = true;
    }

    CHECK(rearSliding, "handbrake locks rear wheels (sliding)");

    // Car should slow down
    float speedAfter = forwardSpeed(v);
    CHECK(speedAfter < speedBefore, "handbrake slows car");
}

// ---------------------------------------------------------------------------
// Acceleration grip tests — BRZ should not easily break rears loose
// ---------------------------------------------------------------------------

static void testHalfThrottleNoSliding() {
    std::printf("  testHalfThrottleNoSliding\n");
    VehiclePhysics v; v.init();
    Terrain t;
    initOnTerrain(v, t);
    float prevComp[4] = {};

    InputState idle{};
    simulate(v, t, idle, 0.5f, prevComp);

    // 50% throttle — BRZ should NOT break rear tires loose
    InputState gas{}; gas.throttle = 0.5f;
    WheelInfo wheels[4];
    getWheels(v, wheels);

    int slidingFrames = 0;
    for (float elapsed = 0.f; elapsed < 3.f; elapsed += DT) {
        physicsStep(v, t, gas, prevComp);
        for (int i = 2; i < 4; ++i)
            if (wheels[i].corner->tire()->tireOutput().sliding)
                slidingFrames++;
    }

    // Allow brief transient sliding (<5% of total frames) but not sustained
    int totalRearFrames = (int)(3.f / DT) * 2;
    float slidingPct = 100.f * slidingFrames / totalRearFrames;
    std::printf("    sliding: %.1f%% of frames (%d/%d)\n",
        slidingPct, slidingFrames, totalRearFrames);
    CHECK(slidingPct < 5.f,
          "50%% throttle: rear tires not sliding (< 5%% of frames)");

    float speed = forwardSpeed(v);
    CHECK(speed > 8.f, "50%% throttle: built reasonable speed");
}

static void testFullThrottleAcceleration() {
    std::printf("  testFullThrottleAcceleration\n");
    VehiclePhysics v; v.init();
    Terrain t;
    initOnTerrain(v, t);
    float prevComp[4] = {};

    InputState idle{};
    simulate(v, t, idle, 0.5f, prevComp);

    // Full throttle — track acceleration and rear slip
    InputState gas{}; gas.throttle = 1.f;
    WheelInfo wheels[4];
    getWheels(v, wheels);

    float speedAt2s = 0.f;
    int slidingFrames = 0;
    int totalFrames = 0;
    for (float elapsed = 0.f; elapsed < 4.f; elapsed += DT) {
        physicsStep(v, t, gas, prevComp);
        if (elapsed > 0.5f) {  // skip initial launch transient
            totalFrames += 2;
            for (int i = 2; i < 4; ++i)
                if (wheels[i].corner->tire()->tireOutput().sliding)
                    slidingFrames++;
        }
        if (std::abs(elapsed - 2.f) < DT)
            speedAt2s = forwardSpeed(v);
    }

    float slidingPct = 100.f * slidingFrames / std::max(totalFrames, 1);
    std::printf("    speed at 2s: %.1f m/s, sliding: %.1f%%\n",
        speedAt2s, slidingPct);

    // BRZ should accelerate well — at least 8 m/s after 2s full throttle
    CHECK(speedAt2s > 8.f, "full throttle: good acceleration (>8 m/s at 2s)");

    // Some sliding in 1st gear OK, but should not be excessive (not >30%)
    CHECK(slidingPct < 30.f,
          "full throttle: rear sliding < 30%% of frames");
}

// ---------------------------------------------------------------------------
int main() {
    std::printf("Running driving characteristic tests...\n\n");
    std::printf("Equilibrium:\n");
    testEquilibriumOnFlatGround();
    testNormalLoadAtRest();

    std::printf("Acceleration:\n");
    testThrottleAccelerates();
    testZeroToSixty();

    std::printf("Braking:\n");
    testBrakeStopsVehicle();
    testBrakeDeceleration();
    testHandBrakeSlows();

    std::printf("Steering:\n");
    testSteeringProducesYaw();
    testSteeringProducesLateralMotion();
    testNoSteeringAtRest();

    std::printf("Combined:\n");
    testDriveAndTurn();
    testAccelBrakeStaysOnGround();

    std::printf("Wheel lockup:\n");
    testFullBrakeLockup();
    testHandBrakeLockup();

    std::printf("Drivetrain:\n");
    testGearShiftsUp();
    testEngineRpmTracksSpeed();

    std::printf("Acceleration grip:\n");
    testHalfThrottleNoSliding();
    testFullThrottleAcceleration();

    return reportResults();
}
