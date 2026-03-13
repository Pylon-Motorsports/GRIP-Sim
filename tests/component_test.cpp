#include "test_common.hpp"
#include "VehiclePhysics.hpp"

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static constexpr float DT = 1.f / 120.f;

static glm::mat3 rotationFromAngles(float heading, float pitch, float roll) {
    float ch = std::cos(heading), sh = std::sin(heading);
    float cp = std::cos(pitch),   sp = std::sin(pitch);
    float cr = std::cos(roll),    sr = std::sin(roll);
    return glm::mat3(
        glm::vec3( ch*cr + sh*sp*sr, cp*sr, -sh*cr + ch*sp*sr),
        glm::vec3(-ch*sr + sh*sp*cr, cp*cr,  sh*sr + ch*sp*cr),
        glm::vec3( sh*cp,           -sp,     ch*cp)
    );
}

static void simulate(VehiclePhysics& v, const InputState& in, float seconds) {
    for (float t = 0.f; t < seconds; t += DT)
        v.update(DT, in);
}

// Default compression balances gravity: mg / (4 * springRate)
// 1300 * 9.81 / (4 * 30000) ≈ 0.106m
static constexpr float EQ_COMPRESSION = 1300.f * 9.81f / (4.f * 30000.f);

static void loadSprings(VehiclePhysics& v, float compression = EQ_COMPRESSION) {
    v.frontSubframe()->left()->spring()->setCompression(compression, 0.f);
    v.frontSubframe()->right()->spring()->setCompression(compression, 0.f);
    v.rearSubframe()->left()->spring()->setCompression(compression, 0.f);
    v.rearSubframe()->right()->spring()->setCompression(compression, 0.f);
}

// Simulate with ground contact: springs loaded, body constrained to surface.
// Without a real terrain system, this helper provides the ground constraint:
//   - springs at equilibrium compression
//   - vertical position/velocity clamped (can't fall through or bounce off ground)
//   - pitch/roll held to surface slope angle
static void simulateWithSprings(VehiclePhysics& v, const InputState& in,
                                 float seconds, float compression = EQ_COMPRESSION) {
    float targetPitch = v.state().pitch;
    float targetRoll  = v.state().roll;
    float groundY     = v.state().position.y;
    for (float t = 0.f; t < seconds; t += DT) {
        loadSprings(v, compression);
        v.update(DT, in);
        // Ground constraint
        v.mutableState().position.y = groundY;
        v.mutableState().velocity.y = 0.f;
        v.mutableState().pitch = targetPitch;
        v.mutableState().roll  = targetRoll;
        v.mutableState().bodyRotation = rotationFromAngles(
            v.state().heading, targetPitch, targetRoll);
    }
}

// Like simulateWithSprings but allows roll/pitch to change freely (for rollover tests).
static void simulateWithSpringsUnclamped(VehiclePhysics& v, const InputState& in,
                                          float seconds, float compression = EQ_COMPRESSION) {
    float groundY = v.state().position.y;
    for (float t = 0.f; t < seconds; t += DT) {
        loadSprings(v, compression);
        v.update(DT, in);
        // Still clamp vertical to simulate ground contact
        v.mutableState().position.y = groundY;
        v.mutableState().velocity.y = 0.f;
    }
}

static VehiclePhysics makeVehicle() {
    VehiclePhysics v;
    v.init();
    return v;
}

// ============================= Tree structure =============================

static void testComponentTreeInit() {
    std::printf("  testComponentTreeInit\n");
    auto v = makeVehicle();

    CHECK(v.body()          != nullptr, "Body exists");
    CHECK(v.splitter()      != nullptr, "Splitter exists");
    CHECK(v.rearWing()      != nullptr, "RearWing exists");
    CHECK(v.frontSubframe() != nullptr, "FrontSubframe exists");
    CHECK(v.rearSubframe()  != nullptr, "RearSubframe exists");

    CHECK(v.body()->mass()  != nullptr, "Body has Mass child");

    CHECK(v.frontSubframe()->left()  != nullptr, "Front has left corner");
    CHECK(v.frontSubframe()->right() != nullptr, "Front has right corner");
    CHECK(v.rearSubframe()->left()   != nullptr, "Rear has left corner");
    CHECK(v.rearSubframe()->right()  != nullptr, "Rear has right corner");

    // Each corner has spring + tire
    auto* fl = v.frontSubframe()->left();
    CHECK(fl->spring() != nullptr, "FL has spring");
    CHECK(fl->tire()   != nullptr, "FL has tire");
}

static void testFrontSubframeSteered() {
    std::printf("  testFrontSubframeSteered\n");
    auto v = makeVehicle();

    auto* flTire = v.frontSubframe()->left()->tire();
    auto* rrTire = v.rearSubframe()->right()->tire();

    CHECK(flTire->steered == true,  "Front tire is steered");
    CHECK(flTire->driven  == false, "Front tire is not driven");
    CHECK(rrTire->steered == false, "Rear tire is not steered");
    CHECK(rrTire->driven  == true,  "Rear tire is driven (RWD)");
}

// ========================= Gravity / Mass =================================

static void testGravityDirectionIsDown() {
    std::printf("  testGravityDirectionIsDown\n");
    auto v = makeVehicle();

    ComponentInput ci{};
    ci.dt = 1.f / 120.f;
    ci.bodyRotation = glm::mat3(1.f);

    auto out = v.body()->update(ci);

    CHECK(out.force.y < 0.f, "Gravity force is downward");
    CHECK(std::abs(out.force.x) < 0.01f, "Gravity has no X component");
    CHECK(std::abs(out.force.z) < 0.01f, "Gravity has no Z component");
}

static void testMassReportsCorrectWeight() {
    std::printf("  testMassReportsCorrectWeight\n");
    auto v = makeVehicle();
    float totalMass = v.body()->totalMassKg();
    CHECK(totalMass > 0.f, "Total mass is positive");
}

// ======================== Aero surfaces ===================================

static void testDragOpposesForwardMotion() {
    std::printf("  testDragOpposesForwardMotion\n");
    auto v = makeVehicle();

    ComponentInput ci{};
    ci.dt = 1.f / 120.f;
    ci.velocity = glm::vec3(0.f, 0.f, 30.f);  // moving forward (+Z)
    ci.bodyRotation = glm::mat3(1.f);

    auto out = v.splitter()->update(ci);
    CHECK(out.force.z < 0.f, "Splitter drag opposes forward motion");

    auto out2 = v.rearWing()->update(ci);
    CHECK(out2.force.z < 0.f, "RearWing drag opposes forward motion");
}

static void testDragOpposesReverseMotion() {
    std::printf("  testDragOpposesReverseMotion\n");
    auto v = makeVehicle();

    ComponentInput ci{};
    ci.dt = 1.f / 120.f;
    ci.velocity = glm::vec3(0.f, 0.f, -20.f);  // reversing
    ci.bodyRotation = glm::mat3(1.f);

    auto out = v.splitter()->update(ci);
    CHECK(out.force.z > 0.f, "Splitter drag opposes reverse motion");
}

static void testDownforceIsDownward() {
    std::printf("  testDownforceIsDownward\n");
    auto v = makeVehicle();

    ComponentInput ci{};
    ci.dt = 1.f / 120.f;
    ci.velocity = glm::vec3(0.f, 0.f, 30.f);
    ci.bodyRotation = glm::mat3(1.f);

    auto out = v.splitter()->update(ci);
    CHECK(out.force.y < 0.f, "Splitter produces downforce (negative lift)");

    auto out2 = v.rearWing()->update(ci);
    CHECK(out2.force.y < 0.f, "RearWing produces downforce (negative lift)");
}

static void testNoAeroAtZeroSpeed() {
    std::printf("  testNoAeroAtZeroSpeed\n");
    auto v = makeVehicle();

    ComponentInput ci{};
    ci.dt = 1.f / 120.f;
    ci.velocity = glm::vec3(0.f);
    ci.bodyRotation = glm::mat3(1.f);

    auto out = v.splitter()->update(ci);
    CHECK(std::abs(out.force.x) < 0.001f &&
          std::abs(out.force.y) < 0.001f &&
          std::abs(out.force.z) < 0.001f,
          "No aero force at zero speed");
}

static void testAeroIncreasesWithSpeed() {
    std::printf("  testAeroIncreasesWithSpeed\n");
    auto v = makeVehicle();

    ComponentInput ci{};
    ci.dt = 1.f / 120.f;
    ci.bodyRotation = glm::mat3(1.f);

    ci.velocity = glm::vec3(0.f, 0.f, 20.f);
    auto slow = v.splitter()->update(ci);

    ci.velocity = glm::vec3(0.f, 0.f, 40.f);
    auto fast = v.splitter()->update(ci);

    CHECK(std::abs(fast.force.z) > std::abs(slow.force.z),
          "Drag increases with speed");
    CHECK(std::abs(fast.force.y) > std::abs(slow.force.y),
          "Downforce increases with speed");
}

// ======================= Spring / Damper ==================================

static void testSpringCompressionProducesUpwardForce() {
    std::printf("  testSpringCompressionProducesUpwardForce\n");
    auto v = makeVehicle();

    auto* spring = v.frontSubframe()->left()->spring();
    spring->setCompression(0.05f, 0.f);

    ComponentInput ci{};
    ci.bodyRotation = glm::mat3(1.f);

    auto out = spring->update(ci);
    CHECK(out.force.y > 0.f, "Compressed spring pushes up");
}

static void testSpringNoTensionForce() {
    std::printf("  testSpringNoTensionForce\n");
    auto v = makeVehicle();

    auto* spring = v.frontSubframe()->left()->spring();
    spring->setCompression(-0.05f, 0.f);  // extended beyond rest

    ComponentInput ci{};
    ci.bodyRotation = glm::mat3(1.f);

    auto out = spring->update(ci);
    CHECK(out.force.y >= 0.f, "Spring cannot pull (no tension)");
}

static void testDamperAddsToSpringForce() {
    std::printf("  testDamperAddsToSpringForce\n");
    auto v = makeVehicle();
    auto* spring = v.frontSubframe()->left()->spring();

    ComponentInput ci{};
    ci.bodyRotation = glm::mat3(1.f);

    spring->setCompression(0.05f, 0.f);
    auto noVel = spring->update(ci);

    spring->setCompression(0.05f, 0.1f);  // compressing
    auto withVel = spring->update(ci);

    CHECK(withVel.force.y > noVel.force.y,
          "Damper adds force during compression");
}

// ========================== Tire ==========================================

static void testTireNoForceWithoutLoad() {
    std::printf("  testTireNoForceWithoutLoad\n");
    auto v = makeVehicle();

    auto* tire = v.frontSubframe()->left()->tire();
    tire->setDriveTorque(1000.f);

    ComponentInput ci{};
    ci.bodyRotation = glm::mat3(1.f);
    ci.normalLoad = 0.f;  // no load = airborne

    auto out = tire->update(ci);
    float mag = glm::length(out.force);
    CHECK(mag < 0.001f, "Tire produces no force without normal load");
}

static void testTireThrottleProducesForwardForce() {
    std::printf("  testTireThrottleProducesForwardForce\n");
    auto v = makeVehicle();

    // Rear tire with drive torque
    auto* tire = v.rearSubframe()->left()->tire();
    tire->setDriveTorque(1000.f);  // ~3200 N at wheel

    ComponentInput ci{};
    ci.bodyRotation = glm::mat3(1.f);
    ci.normalLoad = 3000.f;
    ci.velocity = glm::vec3(0.f, 0.f, 5.f);  // slight forward speed

    auto out = tire->update(ci);
    CHECK(out.force.z > 0.f, "Drive torque produces forward force");
}

static void testTireBrakeOpposesMotion() {
    std::printf("  testTireBrakeOpposesMotion\n");
    auto v = makeVehicle();

    auto* tire = v.rearSubframe()->left()->tire();
    tire->setBrakeTorque(500.f);  // ~1600 N braking at wheel

    ComponentInput ci{};
    ci.bodyRotation = glm::mat3(1.f);
    ci.normalLoad = 3000.f;
    ci.velocity = glm::vec3(0.f, 0.f, 10.f);  // moving forward

    auto out = tire->update(ci);
    CHECK(out.force.z < 0.f, "Brake torque produces rearward force");
}

static void testTireLateralForceFromSlip() {
    std::printf("  testTireLateralForceFromSlip\n");
    auto v = makeVehicle();

    auto* tire = v.frontSubframe()->left()->tire();
    // No drive/brake/steer — just lateral slip

    ComponentInput ci{};
    ci.bodyRotation = glm::mat3(1.f);
    ci.normalLoad = 3000.f;
    ci.velocity = glm::vec3(2.f, 0.f, 20.f);  // sliding right

    auto out = tire->update(ci);
    CHECK(out.force.x < 0.f, "Lateral slip to right produces leftward restoring force");
}

static void testTireLateralForceOppositeSlipDirection() {
    std::printf("  testTireLateralForceOppositeSlipDirection\n");
    auto v = makeVehicle();

    auto* tire = v.frontSubframe()->left()->tire();

    ComponentInput ci{};
    ci.bodyRotation = glm::mat3(1.f);
    ci.normalLoad = 3000.f;

    ci.velocity = glm::vec3(2.f, 0.f, 20.f);  // sliding right
    auto outR = tire->update(ci);

    ci.velocity = glm::vec3(-2.f, 0.f, 20.f); // sliding left
    auto outL = tire->update(ci);

    CHECK(outR.force.x < 0.f, "Right slip → leftward force");
    CHECK(outL.force.x > 0.f, "Left slip → rightward force");
}

static void testFrontTireNotDriven() {
    std::printf("  testFrontTireNotDriven\n");
    auto v = makeVehicle();

    auto* tire = v.frontSubframe()->left()->tire();
    // No drive torque set (Drivetrain only sends to rear)

    ComponentInput ci{};
    ci.bodyRotation = glm::mat3(1.f);
    ci.normalLoad = 3000.f;
    ci.velocity = glm::vec3(0.f, 0.f, 5.f);

    auto out = tire->update(ci);
    // Without drive torque, tire should produce only rolling resistance
    // (opposing forward motion). At 3000N load, ~45N retarding force.
    CHECK(out.force.z < 0.f,
          "Tire with no drive torque: rolling resistance opposes motion");
    CHECK(std::abs(out.force.z) < 100.f,
          "Tire with no drive torque: force is small (rolling resistance only)");
}

// ====================== Controls routing ==================================

static void testControlsRoutedCorrectly() {
    std::printf("  testControlsRoutedCorrectly\n");
    auto v = makeVehicle();

    // Run one update to trigger routeControls
    InputState input{};
    input.steer     = 1.0f;    // full right
    input.throttle  = 0.8f;
    input.brake     = 0.5f;
    input.handBrake = 0.0f;
    loadSprings(v);
    v.update(DT, input);

    auto* flTire = v.frontSubframe()->left()->tire();
    auto* rrTire = v.rearSubframe()->right()->tire();

    // Steering: only front tires
    float expectedSteer = 1.0f * 0.55f;  // maxSteerAngle_
    CHECK(APPROX(flTire->steerAngle(), expectedSteer, 0.01f),
          "Front tire gets steer angle");
    CHECK(APPROX(rrTire->steerAngle(), 0.f, 0.001f),
          "Rear tire has zero steer angle");

    // Drive torque: only rear tires (RWD)
    CHECK(rrTire->driveTorque() > 0.f,
          "Rear tire gets drive torque from drivetrain");
    CHECK(APPROX(flTire->driveTorque(), 0.f, 0.001f),
          "Front tire gets no drive torque");

    // Brake torque: all tires, front-biased
    CHECK(flTire->brakeTorque() > 0.f, "Front tire gets brake torque");
    CHECK(rrTire->brakeTorque() > 0.f, "Rear tire gets brake torque");
    CHECK(flTire->brakeTorque() > rrTire->brakeTorque(),
          "Front brake torque > rear (60/40 bias)");
}

// ====================== Torque / Moment arms ==============================

static void testOffsetForceProducesTorque() {
    std::printf("  testOffsetForceProducesTorque\n");
    auto v = makeVehicle();

    // Splitter is at front (+Z), produces downforce (-Y).
    // Cross product: r=(0,0,+Z) x F=(0,-Y,0) should produce torque
    // that pitches the nose down (negative pitch = nose-down around X axis).
    ComponentInput ci{};
    ci.velocity = glm::vec3(0.f, 0.f, 30.f);
    ci.bodyRotation = glm::mat3(1.f);

    // Update splitter in isolation
    auto splitterOut = v.splitter()->update(ci);
    glm::vec3 r = v.splitter()->attachmentPoint();
    glm::vec3 torqueFromSplitter = glm::cross(r, splitterOut.force);

    // Splitter is at positive Z, downforce is negative Y
    // cross((0,y,+z), (0,-F,dragZ)) → torque about X
    // The exact sign depends on geometry, but torque should be non-zero
    float torqueMag = glm::length(torqueFromSplitter);
    CHECK(torqueMag > 0.1f, "Offset aero force produces non-zero torque");
}

static void testLeftRightSymmetryOfSubframe() {
    std::printf("  testLeftRightSymmetryOfSubframe\n");
    auto v = makeVehicle();

    // Set identical compression on both sides
    v.frontSubframe()->left()->spring()->setCompression(0.05f, 0.f);
    v.frontSubframe()->right()->spring()->setCompression(0.05f, 0.f);

    ComponentInput ci{};
    ci.bodyRotation = glm::mat3(1.f);

    auto leftOut  = v.frontSubframe()->left()->spring()->update(ci);
    auto rightOut = v.frontSubframe()->right()->spring()->update(ci);

    CHECK(APPROX(leftOut.force.y, rightOut.force.y, 0.01f),
          "Symmetric compression produces symmetric vertical force");
}

static void testAsymmetricCompressionProducesRollTorque() {
    std::printf("  testAsymmetricCompressionProducesRollTorque\n");
    auto v = makeVehicle();

    // Left side more compressed than right
    v.frontSubframe()->left()->spring()->setCompression(0.08f, 0.f);
    v.frontSubframe()->right()->spring()->setCompression(0.02f, 0.f);

    ComponentInput ci{};
    ci.bodyRotation = glm::mat3(1.f);

    auto out = v.frontSubframe()->update(ci);
    // Asymmetric vertical forces at different X offsets → roll torque (about Z)
    CHECK(std::abs(out.torque.z) > 0.1f || std::abs(out.torque.x) > 0.1f,
          "Asymmetric compression produces roll torque");
}

// ======================== Force propagation ================================

static void testTireForceReachesVehicle() {
    std::printf("  testTireForceReachesVehicle\n");
    auto v = makeVehicle();

    InputState input{};
    input.throttle = 1.f;
    simulateWithSprings(v, input, 0.5f);

    // Vehicle should have moved forward (+Z)
    CHECK(v.state().velocity.z > 0.f,
          "Rear tire throttle force propagates to vehicle velocity");
    CHECK(v.state().position.z > 0.f,
          "Rear tire throttle force propagates to vehicle position");
}

static void testBrakingSlowsVehicle() {
    std::printf("  testBrakingSlowsVehicle\n");
    auto v = makeVehicle();

    // Accelerate first
    InputState gas{};
    gas.throttle = 1.f;
    simulateWithSprings(v, gas, 1.f);
    float speedBefore = v.state().velocity.z;
    CHECK(speedBefore > 0.f, "Vehicle moving forward before braking");

    // Now brake
    InputState brk{};
    brk.brake = 1.f;
    simulateWithSprings(v, brk, 1.f);
    float speedAfter = v.state().velocity.z;

    CHECK(speedAfter < speedBefore,
          "Braking reduces forward speed");
}

static void testSteeringProducesLateralForce() {
    std::printf("  testSteeringProducesLateralForce\n");
    auto v = makeVehicle();

    // Build up forward speed
    InputState gas{};
    gas.throttle = 1.f;
    simulateWithSprings(v, gas, 2.f);
    CHECK(std::abs(v.state().velocity.z) > 0.1f, "Vehicle has forward speed before steering");

    // Steer left — steered front tires redirect force, producing lateral velocity
    InputState steerLeft{};
    steerLeft.throttle = 0.5f;
    steerLeft.steer = -1.f;
    simulateWithSprings(v, steerLeft, 1.f);

    // Steering should produce a lateral velocity component
    CHECK(v.state().velocity.x != 0.f || v.state().position.x != 0.f,
          "Steering while moving produces lateral motion");
}

static void testGravityPullsVehicleDown() {
    std::printf("  testGravityPullsVehicleDown\n");
    auto v = makeVehicle();

    // No suspension compression = no upward force = freefall
    InputState none{};
    simulate(v, none, 0.5f);

    CHECK(v.state().velocity.y < 0.f, "Gravity accelerates vehicle downward");
    CHECK(v.state().position.y < 0.f, "Gravity moves vehicle downward");
}

// ========================= Drawables ======================================

static void testDrawablesNonEmpty() {
    std::printf("  testDrawablesNonEmpty\n");
    auto v = makeVehicle();

    // Set some compression so springs have geometry
    v.frontSubframe()->left()->spring()->setCompression(0.05f, 0.f);

    Drawable d = v.collectDrawables();
    CHECK(!d.empty(), "collectDrawables returns non-empty geometry");
    CHECK(d.indices.size() > 0, "Drawables have indices");
    CHECK(d.vertices.size() > 0, "Drawables have vertices");
}

static void testDrawablesMergeCorrectly() {
    std::printf("  testDrawablesMergeCorrectly\n");

    Drawable a, b;
    a.vertices.push_back({{0,0,0},{0,1,0},{1,1,1,1}});
    a.indices.push_back(0);

    b.vertices.push_back({{1,0,0},{0,1,0},{1,1,1,1}});
    b.vertices.push_back({{2,0,0},{0,1,0},{1,1,1,1}});
    b.indices.push_back(0);
    b.indices.push_back(1);

    a.merge(b);
    CHECK(a.vertices.size() == 3, "Merged vertex count");
    CHECK(a.indices.size() == 3, "Merged index count");
    CHECK(a.indices[1] == 1, "Merged index offset correct (first b index)");
    CHECK(a.indices[2] == 2, "Merged index offset correct (second b index)");
}

// =========================== Reset ========================================

static void testResetZerosState() {
    std::printf("  testResetZerosState\n");
    auto v = makeVehicle();

    InputState gas{};
    gas.throttle = 1.f;
    v.rearSubframe()->left()->spring()->setCompression(0.05f, 0.f);
    v.rearSubframe()->right()->spring()->setCompression(0.05f, 0.f);
    simulate(v, gas, 1.f);

    CHECK(glm::length(v.state().velocity) > 0.f, "Vehicle moved before reset");

    v.reset();

    CHECK(APPROX(glm::length(v.state().velocity), 0.f, 0.001f),
          "Velocity zeroed after reset");
    CHECK(APPROX(glm::length(v.state().position), 0.f, 0.001f),
          "Position zeroed after reset");
    CHECK(APPROX(v.state().heading, 0.f, 0.001f),
          "Heading zeroed after reset");
}

// ======================== Equilibrium / Stopping ===========================

static void testStationaryOnFlatGround() {
    std::printf("  testStationaryOnFlatGround\n");
    auto v = makeVehicle();

    InputState none{};
    simulateWithSprings(v, none, 2.f);

    float speed = glm::length(v.state().velocity);
    CHECK(speed < 0.5f, "Vehicle stays near-motionless on flat ground");
}

static void testDriveAndStopWithBrake() {
    std::printf("  testDriveAndStopWithBrake\n");
    auto v = makeVehicle();

    // Drive forward
    InputState gas{};
    gas.throttle = 1.f;
    simulateWithSprings(v, gas, 1.f);
    CHECK(v.state().velocity.z > 0.5f, "Vehicle moving before braking");

    // Brake to stop
    InputState brk{};
    brk.brake = 1.f;
    simulateWithSprings(v, brk, 3.f);

    // Should be nearly stopped
    float speed = std::abs(v.state().velocity.z);
    CHECK(speed < 0.5f, "Vehicle nearly stopped after sustained braking");

    // Hold still with no inputs — stiction should keep it motionless
    InputState none{};
    float speedBefore = glm::length(v.state().velocity);
    simulateWithSprings(v, none, 2.f);
    float speedAfter = glm::length(v.state().velocity);
    CHECK(speedAfter < speedBefore + 0.1f,
          "Stopped vehicle stays motionless without brake");
}

static void testHandBrakeLocksRearWheels() {
    std::printf("  testHandBrakeLocksRearWheels\n");
    auto v = makeVehicle();

    // Drive forward
    InputState gas{};
    gas.throttle = 1.f;
    simulateWithSprings(v, gas, 1.f);
    float speedBefore = v.state().velocity.z;
    CHECK(speedBefore > 0.f, "Vehicle moving before handbrake");

    // Apply handbrake (no foot brake, no throttle)
    InputState hb{};
    hb.handBrake = 1.f;
    simulateWithSprings(v, hb, 2.f);
    float speedAfter = v.state().velocity.z;

    CHECK(speedAfter < speedBefore,
          "Handbrake slows the vehicle");
}

// ======================== Slope scenarios ==================================

// Helper: set vehicle onto a slope defined by roll and pitch angles.
// groundNormal is the slope surface normal (body Y direction on the slope).
static void placeOnSlope(VehiclePhysics& v, float pitch, float roll) {
    v.mutableState().pitch = pitch;
    v.mutableState().roll  = roll;
    glm::mat3 rot = rotationFromAngles(0.f, pitch, roll);
    v.mutableState().bodyRotation = rot;
    // Ground normal = the slope surface "up" direction
    v.setGroundNormal(rot * glm::vec3(0.f, 1.f, 0.f));
}

static void testSlipperySideHillSlides() {
    std::printf("  testSlipperySideHillSlides\n");
    auto v = makeVehicle();
    v.setSurfaceGrip(0.05f);  // ice

    // Place on a 25° side slope (tilted right)
    placeOnSlope(v, 0.f, 0.44f);

    InputState none{};
    simulateWithSprings(v, none, 2.f);

    // Vehicle should slide laterally (downhill)
    float lateralSpeed = std::abs(v.state().velocity.x);
    CHECK(lateralSpeed > 0.5f,
          "Vehicle slides laterally on icy side hill");
}

static void testGrippySideHillRolls() {
    std::printf("  testGrippySideHillRolls\n");
    auto v = makeVehicle();
    v.setSurfaceGrip(1.0f);  // grippy tarmac

    // Place on a steep 35° side slope
    float slopeAngle = 0.61f;  // ~35 degrees — steep enough to roll
    placeOnSlope(v, 0.f, slopeAngle);

    InputState none{};
    simulateWithSpringsUnclamped(v, none, 2.f);

    // With grip, tires resist sliding but the high CG creates a roll torque.
    // The roll angle should increase (vehicle tipping over).
    CHECK(std::abs(v.state().roll) > slopeAngle + 0.01f ||
          std::abs(v.state().angularVel.z) > 0.1f,
          "Vehicle rolls on steep grippy side hill");
}

static void testSlippyDownhillSlides() {
    std::printf("  testSlippyDownhillSlides\n");
    auto v = makeVehicle();
    v.setSurfaceGrip(0.05f);  // ice

    // Place on a 20° downhill slope (nose down = positive pitch)
    placeOnSlope(v, 0.35f, 0.f);

    InputState brk{};
    brk.brake = 1.f;  // try to brake but ice won't let you
    simulateWithSprings(v, brk, 2.f);

    // Vehicle should slide forward (downhill = +Z when nose points down)
    float fwdSpeed = v.state().velocity.z;
    CHECK(fwdSpeed > 0.5f,
          "Vehicle slides downhill on icy slope despite braking");
}

static void testGrippyUphillHoldsWithBrake() {
    std::printf("  testGrippyUphillHoldsWithBrake\n");
    auto v = makeVehicle();
    v.setSurfaceGrip(1.0f);  // grippy

    // Place on a moderate 10° uphill (nose up = negative pitch)
    placeOnSlope(v, -0.17f, 0.f);

    InputState brk{};
    brk.brake = 1.f;
    simulateWithSprings(v, brk, 2.f);

    // Vehicle should hold position (not slide backward significantly)
    float speed = glm::length(v.state().velocity);
    CHECK(speed < 1.f,
          "Vehicle holds on grippy uphill with brake applied");
}

static void testSlippyUphillSlidesBack() {
    std::printf("  testSlippyUphillSlidesBack\n");
    auto v = makeVehicle();
    v.setSurfaceGrip(0.05f);  // ice

    // Place on a 15° uphill (nose up = negative pitch)
    placeOnSlope(v, -0.26f, 0.f);

    InputState brk{};
    brk.brake = 1.f;  // brakes can't hold on ice
    simulateWithSprings(v, brk, 2.f);

    // Vehicle should slide backward (negative Z = downhill)
    CHECK(v.state().velocity.z < -0.3f,
          "Vehicle slides backward on icy uphill despite braking");
}

static void testSurfaceGripAffectsTireForce() {
    std::printf("  testSurfaceGripAffectsTireForce\n");
    auto v = makeVehicle();

    auto* tire = v.rearSubframe()->left()->tire();
    tire->setDriveTorque(1000.f);

    ComponentInput ci{};
    ci.bodyRotation = glm::mat3(1.f);
    ci.normalLoad = 3000.f;
    ci.velocity = glm::vec3(0.f, 0.f, 5.f);

    ci.surfaceGrip = 1.0f;
    auto fullGrip = tire->update(ci);

    ci.surfaceGrip = 0.1f;
    auto lowGrip = tire->update(ci);

    CHECK(std::abs(fullGrip.force.z) > std::abs(lowGrip.force.z),
          "Lower surface grip reduces tire force");
}

// ========================= Freefall ========================================

static void testFreefallPitchesFromCGOffset() {
    std::printf("  testFreefallPitchesFromCGOffset\n");
    auto v = makeVehicle();

    // CG is at z=-0.05 (slightly behind center) and rear wing produces more
    // downforce than splitter. Both effects pitch the nose UP in freefall.
    // To pitch nose DOWN, CG would need to be forward of the aero center
    // of pressure.
    v.mutableState().velocity = glm::vec3(0.f, 0.f, 30.f);

    InputState none{};
    simulate(v, none, 1.f);

    // With rear-biased CG and rear-biased downforce, nose pitches UP
    // (negative pitch = nose up in our convention)
    CHECK(v.state().pitch < -0.01f,
          "Freefall pitches nose up (CG behind center, rear-heavy downforce)");
}

static void testFreefallDropsWithoutSprings() {
    std::printf("  testFreefallDropsWithoutSprings\n");
    auto v = makeVehicle();

    InputState none{};
    simulate(v, none, 1.f);

    // After 1 second of freefall: v = g*t = 9.81 m/s downward
    CHECK(v.state().velocity.y < -9.f, "Freefall velocity ~ g*t");
    // Position: 0.5*g*t^2 = ~4.9m
    CHECK(v.state().position.y < -4.f, "Freefall position ~ 0.5*g*t^2");
}

// ========================= Body roll =======================================

static void testAccelerationCausesRearSquat() {
    std::printf("  testAccelerationCausesRearSquat\n");
    auto v = makeVehicle();

    // Accelerate hard — weight transfers to rear, nose lifts
    InputState gas{};
    gas.throttle = 1.f;
    simulateWithSpringsUnclamped(v, gas, 1.f);

    // Positive pitch = nose down, so acceleration should cause negative pitch
    // (nose up / rear squat) OR positive depending on convention...
    // Drive force is at rear subframe (z=-1.25), below CG (y=0.35).
    // Torque from rear drive force = cross((-1.25, 0, 0)_relative, (0, 0, Fz))
    // Actually the relevant torque is from weight transfer due to longitudinal accel.
    // Under acceleration, the CG's inertia causes the body to pitch nose-up.
    // With our convention: positive angularVel.x → increasing pitch → nose down.
    // So acceleration should make pitch go negative (nose up).
    CHECK(v.state().pitch < -0.001f || v.state().angularVel.x < 0.f,
          "Acceleration causes nose-up pitch (rear squat)");
}

static void testCGOffsetCreatesPitchTorque() {
    std::printf("  testCGOffsetCreatesPitchTorque\n");
    auto v = makeVehicle();

    // CG is at z=-0.05 (behind center). Gravity at this offset creates
    // a constant nose-up pitch torque about the vehicle origin.
    // With unclamped pitch and constant springs, the pitch should accumulate.
    InputState none{};
    simulateWithSpringsUnclamped(v, none, 1.f);

    // Negative pitch = nose up (rear-biased CG pulls rear down)
    CHECK(v.state().pitch < -0.01f,
          "CG behind center creates nose-up pitch torque");
}

static void testCorneringProducesYaw() {
    std::printf("  testCorneringProducesYaw\n");
    auto v = makeVehicle();

    // Build speed
    InputState gas{};
    gas.throttle = 1.f;
    simulateWithSprings(v, gas, 1.f);

    // Steer left
    InputState steerLeft{};
    steerLeft.throttle = 0.5f;
    steerLeft.steer = -1.f;
    simulateWithSprings(v, steerLeft, 1.f);

    // Negative heading = left turn in our convention
    CHECK(v.state().heading < -0.01f,
          "Steering left produces left-turn heading change");
    CHECK(v.state().angularVel.y < 0.f,
          "Steering left produces negative yaw rate");
}

// ========================= Steer-then-stop =================================

static void testSteerThenStopNoResidualYaw() {
    std::printf("  testSteerThenStopNoResidualYaw\n");
    auto v = makeVehicle();

    // Drive and steer
    InputState steer{};
    steer.throttle = 1.f;
    steer.steer = -0.5f;
    simulateWithSprings(v, steer, 1.f);
    CHECK(std::abs(v.state().angularVel.y) > 0.01f,
          "Vehicle has yaw rate while steering");

    // Straighten and brake to stop
    InputState brk{};
    brk.brake = 1.f;
    simulateWithSprings(v, brk, 3.f);

    // Hold still — low-speed angular damping from tire contact patches kills yaw
    InputState none{};
    simulateWithSprings(v, none, 2.f);

    float yawRate = std::abs(v.state().angularVel.y);
    CHECK(yawRate < 0.05f,
          "No residual yaw after steering, braking, and resting");
}

// ========================= Brake lock-up ===================================

static void testBrakeLockUpEasierOnIce() {
    std::printf("  testBrakeLockUpEasierOnIce\n");
    auto v = makeVehicle();

    auto* tire = v.rearSubframe()->left()->tire();
    tire->setBrakeTorque(500.f);

    ComponentInput ci{};
    ci.bodyRotation = glm::mat3(1.f);
    ci.normalLoad = 3000.f;
    ci.velocity = glm::vec3(0.f, 0.f, 10.f);

    // Full grip: brake force available
    ci.surfaceGrip = 1.0f;
    auto dryBrake = tire->update(ci);

    // Ice: much less brake force available (limited by friction)
    ci.surfaceGrip = 0.05f;
    auto iceBrake = tire->update(ci);

    CHECK(std::abs(dryBrake.force.z) > std::abs(iceBrake.force.z) * 5.f,
          "Dry braking force >> ice braking force");
    CHECK(std::abs(iceBrake.force.z) > 0.f,
          "Some brake force even on ice");
}

// ========================= Spring bounce ===================================

static void testGroundMovesUpCompressesSprings() {
    std::printf("  testGroundMovesUpCompressesSprings\n");
    auto v = makeVehicle();

    // Start at equilibrium on flat ground
    InputState none{};
    simulateWithSprings(v, none, 1.f);
    float velBefore = v.state().velocity.y;

    // Ground suddenly rises — increase spring compression beyond equilibrium
    // (simulates hitting a bump or terrain rising under the car)
    float bumpCompression = EQ_COMPRESSION + 0.05f;  // 5cm extra compression
    loadSprings(v, bumpCompression);

    ComponentInput ci = {};
    ci.bodyRotation = glm::mat3(1.f);

    // Check that the extra compression produces more upward force
    auto* spring = v.frontSubframe()->left()->spring();
    spring->setCompression(bumpCompression, 0.f);
    auto out = spring->update(ci);

    float eqForce  = 30000.f * EQ_COMPRESSION;  // equilibrium force
    float bumpForce = out.force.y;
    CHECK(bumpForce > eqForce * 1.3f,
          "Extra compression produces significantly more spring force");
}

static void testVehicleBounces() {
    std::printf("  testVehicleBounces\n");
    auto v = makeVehicle();

    // Start at rest on flat ground
    InputState none{};
    simulateWithSprings(v, none, 0.5f);

    // Simulate a bump: over-compress springs for a few frames, then
    // return to equilibrium. The extra upward impulse should make
    // the vehicle bounce upward.
    float bumpCompression = EQ_COMPRESSION + 0.08f;
    for (int i = 0; i < 10; ++i) {
        loadSprings(v, bumpCompression);
        v.update(DT, none);
    }

    // After the bump, the vehicle should have upward velocity
    CHECK(v.state().velocity.y > 0.1f,
          "Vehicle bounces upward after bump");
}

// ========================= Collisions ======================================

// An infinite plane obstacle defined by a point on the plane and its outward normal.
struct PlaneObstacle {
    glm::vec3 point;   // any point on the plane
    glm::vec3 normal;  // outward normal (pointing away from the solid side)
};

// Generate collision contacts for body corners penetrating an obstacle plane.
static std::vector<CollisionContact> checkPlane(VehiclePhysics& v,
                                                 const PlaneObstacle& plane) {
    std::vector<CollisionContact> contacts;
    auto corners = v.body()->colliderCorners();
    const auto& st = v.state();
    glm::vec3 cgWorld = st.position + st.bodyRotation * v.body()->attachmentPoint();

    for (auto& localCorner : corners) {
        glm::vec3 worldPt = cgWorld + st.bodyRotation * localCorner;
        // Signed distance to plane (negative = penetrating)
        float dist = glm::dot(worldPt - plane.point, plane.normal);
        if (dist < 0.f) {
            contacts.push_back({
                .worldPoint  = worldPt,
                .normal      = plane.normal,
                .penetration = -dist
            });
        }
    }
    return contacts;
}

// Simulate with collision against obstacle planes each step.
static void simulateWithCollisions(VehiclePhysics& v, const InputState& in,
                                    float seconds,
                                    const std::vector<PlaneObstacle>& obstacles) {
    for (float t = 0.f; t < seconds; t += DT) {
        // Check all obstacles and accumulate contacts
        std::vector<CollisionContact> allContacts;
        for (auto& obs : obstacles) {
            auto c = checkPlane(v, obs);
            allContacts.insert(allContacts.end(), c.begin(), c.end());
        }
        // Apply collision response (stores forces in Body)
        v.applyCollisions(allContacts, DT);
        // Normal physics step
        v.update(DT, in);
    }
}

// --- 45-degree incline tests (ramp facing the vehicle) ---

static void testHeadOnRampNoLaunch() {
    std::printf("  testHeadOnRampNoLaunch\n");
    auto v = makeVehicle();
    v.body()->setRestitution(0.1f);  // energy-absorbing

    // Vehicle driving forward at 20 m/s into a 45-degree ramp
    v.mutableState().velocity = glm::vec3(0.f, 0.f, 20.f);

    // Ramp plane: starts at z=3.0m, faces backward-and-up at 45 degrees.
    // Normal = (0, sin45, -cos45) = (0, 0.707, -0.707)
    float n45 = 0.7071f;
    PlaneObstacle ramp{
        .point  = glm::vec3(0.f, 0.f, 3.0f),
        .normal = glm::vec3(0.f, n45, -n45)
    };

    InputState none{};
    simulateWithCollisions(v, none, 1.0f, { ramp });

    // The key check: upward velocity should not exceed forward approach speed.
    // With restitution 0.1, the vehicle should NOT be launched sky-high.
    float upVel = v.state().velocity.y;
    CHECK(upVel < 15.f,
          "Head-on ramp: upward velocity not disproportionate to approach speed");
    // Forward speed should be significantly reduced
    CHECK(v.state().velocity.z < 15.f,
          "Head-on ramp: forward speed reduced by collision");
}

static void testGlancingRampNoLaunch() {
    std::printf("  testGlancingRampNoLaunch\n");
    auto v = makeVehicle();
    v.body()->setRestitution(0.1f);

    // Vehicle driving at 20 m/s, slightly angled (10 degrees off-center)
    float angle = 0.17f;  // ~10 degrees
    v.mutableState().velocity = glm::vec3(
        20.f * std::sin(angle), 0.f, 20.f * std::cos(angle));

    // Same 45-degree ramp
    float n45 = 0.7071f;
    PlaneObstacle ramp{
        .point  = glm::vec3(0.f, 0.f, 3.0f),
        .normal = glm::vec3(0.f, n45, -n45)
    };

    InputState none{};
    simulateWithCollisions(v, none, 1.0f, { ramp });

    float upVel = v.state().velocity.y;
    CHECK(upVel < 15.f,
          "Glancing ramp: upward velocity not disproportionate");
    // Vehicle should still be moving (glancing = partial deflection)
    float totalSpeed = glm::length(v.state().velocity);
    CHECK(totalSpeed > 1.f,
          "Glancing ramp: vehicle still moving after deflection");
}

// --- 45-degree overhang tests (body front hangs over edge) ---

static void testOverhangHeadOnRamp() {
    std::printf("  testOverhangHeadOnRamp\n");
    auto v = makeVehicle();
    v.body()->setRestitution(0.1f);

    // Position vehicle so the body overhang (front 0.30m past axle) hits first.
    // Front axle at z=1.35, body front at z=1.65 from CG.
    // Place the ramp close enough that only the overhang contacts it initially.
    v.mutableState().velocity = glm::vec3(0.f, 0.f, 15.f);

    // Ramp at z=2.0 — the overhang extends 0.30m ahead of the front axle,
    // so the bottom-front corners (z = CG_z + 1.65) will hit first.
    float n45 = 0.7071f;
    PlaneObstacle ramp{
        .point  = glm::vec3(0.f, 0.f, 2.0f),
        .normal = glm::vec3(0.f, n45, -n45)
    };

    InputState none{};
    simulateWithCollisions(v, none, 0.5f, { ramp });

    // Key checks:
    float upVel = v.state().velocity.y;
    float fwdVel = v.state().velocity.z;

    // Upward velocity shouldn't be extreme
    CHECK(upVel < 12.f,
          "Overhang head-on: no extreme upward launch");
    // Should decelerate
    CHECK(fwdVel < 15.f,
          "Overhang head-on: forward speed reduced");
    // The pitch change shouldn't be extreme either (contact at overhang creates
    // a torque, but the restitution cap should limit it)
    CHECK(std::abs(v.state().angularVel.x) < 10.f,
          "Overhang head-on: pitch rate not extreme");
}

static void testOverhangGlancingRamp() {
    std::printf("  testOverhangGlancingRamp\n");
    auto v = makeVehicle();
    v.body()->setRestitution(0.1f);

    // Angled approach — only one front corner hits the ramp
    float angle = 0.3f;  // ~17 degrees
    v.mutableState().velocity = glm::vec3(
        15.f * std::sin(angle), 0.f, 15.f * std::cos(angle));
    v.mutableState().heading = angle;
    v.mutableState().bodyRotation = rotationFromAngles(angle, 0.f, 0.f);

    float n45 = 0.7071f;
    PlaneObstacle ramp{
        .point  = glm::vec3(0.f, 0.f, 2.0f),
        .normal = glm::vec3(0.f, n45, -n45)
    };

    InputState none{};
    simulateWithCollisions(v, none, 0.5f, { ramp });

    float upVel = v.state().velocity.y;
    CHECK(upVel < 12.f,
          "Overhang glancing: no extreme upward launch");
    // Should produce some yaw (asymmetric contact)
    CHECK(std::abs(v.state().angularVel.y) > 0.001f ||
          std::abs(v.state().heading - angle) > 0.001f,
          "Overhang glancing: asymmetric contact produces yaw");
    // But yaw shouldn't be extreme
    CHECK(std::abs(v.state().angularVel.y) < 15.f,
          "Overhang glancing: yaw rate not extreme");
}

// --- Wall collision (sanity) ---

static void testWallStopsVehicle() {
    std::printf("  testWallStopsVehicle\n");
    auto v = makeVehicle();
    v.body()->setRestitution(0.1f);

    v.mutableState().velocity = glm::vec3(0.f, 0.f, 20.f);

    // Vertical wall at z=2.5
    PlaneObstacle wall{
        .point  = glm::vec3(0.f, 0.f, 2.5f),
        .normal = glm::vec3(0.f, 0.f, -1.f)
    };

    InputState none{};
    simulateWithCollisions(v, none, 1.0f, { wall });

    // Vehicle should be nearly stopped or bounced back slightly
    CHECK(v.state().velocity.z < 5.f,
          "Wall collision significantly reduces forward speed");
}

static void testCollisionAbsorbsEnergy() {
    std::printf("  testCollisionAbsorbsEnergy\n");
    auto v = makeVehicle();
    v.body()->setRestitution(0.1f);

    float approachSpeed = 20.f;
    v.mutableState().velocity = glm::vec3(0.f, 0.f, approachSpeed);

    PlaneObstacle wall{
        .point  = glm::vec3(0.f, 0.f, 2.5f),
        .normal = glm::vec3(0.f, 0.f, -1.f)
    };

    InputState none{};
    simulateWithCollisions(v, none, 1.0f, { wall });

    // With restitution=0.1, the rebound speed should be much less than approach
    float reboundSpeed = glm::length(v.state().velocity);
    CHECK(reboundSpeed < approachSpeed * 0.5f,
          "Low restitution collision absorbs most energy (crumple, not bounce)");
}

// ========================= Speed bump =====================================

// A raised bump on the road surface with a half-sine profile.
struct SpeedBump {
    float zStart;      // world z where bump begins
    float zLength;     // extent along z
    float height;      // peak height (m)
    float xMin, xMax;  // lateral extent (use large range for full-width)

    float heightAt(float x, float z) const {
        if (x < xMin || x > xMax) return 0.f;
        if (z < zStart || z > zStart + zLength) return 0.f;
        float t = (z - zStart) / zLength;
        return height * std::sin(3.14159265f * t);
    }
};

// Wheel body-local offsets (XZ only — Y handled by spring model).
// Front subframe at z=+1.35, rear at z=-1.25, half-track=0.76
static glm::vec3 wheelLocalPos(bool front, bool left) {
    float x = left ? -0.76f : 0.76f;
    float z = front ? 1.35f : -1.25f;
    return glm::vec3(x, 0.f, z);
}

// Per-wheel spring compression from ground height profile.
// Uses each wheel's actual world Y for geometric pitch/roll restoring.
struct BumpSimState {
    float prevComp[4] = { EQ_COMPRESSION, EQ_COMPRESSION, EQ_COMPRESSION, EQ_COMPRESSION };
    float peakRollDev = 0.f;   // max |roll - baseRoll| during sim
    float peakHeightDev = 0.f; // max |y - baseY| during sim
    float baseRoll = 0.f;
    float baseY    = 0.f;
};

static void simulateOverBump(VehiclePhysics& v, const InputState& in,
                              float seconds, const SpeedBump& bump,
                              BumpSimState* state = nullptr) {
    static constexpr bool frontLeft[4]  = { true,  true,  false, false };
    static constexpr bool leftSide[4]   = { true,  false, true,  false };

    BumpSimState local;
    BumpSimState& s = state ? *state : local;

    for (float t = 0.f; t < seconds; t += DT) {
        const auto& st = v.state();

        SpringDamper* springs[4] = {
            v.frontSubframe()->left()->spring(),
            v.frontSubframe()->right()->spring(),
            v.rearSubframe()->left()->spring(),
            v.rearSubframe()->right()->spring()
        };

        for (int i = 0; i < 4; ++i) {
            glm::vec3 wLocal = wheelLocalPos(frontLeft[i], leftSide[i]);
            glm::vec3 wWorld = st.position + st.bodyRotation * wLocal;
            float groundH = bump.heightAt(wWorld.x, wWorld.z);
            float comp = std::max(0.f, EQ_COMPRESSION + groundH - wWorld.y);
            float compVel = (comp - s.prevComp[i]) / DT;
            springs[i]->setCompression(comp, compVel);
            s.prevComp[i] = comp;
        }

        v.update(DT, in);

        // Floor constraint
        if (v.state().position.y < 0.f && v.state().velocity.y < 0.f) {
            v.mutableState().position.y = 0.f;
            v.mutableState().velocity.y = 0.f;
        }

        // Track peak deviations
        float rollDev = std::abs(v.state().roll - s.baseRoll);
        float heightDev = std::abs(v.state().position.y - s.baseY);
        if (rollDev > s.peakRollDev) s.peakRollDev = rollDev;
        if (heightDev > s.peakHeightDev) s.peakHeightDev = heightDev;
    }
}

static void testFullWidthSpeedBump() {
    std::printf("  testFullWidthSpeedBump\n");
    auto v = makeVehicle();

    // Let the spring model find its own equilibrium on flat ground first
    SpeedBump noBump{ 999.f, 0.5f, 0.08f, -10.f, 10.f }; // bump far away
    InputState cruise{};
    cruise.throttle = 0.3f;
    v.mutableState().velocity = glm::vec3(0.f, 0.f, 10.f);
    simulateOverBump(v, cruise, 1.0f, noBump);

    // Record equilibrium state
    float eqY     = v.state().position.y;
    float eqPitch = v.state().pitch;
    float eqRoll  = v.state().roll;

    // Reset position, keep velocity and equilibrium angles
    v.mutableState().position = glm::vec3(0.f, eqY, 0.f);

    // Full-width bump at z=5, 0.5m long, 8cm tall
    SpeedBump bump{ 5.f, 0.5f, 0.08f, -10.f, 10.f };

    // Drive over the bump
    simulateOverBump(v, cruise, 3.f, bump);

    CHECK(v.state().position.z > 10.f,
          "Vehicle drove past the bump");

    // Body should return near its equilibrium height
    CHECK(std::abs(v.state().position.y - eqY) < 0.03f,
          "Body returns to equilibrium height after full-width bump");

    // Pitch should return near equilibrium
    CHECK(std::abs(v.state().pitch - eqPitch) < 0.05f,
          "Pitch returns to near-level after full-width bump");

    // Roll should be near equilibrium (symmetric bump = no lasting roll)
    CHECK(std::abs(v.state().roll - eqRoll) < 0.05f,
          "No roll from symmetric full-width bump");
}

static void testLeftSideSpeedBump() {
    std::printf("  testLeftSideSpeedBump\n");
    auto v = makeVehicle();

    // Settle to equilibrium on flat ground
    SpeedBump noBump{ 999.f, 0.5f, 0.08f, -10.f, 0.f };
    InputState cruise{};
    cruise.throttle = 0.3f;
    v.mutableState().velocity = glm::vec3(0.f, 0.f, 10.f);
    simulateOverBump(v, cruise, 1.0f, noBump);
    float eqY     = v.state().position.y;
    float eqPitch = v.state().pitch;
    float eqRoll  = v.state().roll;
    v.mutableState().position = glm::vec3(0.f, eqY, 0.f);

    // Left-side only bump: xMin=-10, xMax=0 catches left wheels (x = -0.76)
    SpeedBump bump{ 5.f, 0.5f, 0.08f, -10.f, 0.f };

    // Drive over bump — track peak roll deviation
    BumpSimState bs;
    bs.baseRoll = eqRoll;
    bs.baseY    = eqY;
    simulateOverBump(v, cruise, 3.f, bump, &bs);

    CHECK(bs.peakRollDev > 0.005f,
          "Left-side bump induces roll");

    CHECK(v.state().position.z > 10.f,
          "Vehicle drove past the left-side bump");

    CHECK(std::abs(v.state().position.y - eqY) < 0.03f,
          "Body returns to equilibrium height after left-side bump");

    CHECK(std::abs(v.state().roll - eqRoll) < 0.05f,
          "Roll returns to near-level after left-side bump");

    CHECK(std::abs(v.state().pitch - eqPitch) < 0.05f,
          "Pitch returns to near-level after left-side bump");
}

static void testRightSideSpeedBump() {
    std::printf("  testRightSideSpeedBump\n");
    auto v = makeVehicle();

    // Settle to equilibrium on flat ground
    SpeedBump noBump{ 999.f, 0.5f, 0.08f, 0.f, 10.f };
    InputState cruise{};
    cruise.throttle = 0.3f;
    v.mutableState().velocity = glm::vec3(0.f, 0.f, 10.f);
    simulateOverBump(v, cruise, 1.0f, noBump);
    float eqY     = v.state().position.y;
    float eqPitch = v.state().pitch;
    float eqRoll  = v.state().roll;
    v.mutableState().position = glm::vec3(0.f, eqY, 0.f);

    // Right-side only bump: xMin=0, xMax=10 catches right wheels (x = +0.76)
    SpeedBump bump{ 5.f, 0.5f, 0.08f, 0.f, 10.f };

    // Drive over bump — track peak roll deviation
    BumpSimState bs;
    bs.baseRoll = eqRoll;
    bs.baseY    = eqY;
    simulateOverBump(v, cruise, 3.f, bump, &bs);

    CHECK(bs.peakRollDev > 0.005f,
          "Right-side bump induces roll");

    CHECK(v.state().position.z > 10.f,
          "Vehicle drove past the right-side bump");

    CHECK(std::abs(v.state().position.y - eqY) < 0.03f,
          "Body returns to equilibrium height after right-side bump");

    CHECK(std::abs(v.state().roll - eqRoll) < 0.05f,
          "Roll returns to near-level after right-side bump");

    CHECK(std::abs(v.state().pitch - eqPitch) < 0.05f,
          "Pitch returns to near-level after right-side bump");
}

static void testLeftAndRightBumpRollOpposite() {
    std::printf("  testLeftAndRightBumpRollOpposite\n");

    InputState cruise{};
    cruise.throttle = 0.3f;

    // Helper: settle, then drive over a half-width bump,
    // return the signed peak roll deviation from equilibrium.
    auto peakRollFromBump = [&](float xMin, float xMax) -> float {
        auto v = makeVehicle();
        SpeedBump noBump{ 999.f, 0.5f, 0.08f, xMin, xMax };
        v.mutableState().velocity = glm::vec3(0.f, 0.f, 10.f);
        simulateOverBump(v, cruise, 1.0f, noBump);
        float eqRoll = v.state().roll;
        v.mutableState().position = glm::vec3(0.f, v.state().position.y, 0.f);

        SpeedBump bump{ 5.f, 0.5f, 0.08f, xMin, xMax };

        // Track the signed peak roll (largest absolute deviation, preserving sign)
        float peakRoll = 0.f;
        BumpSimState bs;
        bs.baseRoll = eqRoll;

        // Run short sim to capture the peak
        static constexpr bool frontLeft[4]  = { true,  true,  false, false };
        static constexpr bool leftSide_[4]  = { true,  false, true,  false };
        float prevComp[4] = { EQ_COMPRESSION, EQ_COMPRESSION, EQ_COMPRESSION, EQ_COMPRESSION };
        for (float t = 0.f; t < 1.5f; t += DT) {
            const auto& st = v.state();
            SpringDamper* springs[4] = {
                v.frontSubframe()->left()->spring(),
                v.frontSubframe()->right()->spring(),
                v.rearSubframe()->left()->spring(),
                v.rearSubframe()->right()->spring()
            };
            for (int i = 0; i < 4; ++i) {
                glm::vec3 wLocal = wheelLocalPos(frontLeft[i], leftSide_[i]);
                glm::vec3 wWorld = st.position + st.bodyRotation * wLocal;
                float groundH = bump.heightAt(wWorld.x, wWorld.z);
                float comp = std::max(0.f, EQ_COMPRESSION + groundH - wWorld.y);
                float compVel = (comp - prevComp[i]) / DT;
                springs[i]->setCompression(comp, compVel);
                prevComp[i] = comp;
            }
            v.update(DT, cruise);
            if (v.state().position.y < 0.f && v.state().velocity.y < 0.f) {
                v.mutableState().position.y = 0.f;
                v.mutableState().velocity.y = 0.f;
            }
            float dev = v.state().roll - eqRoll;
            if (std::abs(dev) > std::abs(peakRoll)) peakRoll = dev;
        }
        return peakRoll;
    };

    float rollLeft  = peakRollFromBump(-10.f, 0.f);
    float rollRight = peakRollFromBump(0.f, 10.f);

    // Left and right bumps should produce roll in opposite directions
    CHECK(rollLeft * rollRight < 0.f,
          "Left and right bumps produce opposite roll directions");
}

// ========================= Drop tests =====================================

// Drop vehicle upside-down onto ground plane. Expect energy absorption, not bounce.
static void testDropUpsideDown() {
    std::printf("  testDropUpsideDown\n");
    auto v = makeVehicle();
    v.body()->setRestitution(0.1f);

    // Position above ground, upside-down (roll = pi)
    v.mutableState().position = glm::vec3(0.f, 3.f, 0.f);
    v.mutableState().roll = 3.14159f;
    v.mutableState().bodyRotation = rotationFromAngles(0.f, 0.f, 3.14159f);

    // Ground plane at y=0, normal pointing up
    PlaneObstacle ground{
        .point  = glm::vec3(0.f, 0.f, 0.f),
        .normal = glm::vec3(0.f, 1.f, 0.f)
    };

    InputState none{};

    // Simulate 2 seconds — enough to fall, hit, and settle
    simulateWithCollisions(v, none, 2.0f, { ground });

    // After hitting ground upside-down, vehicle should NOT bounce higher than drop height
    CHECK(v.state().position.y < 3.f,
          "Upside-down drop: doesn't bounce above drop height");

    // Speed should be mostly absorbed (not bouncing endlessly)
    float speed = glm::length(v.state().velocity);
    CHECK(speed < 5.f,
          "Upside-down drop: energy mostly absorbed after 2s");
}

// Drop vehicle nose-down at 45 degrees.
static void testDrop45DegPitch() {
    std::printf("  testDrop45DegPitch\n");
    auto v = makeVehicle();
    v.body()->setRestitution(0.1f);

    // 45 degrees nose down (negative pitch = nose down)
    float pitchAngle = -0.7854f;  // -pi/4
    v.mutableState().position = glm::vec3(0.f, 3.f, 0.f);
    v.mutableState().pitch = pitchAngle;
    v.mutableState().bodyRotation = rotationFromAngles(0.f, pitchAngle, 0.f);

    PlaneObstacle ground{
        .point  = glm::vec3(0.f, 0.f, 0.f),
        .normal = glm::vec3(0.f, 1.f, 0.f)
    };

    InputState none{};
    simulateWithCollisions(v, none, 2.0f, { ground });

    CHECK(v.state().position.y < 3.f,
          "45-deg pitch drop: doesn't bounce above drop height");

    float speed = glm::length(v.state().velocity);
    CHECK(speed < 5.f,
          "45-deg pitch drop: energy mostly absorbed after 2s");
}

// Drop vehicle rolled 45 degrees onto its side.
static void testDrop45DegRoll() {
    std::printf("  testDrop45DegRoll\n");
    auto v = makeVehicle();
    v.body()->setRestitution(0.1f);

    // 45 degrees roll (right side down)
    float rollAngle = 0.7854f;  // pi/4
    v.mutableState().position = glm::vec3(0.f, 3.f, 0.f);
    v.mutableState().roll = rollAngle;
    v.mutableState().bodyRotation = rotationFromAngles(0.f, 0.f, rollAngle);

    PlaneObstacle ground{
        .point  = glm::vec3(0.f, 0.f, 0.f),
        .normal = glm::vec3(0.f, 1.f, 0.f)
    };

    InputState none{};
    simulateWithCollisions(v, none, 2.0f, { ground });

    CHECK(v.state().position.y < 3.f,
          "45-deg roll drop: doesn't bounce above drop height");

    float speed = glm::length(v.state().velocity);
    CHECK(speed < 5.f,
          "45-deg roll drop: energy mostly absorbed after 2s");
}

// Drop vehicle pitched 90° nose-down (standing on front bumper).
static void testDrop90DegNoseDown() {
    std::printf("  testDrop90DegNoseDown\n");
    auto v = makeVehicle();
    v.body()->setRestitution(0.1f);

    // Pitched -90° (nose straight down)
    float pitchAngle = -1.5708f;  // -pi/2
    v.mutableState().position = glm::vec3(0.f, 4.f, 0.f);
    v.mutableState().pitch = pitchAngle;
    v.mutableState().bodyRotation = rotationFromAngles(0.f, pitchAngle, 0.f);

    PlaneObstacle ground{
        .point  = glm::vec3(0.f, 0.f, 0.f),
        .normal = glm::vec3(0.f, 1.f, 0.f)
    };

    InputState none{};
    simulateWithCollisions(v, none, 3.0f, { ground });

    CHECK(v.state().position.y < 4.f,
          "90-deg nose-down drop: doesn't bounce above drop height");

    float speed = glm::length(v.state().velocity);
    CHECK(speed < 5.f,
          "90-deg nose-down drop: energy mostly absorbed after 3s");
}

// Drop vehicle pitched +90° (standing on rear bumper, nose up).
static void testDrop90DegNoseUp() {
    std::printf("  testDrop90DegNoseUp\n");
    auto v = makeVehicle();
    v.body()->setRestitution(0.1f);

    // Pitched +90° (nose straight up, tail down)
    float pitchAngle = 1.5708f;  // +pi/2
    v.mutableState().position = glm::vec3(0.f, 4.f, 0.f);
    v.mutableState().pitch = pitchAngle;
    v.mutableState().bodyRotation = rotationFromAngles(0.f, pitchAngle, 0.f);

    PlaneObstacle ground{
        .point  = glm::vec3(0.f, 0.f, 0.f),
        .normal = glm::vec3(0.f, 1.f, 0.f)
    };

    InputState none{};
    simulateWithCollisions(v, none, 3.0f, { ground });

    CHECK(v.state().position.y < 4.f,
          "90-deg nose-up drop: doesn't bounce above drop height");

    float speed = glm::length(v.state().velocity);
    CHECK(speed < 5.f,
          "90-deg nose-up drop: energy mostly absorbed after 3s");
}

// ========================= Catch-ground spin tests =========================

// Vehicle moving forward at speed, pitched nose-down, catches ground.
// Should rotate and settle, NOT spin crazily.
static void testCatchGroundNoseDown() {
    std::printf("  testCatchGroundNoseDown\n");
    auto v = makeVehicle();

    // Moving forward at 15 m/s, pitched -15° (nose down), slightly above ground
    float pitchAngle = -0.26f;  // ~-15°
    v.mutableState().position = glm::vec3(0.f, 1.0f, 0.f);
    v.mutableState().velocity = glm::vec3(0.f, 0.f, 15.f);
    v.mutableState().pitch = pitchAngle;
    v.mutableState().bodyRotation = rotationFromAngles(0.f, pitchAngle, 0.f);

    PlaneObstacle ground{
        .point  = glm::vec3(0.f, 0.f, 0.f),
        .normal = glm::vec3(0.f, 1.f, 0.f)
    };

    InputState none{};

    // Track peak angular velocity — the key measure of "no crazy spin"
    float peakAngSpeed = 0.f;
    for (float t = 0.f; t < 3.f; t += DT) {
        std::vector<CollisionContact> allContacts;
        auto c = checkPlane(v, ground);
        allContacts.insert(allContacts.end(), c.begin(), c.end());
        v.applyCollisions(allContacts, DT);
        v.update(DT, none);
        float angSpeed = glm::length(v.state().angularVel);
        if (angSpeed > peakAngSpeed) peakAngSpeed = angSpeed;
    }

    CHECK(peakAngSpeed < 4.f * 3.14159f,
          "Catch-ground nose-down: peak angular velocity < 2 rev/s");
    CHECK(v.state().position.y < 5.f,
          "Catch-ground nose-down: not launched high");
}

// Vehicle moving forward, rolled to one side, catches ground on lower corner.
static void testCatchGroundRolled() {
    std::printf("  testCatchGroundRolled\n");
    auto v = makeVehicle();

    // Moving forward at 15 m/s, rolled 20° right, slightly above ground
    float rollAngle = 0.35f;  // ~20°
    v.mutableState().position = glm::vec3(0.f, 1.0f, 0.f);
    v.mutableState().velocity = glm::vec3(0.f, 0.f, 15.f);
    v.mutableState().roll = rollAngle;
    v.mutableState().bodyRotation = rotationFromAngles(0.f, 0.f, rollAngle);

    PlaneObstacle ground{
        .point  = glm::vec3(0.f, 0.f, 0.f),
        .normal = glm::vec3(0.f, 1.f, 0.f)
    };

    InputState none{};

    float peakAngSpeed = 0.f;
    for (float t = 0.f; t < 3.f; t += DT) {
        std::vector<CollisionContact> allContacts;
        auto c = checkPlane(v, ground);
        allContacts.insert(allContacts.end(), c.begin(), c.end());
        v.applyCollisions(allContacts, DT);
        v.update(DT, none);
        float angSpeed = glm::length(v.state().angularVel);
        if (angSpeed > peakAngSpeed) peakAngSpeed = angSpeed;
    }

    CHECK(peakAngSpeed < 4.f * 3.14159f,
          "Catch-ground rolled: peak angular velocity < 2 rev/s");
    CHECK(v.state().position.y < 5.f,
          "Catch-ground rolled: not launched high");
}

// Vehicle with existing angular velocity, single corner touching.
// Should NOT develop unbounded spin.
static void testSingleCornerContactBounded() {
    std::printf("  testSingleCornerContactBounded\n");
    auto v = makeVehicle();
    v.body()->setRestitution(0.1f);

    // Vehicle barely above ground with moderate angular velocity
    v.mutableState().position = glm::vec3(0.f, 0.8f, 0.f);
    v.mutableState().velocity = glm::vec3(0.f, -2.f, 10.f);
    v.mutableState().angularVel = glm::vec3(3.f, 0.f, 0.f);  // pitching
    float pitchAngle = -0.3f;
    v.mutableState().pitch = pitchAngle;
    v.mutableState().bodyRotation = rotationFromAngles(0.f, pitchAngle, 0.f);

    PlaneObstacle ground{
        .point  = glm::vec3(0.f, 0.f, 0.f),
        .normal = glm::vec3(0.f, 1.f, 0.f)
    };

    InputState none{};

    // Track peak angular velocity through simulation
    float peakAngSpeed = 0.f;
    for (float t = 0.f; t < 3.f; t += DT) {
        std::vector<CollisionContact> allContacts;
        auto c = checkPlane(v, ground);
        allContacts.insert(allContacts.end(), c.begin(), c.end());
        v.applyCollisions(allContacts, DT);
        v.update(DT, none);
        float angSpeed = glm::length(v.state().angularVel);
        if (angSpeed > peakAngSpeed) peakAngSpeed = angSpeed;
    }

    CHECK(peakAngSpeed < 4.f * 3.14159f,
          "Single-corner contact: peak angular velocity < 2 rev/s");
}

// ========================= High-centering =================================

// Vehicle on a raised platform between the wheels — wheels in air,
// body resting on the platform. Should not make forward progress under throttle.
static void testHighCentering() {
    std::printf("  testHighCentering\n");
    auto v = makeVehicle();

    // Platform: a horizontal plane at y=0.40 (above wheel bottom, below body).
    // Vehicle origin at y=0.45 (body resting on platform, wheels dangling).
    // Springs at zero compression (no ground contact for wheels).
    v.mutableState().position = glm::vec3(0.f, 0.45f, 0.f);

    PlaneObstacle platform{
        .point  = glm::vec3(0.f, 0.40f, 0.f),
        .normal = glm::vec3(0.f, 1.f, 0.f)
    };

    // Full throttle, no springs (wheels in air)
    InputState gas{};
    gas.throttle = 1.f;

    float startZ = v.state().position.z;

    // Simulate with collisions (platform holds body up) but no spring compression
    // (wheels hanging free, no traction)
    for (float t = 0.f; t < 2.f; t += DT) {
        std::vector<CollisionContact> contacts;
        auto c = checkPlane(v, platform);
        contacts.insert(contacts.end(), c.begin(), c.end());
        v.applyCollisions(contacts, DT);
        // No loadSprings — wheels have no ground contact
        v.update(DT, gas);
    }

    float travel = std::abs(v.state().position.z - startZ);
    CHECK(travel < 0.5f,
          "High-centered: minimal forward progress with wheels in air");
}

// ========================= Wall impact: no excessive vertical force ========

static void testWallImpactNoVerticalLaunch() {
    std::printf("  testWallImpactNoVerticalLaunch\n");
    auto v = makeVehicle();
    v.body()->setRestitution(0.3f);
    v.mutableState().velocity = glm::vec3(0.f, 0.f, 15.f);  // 15 m/s into wall

    PlaneObstacle wall{
        .point  = glm::vec3(0.f, 0.f, 2.5f),
        .normal = glm::vec3(0.f, 0.f, -1.f)
    };
    PlaneObstacle ground{
        .point  = glm::vec3(0.f, 0.f, 0.f),
        .normal = glm::vec3(0.f, 1.f, 0.f)
    };

    InputState none{};
    float peakVy = 0.f;
    for (float t = 0.f; t < 2.f; t += DT) {
        auto contacts = checkPlane(v, wall);
        auto gc = checkPlane(v, ground);
        contacts.insert(contacts.end(), gc.begin(), gc.end());
        v.applyCollisions(contacts, DT);
        v.update(DT, none);
        peakVy = std::max(peakVy, v.state().velocity.y);
    }

    std::printf("    peak upward velocity: %.2f m/s\n", peakVy);
    CHECK(peakVy < 3.f,
          "Wall impact doesn't launch vehicle upward");
}

// ========================= Resting on sides/ends/roof =====================

static void testRestOnSide() {
    std::printf("  testRestOnSide\n");
    auto v = makeVehicle();
    // Start on right side (roll = π/2), slightly above ground
    v.mutableState().position = glm::vec3(0.f, 0.8f, 0.f);
    v.mutableState().roll = 3.14159265f / 2.f;
    v.mutableState().bodyRotation = rotationFromAngles(0.f, 0.f, 3.14159265f / 2.f);

    PlaneObstacle ground{
        .point  = glm::vec3(0.f, 0.f, 0.f),
        .normal = glm::vec3(0.f, 1.f, 0.f)
    };

    InputState none{};
    simulateWithCollisions(v, none, 3.f, { ground });

    float speed = glm::length(v.state().velocity);
    float angSpeed = glm::length(v.state().angularVel);
    float y = v.state().position.y;
    std::printf("    speed=%.2f, angSpeed=%.2f, y=%.2f\n", speed, angSpeed, y);
    CHECK(speed < 3.f, "Vehicle on side: nearly stationary");
    // Side is unstable — car topples, so angular velocity is expected
    CHECK(angSpeed < 12.f, "Vehicle on side: bounded spin");
    CHECK(y < 1.5f, "Vehicle on side: not launched");
}

static void testRestOnRoof() {
    std::printf("  testRestOnRoof\n");
    auto v = makeVehicle();
    // Start upside down (roll = π), slightly above ground
    v.mutableState().position = glm::vec3(0.f, 1.0f, 0.f);
    v.mutableState().roll = 3.14159265f;
    v.mutableState().bodyRotation = rotationFromAngles(0.f, 0.f, 3.14159265f);

    PlaneObstacle ground{
        .point  = glm::vec3(0.f, 0.f, 0.f),
        .normal = glm::vec3(0.f, 1.f, 0.f)
    };

    InputState none{};
    simulateWithCollisions(v, none, 3.f, { ground });

    float speed = glm::length(v.state().velocity);
    float angSpeed = glm::length(v.state().angularVel);
    std::printf("    speed=%.2f, angSpeed=%.2f\n", speed, angSpeed);
    CHECK(speed < 3.f, "Vehicle on roof: nearly stationary");
    CHECK(angSpeed < 5.f, "Vehicle on roof: bounded spin");
}

static void testRestOnNose() {
    std::printf("  testRestOnNose\n");
    auto v = makeVehicle();
    // Nose down (pitch = -π/2), elevated
    v.mutableState().position = glm::vec3(0.f, 1.5f, 0.f);
    v.mutableState().pitch = -3.14159265f / 2.f;
    v.mutableState().bodyRotation = rotationFromAngles(0.f, -3.14159265f / 2.f, 0.f);

    PlaneObstacle ground{
        .point  = glm::vec3(0.f, 0.f, 0.f),
        .normal = glm::vec3(0.f, 1.f, 0.f)
    };

    InputState none{};
    simulateWithCollisions(v, none, 3.f, { ground });

    float speed = glm::length(v.state().velocity);
    float angSpeed = glm::length(v.state().angularVel);
    std::printf("    speed=%.2f, angSpeed=%.2f\n", speed, angSpeed);
    CHECK(speed < 3.f, "Vehicle on nose: settles");
    // Nose-down is unstable (CG above contact) — car topples forward
    CHECK(angSpeed < 12.f, "Vehicle on nose: bounded spin");
}

static void testRestOnTail() {
    std::printf("  testRestOnTail\n");
    auto v = makeVehicle();
    // Tail down (pitch = +π/2), elevated
    v.mutableState().position = glm::vec3(0.f, 1.5f, 0.f);
    v.mutableState().pitch = 3.14159265f / 2.f;
    v.mutableState().bodyRotation = rotationFromAngles(0.f, 3.14159265f / 2.f, 0.f);

    PlaneObstacle ground{
        .point  = glm::vec3(0.f, 0.f, 0.f),
        .normal = glm::vec3(0.f, 1.f, 0.f)
    };

    InputState none{};
    simulateWithCollisions(v, none, 3.f, { ground });

    float speed = glm::length(v.state().velocity);
    float angSpeed = glm::length(v.state().angularVel);
    std::printf("    speed=%.2f, angSpeed=%.2f\n", speed, angSpeed);
    CHECK(speed < 3.f, "Vehicle on tail: settles");
    CHECK(angSpeed < 5.f, "Vehicle on tail: bounded spin");
}

// ========================= Main ===========================================

int main() {
    std::printf("Running component system tests...\n\n");

    std::printf("Tree structure:\n");
    testComponentTreeInit();
    testFrontSubframeSteered();

    std::printf("Gravity / Mass:\n");
    testGravityDirectionIsDown();
    testMassReportsCorrectWeight();

    std::printf("Aero surfaces:\n");
    testDragOpposesForwardMotion();
    testDragOpposesReverseMotion();
    testDownforceIsDownward();
    testNoAeroAtZeroSpeed();
    testAeroIncreasesWithSpeed();

    std::printf("Spring / Damper:\n");
    testSpringCompressionProducesUpwardForce();
    testSpringNoTensionForce();
    testDamperAddsToSpringForce();

    std::printf("Tire:\n");
    testTireNoForceWithoutLoad();
    testTireThrottleProducesForwardForce();
    testTireBrakeOpposesMotion();
    testTireLateralForceFromSlip();
    testTireLateralForceOppositeSlipDirection();
    testFrontTireNotDriven();

    std::printf("Controls routing:\n");
    testControlsRoutedCorrectly();

    std::printf("Torque / Moment arms:\n");
    testOffsetForceProducesTorque();
    testLeftRightSymmetryOfSubframe();
    testAsymmetricCompressionProducesRollTorque();

    std::printf("Force propagation (integration):\n");
    testTireForceReachesVehicle();
    testBrakingSlowsVehicle();
    testSteeringProducesLateralForce();
    testGravityPullsVehicleDown();

    std::printf("Drawables:\n");
    testDrawablesNonEmpty();
    testDrawablesMergeCorrectly();

    std::printf("Equilibrium / Stopping:\n");
    testStationaryOnFlatGround();
    testDriveAndStopWithBrake();
    testHandBrakeLocksRearWheels();

    std::printf("Slope scenarios:\n");
    testSlipperySideHillSlides();
    testGrippySideHillRolls();
    testSlippyDownhillSlides();
    testGrippyUphillHoldsWithBrake();
    testSlippyUphillSlidesBack();
    testSurfaceGripAffectsTireForce();

    std::printf("Freefall:\n");
    testFreefallDropsWithoutSprings();
    testFreefallPitchesFromCGOffset();

    std::printf("Body dynamics:\n");
    testAccelerationCausesRearSquat();
    testCGOffsetCreatesPitchTorque();
    testCorneringProducesYaw();

    std::printf("Steer-then-stop:\n");
    testSteerThenStopNoResidualYaw();

    std::printf("Brake lock-up:\n");
    testBrakeLockUpEasierOnIce();

    std::printf("Spring bounce:\n");
    testGroundMovesUpCompressesSprings();
    testVehicleBounces();

    std::printf("Collisions:\n");
    testWallStopsVehicle();
    testCollisionAbsorbsEnergy();
    testHeadOnRampNoLaunch();
    testGlancingRampNoLaunch();
    testOverhangHeadOnRamp();
    testOverhangGlancingRamp();

    std::printf("Speed bump:\n");
    testFullWidthSpeedBump();
    testLeftSideSpeedBump();
    testRightSideSpeedBump();
    testLeftAndRightBumpRollOpposite();

    std::printf("Drop tests:\n");
    testDropUpsideDown();
    testDrop45DegPitch();
    testDrop45DegRoll();
    testDrop90DegNoseDown();
    testDrop90DegNoseUp();

    std::printf("Catch-ground spin:\n");
    testCatchGroundNoseDown();
    testCatchGroundRolled();
    testSingleCornerContactBounded();

    std::printf("Wall impact:\n");
    testWallImpactNoVerticalLaunch();

    std::printf("Resting orientations:\n");
    testRestOnSide();
    testRestOnRoof();
    testRestOnNose();
    testRestOnTail();

    std::printf("High-centering:\n");
    testHighCentering();

    std::printf("Reset:\n");
    testResetZerosState();

    return reportResults();
}
