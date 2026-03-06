#pragma once
#include "IVehicleDynamics.h"
#include "MultiBodyParams.h"
#include "road/TerrainQuery.h"
#include "rendering/RoadMesh.h"
#include <glm/glm.hpp>
#include <array>
#include <vector>

/// Full multi-body vehicle: rigid body + 4 independent suspension corners + 4 wheels.
///
/// Physics hierarchy:
///   Ground → tyre contact forces → wheel → suspension spring/damper → body (rigid body 6-DOF).
///
/// Coordinate system:
///   World: X=right, Y=up, Z=forward (when heading=0)
///   headingRad increases CW (turning right is positive) to match rally convention.
///
class MultiBodyVehicle : public IVehicleDynamics {
public:
    /// Corner indices.
    enum Corner { FL = 0, FR = 1, RL = 2, RR = 3 };

    explicit MultiBodyVehicle(MultiBodyParams params = {});

    void reset(glm::vec3 position, float headingRad) override;
    void integrate(const InputFrame& input, float dt) override;
    [[nodiscard]] const VehicleState& state() const override { return state_; }

    /// Provide centreline points for segment tracking (same API as SemiRealisticVehicle).
    void setCenterlinePoints(const std::vector<glm::vec3>& points,
                             const std::vector<uint32_t>&  segmentStartVertex,
                             int verticesPerRow);

    /// Provide terrain data for per-wheel ground height queries.
    void setTerrainQuery(const TerrainQuery& terrain);

    /// Provide tree collision cylinders (from RoadBuilder tree placement).
    void setTrees(const std::vector<TreeInstance>& trees);

private:
    // -----------------------------------------------------------------------
    // Internal state
    // -----------------------------------------------------------------------

    struct WheelState {
        float steerAngle  { 0.f };   ///< Current steering angle (rad), front wheels only
        float spinRate    { 0.f };   ///< Wheel angular velocity (rad/s)
        float normalForce { 0.f };   ///< Normal force from suspension (N), updated each step
        float Fx          { 0.f };   ///< Longitudinal tyre force in world (N)
        float Fy          { 0.f };   ///< Lateral tyre force in world (N)
        float alphaFiltered { 0.f }; ///< Relaxation-length filtered slip angle (rad)
        bool  inContact   { true };  ///< Whether wheel is touching the ground
    };

    struct SuspState {
        float currentLength { 0.f }; ///< Current spring length (m)
        float prevLength    { 0.f }; ///< Previous length (for damper velocity)
        float normalForce   { 0.f }; ///< Computed suspension normal force (N)
    };

    // Body rigid-body state
    glm::vec3 pos_    { 0.f, 0.f, 0.f }; ///< CG world position
    glm::vec3 vel_    { 0.f, 0.f, 0.f }; ///< CG world velocity
    float     yaw_    { 0.f };            ///< Heading (rad), CW = positive
    float     roll_   { 0.f };            ///< Body roll  (rad), right-side-down = positive
    float     pitch_  { 0.f };            ///< Body pitch (rad), nose-down = positive
    float     yawRate_  { 0.f };          ///< rad/s
    float     rollRate_ { 0.f };          ///< rad/s
    float     pitchRate_{ 0.f };          ///< rad/s

    // Per-wheel state
    std::array<WheelState, 4> wheels_ {};
    std::array<SuspState,  4> susp_   {};

    // Engine / transmission
    float engineRpm_   { 800.f };
    int   currentGear_ { 1 };

    // Segment tracking
    std::vector<glm::vec3> centerline_;
    std::vector<uint32_t>  segmentStartVertex_;
    int                    vertsPerRow_ { 4 };

    MultiBodyParams params_;
    VehicleState    state_;
    TerrainQuery    terrain_;
    float           wheelGroundHeight_[4] {};
    std::vector<TreeInstance> trees_;

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    /// Build body→world rotation matrix from current yaw/roll/pitch.
    glm::mat3 bodyToWorld() const;

    /// Body-frame position of each suspension attachment point (spring top).
    glm::vec3 cornerBodyPos(int c) const;

    /// Spring rest-length minus static deflection (used to init suspension).
    float staticSpringLength(int c) const;

    /// Engine torque from throttle + RPM.
    float engineTorque(float throttle, float rpm) const;

    void autoShift();
    void updateSegmentTracking();
};
