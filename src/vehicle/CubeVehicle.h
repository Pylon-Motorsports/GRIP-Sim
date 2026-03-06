#pragma once
#include "IVehicleDynamics.h"
#include "road/TerrainQuery.h"
#include "rendering/RoadMesh.h"
#include <glm/glm.hpp>
#include <vector>

/// Minimal cube vehicle: rigid body box that slides on the ground.
/// No wheels, no suspension, no drivetrain — just gravity, ground contact,
/// tree collision, and input-driven forces.
///
/// Used to validate the modular architecture and as a starting point for
/// building up physics incrementally.
class CubeVehicle : public IVehicleDynamics {
public:
    struct Params {
        float massKg       { 1200.f };
        float halfWidth    { 0.9f };    ///< Half-width (X)
        float halfLength   { 2.0f };    ///< Half-length (Z)
        float halfHeight   { 0.7f };    ///< Half-height (Y) — bottom to CG
        float maxForceN    { 8000.f };  ///< Throttle force
        float maxBrakeN    { 15000.f }; ///< Brake force
        float maxSteerRad  { 0.6f };    ///< Max yaw rate per unit steer (rad/s)
        float dragCoeff    { 0.5f };    ///< Simple velocity drag
        float frictionMu   { 0.8f };    ///< Ground friction coefficient
    };

    explicit CubeVehicle(Params params = {});

    void reset(glm::vec3 position, float headingRad) override;
    void integrate(const InputFrame& input, float dt) override;
    [[nodiscard]] const VehicleState& state() const override { return state_; }

    void setTerrainQuery(const TerrainQuery& terrain);
    void setTrees(const std::vector<TreeInstance>& trees);
    void setCenterlinePoints(const std::vector<glm::vec3>& points,
                             const std::vector<uint32_t>&  segmentStartVertex,
                             int verticesPerRow);

private:
    Params params_;
    VehicleState state_;
    TerrainQuery terrain_;
    std::vector<TreeInstance> trees_;

    glm::vec3 pos_  { 0.f };
    glm::vec3 vel_  { 0.f };
    float     yaw_  { 0.f };
    float     yawRate_ { 0.f };

    // Segment tracking
    std::vector<glm::vec3> centerline_;
    std::vector<uint32_t>  segmentStartVertex_;
    int                    vertsPerRow_ { 7 };

    void updateSegmentTracking();
};
