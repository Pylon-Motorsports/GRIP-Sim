#pragma once
#include "IVehicleDynamics.h"
#include "VehicleParams.h"
#include <vector>
#include <glm/glm.hpp>

/// Semi-realistic bicycle model with weight transfer and linear tyre cornering.
/// Physics integrated at 120 Hz (called by Engine fixed step).
/// Replace with a full multi-body solver by binding a different IVehicleDynamics.
class SemiRealisticVehicle : public IVehicleDynamics {
public:
    explicit SemiRealisticVehicle(VehicleParams params = {});

    void reset(glm::vec3 position, float headingRad) override;
    void integrate(const InputFrame& input, float dt) override;
    [[nodiscard]] const VehicleState& state() const override { return state_; }

    /// Provide centreline points so the vehicle can track its segment.
    void setCenterlinePoints(const std::vector<glm::vec3>& points,
                             const std::vector<uint32_t>&  segmentStartVertex,
                             int verticesPerRow);

private:
    VehicleParams params_;
    VehicleState  state_;

    // Local velocity frame (body-fixed)
    float vx_ { 0.f };  ///< Forward velocity (m/s)
    float vy_ { 0.f };  ///< Lateral velocity (m/s, positive = right)
    float yawRate_ { 0.f }; ///< Yaw rate (rad/s, CCW positive)
    float prevSpeedMs_ { 0.f };

    // Geometry lookup for segment tracking
    std::vector<glm::vec3> centerline_;
    std::vector<uint32_t>  segmentStartVertex_;
    int                    vertsPerRow_ { 4 };

    void updateSegmentTracking();
    float engineTorque(float throttle, float rpm) const;
    void  autoShift();
};
