#pragma once
#include <glm/glm.hpp>
#include "vehicle/VehicleState.h"

/// Chase camera that follows the car from behind and above.
/// Lerps toward a target position each frame for smooth lag.
/// Body roll from physics is reflected in the camera's up-vector for natural feel.
class Camera {
public:
    struct Params {
        float followDistanceM { 8.f  };  ///< Metres behind the car
        float followHeightM   { 2.5f };  ///< Metres above the car
        float fovYDeg         { 70.f };
        float nearPlane       { 0.1f };
        float farPlane        { 800.f };
        float lagFactor       { 0.08f }; ///< Lerp factor per frame (0=frozen, 1=instant)
        float rollScale       { 0.5f };  ///< Scale applied to physics roll for camera up-vector
    };

    explicit Camera(Params p = {}) : params_(p) {}

    /// Update camera position based on vehicle state and aspect ratio.
    void update(const VehicleState& car, float dt, float aspectRatio);

    [[nodiscard]] glm::mat4 viewMatrix()       const;
    [[nodiscard]] glm::mat4 projectionMatrix() const;
    [[nodiscard]] glm::vec3 position()         const { return smoothPos_; }

private:
    Params    params_;
    glm::vec3 smoothPos_    { 0.f, 5.f, -10.f };
    glm::vec3 smoothTarget_ { 0.f, 0.f,   0.f };
    float     aspectRatio_  { 16.f / 9.f };
    float     smoothRoll_   { 0.f };  ///< Smoothed body roll from physics (radians)
};
