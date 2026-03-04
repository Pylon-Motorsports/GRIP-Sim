#include "Camera.h"
#include <glm/gtc/matrix_transform.hpp>
#include <cmath>

void Camera::update(const VehicleState& car, float /*dt*/, float aspectRatio)
{
    aspectRatio_ = aspectRatio;

    float fwd_x = std::sin(car.headingRad);
    float fwd_z = std::cos(car.headingRad);
    glm::vec3 forward { fwd_x, 0.f, fwd_z };

    // Target camera position: behind + above the car
    glm::vec3 targetPos    = car.position
                           - forward * params_.followDistanceM
                           + glm::vec3(0.f, params_.followHeightM, 0.f);
    glm::vec3 targetLookAt = car.position + glm::vec3(0.f, 0.5f, 0.f);

    // Lerp for smooth lag
    float lag = glm::clamp(params_.lagFactor, 0.f, 1.f);
    smoothPos_    = glm::mix(smoothPos_,    targetPos,    lag);
    smoothTarget_ = glm::mix(smoothTarget_, targetLookAt, lag * 1.5f);
}

glm::mat4 Camera::viewMatrix() const
{
    return glm::lookAt(smoothPos_, smoothTarget_, glm::vec3(0.f, 1.f, 0.f));
}

glm::mat4 Camera::projectionMatrix() const
{
    auto proj = glm::perspective(glm::radians(params_.fovYDeg),
                                 aspectRatio_,
                                 params_.nearPlane,
                                 params_.farPlane);
    // Vulkan clip space: flip Y
    proj[1][1] *= -1.f;
    return proj;
}
