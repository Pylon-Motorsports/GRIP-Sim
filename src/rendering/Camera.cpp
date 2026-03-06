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

    // Lerp camera position
    float lag     = glm::clamp(params_.lagFactor, 0.f, 1.f);
    smoothPos_    = glm::mix(smoothPos_,    targetPos,    lag);
    smoothTarget_ = glm::mix(smoothTarget_, targetLookAt, lag * 1.5f);

    // Smooth the body roll from physics so the camera tilts naturally into corners.
    float targetRoll = car.rollRad * params_.rollScale;
    smoothRoll_ = glm::mix(smoothRoll_, targetRoll, 0.08f);
}

glm::mat4 Camera::viewMatrix() const
{
    // Rotate the world-up vector by smoothRoll_ around the camera's forward axis
    // to achieve the lean-into-corner feel.
    glm::vec3 camForward = glm::normalize(smoothTarget_ - smoothPos_);
    glm::mat4 rollMat    = glm::rotate(glm::mat4(1.f), smoothRoll_, camForward);
    glm::vec3 rolledUp   = glm::vec3(rollMat * glm::vec4(0.f, 1.f, 0.f, 0.f));

    // glm::lookAt computes right = cross(forward, up), which gives -X for a Z-forward
    // world (camera behind car looking toward +Z). This mirrors the scene so that
    // world +X appears on screen left. Fix: use cross(up, forward) so world +X = screen right.
    glm::vec3 right = glm::normalize(glm::cross(rolledUp, camForward));
    glm::vec3 up    = glm::cross(camForward, right);

    // Build view matrix (column-major: mat[col][row])
    glm::mat4 view(1.f);
    view[0][0] =  right.x;       view[1][0] =  right.y;       view[2][0] =  right.z;
    view[0][1] =  up.x;          view[1][1] =  up.y;          view[2][1] =  up.z;
    view[0][2] = -camForward.x;  view[1][2] = -camForward.y;  view[2][2] = -camForward.z;
    view[3][0] = -glm::dot(right,      smoothPos_);
    view[3][1] = -glm::dot(up,         smoothPos_);
    view[3][2] =  glm::dot(camForward, smoothPos_);
    return view;
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
