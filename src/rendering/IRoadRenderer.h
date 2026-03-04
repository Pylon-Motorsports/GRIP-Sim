#pragma once
#include <cstdint>
#include <glm/glm.hpp>
#include "RoadMesh.h"

struct SDL_Window;

/// Module interface for road rendering.
/// Swap implementations for different graphics APIs, headless testing, etc.
class IRoadRenderer {
public:
    virtual ~IRoadRenderer() = default;

    /// One-time initialization. Returns false on failure.
    virtual bool initialize(SDL_Window* window) = 0;

    /// Upload (or re-upload if mesh.dirty) vertex/index data to GPU.
    virtual void uploadMesh(const RoadMesh& mesh) = 0;

    /// Set the combined view-projection matrix before drawFrame().
    virtual void setViewProjection(const glm::mat4& view, const glm::mat4& proj) = 0;

    /// Set the car world transform for the debug wireframe box.
    virtual void setCarTransform(const glm::mat4& model) = 0;

    /// Record and submit one rendered frame.
    virtual void drawFrame() = 0;

    /// Handle window resize (recreate swapchain).
    virtual void onResize(uint32_t w, uint32_t h) = 0;

    /// Clean shutdown; release all GPU resources.
    virtual void shutdown() = 0;
};
