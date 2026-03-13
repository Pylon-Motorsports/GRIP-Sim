#pragma once
#include <glm/glm.hpp>
#include <vector>
#include <memory>
#include <string>
#include <cstdint>

// Driver controls — shared across the tree, read only by components that care.
struct VehicleControls {
    float steerAngle { 0.f };   // radians (already scaled from -1..1)
    float throttle   { 0.f };   // 0..1
    float brake      { 0.f };   // 0..1
    float handBrake  { 0.f };   // 0..1
};

// State passed from parent to child during update.
// Pure physics context — no driver controls here.
struct ComponentInput {
    float     dt            { 0.f };
    glm::vec3 position      { 0.f };       // world position of attachment point
    glm::vec3 velocity      { 0.f };       // world velocity at attachment point
    glm::vec3 angularVel    { 0.f };       // body angular velocity (rad/s)
    glm::mat3 bodyRotation  { 1.f };       // body-local to world rotation
    float     normalLoad    { 0.f };       // vertical load pressing on this component (N)
    float     surfaceGrip   { 1.f };       // 0..1 surface friction multiplier
    glm::vec3 groundNormal  { 0.f, 1.f, 0.f }; // surface normal (world space)
};

// Forces/torques a component contributes back to its parent.
struct ComponentOutput {
    glm::vec3 force  { 0.f };   // world-space force (N)
    glm::vec3 torque { 0.f };   // world-space torque about parent attachment point (N*m)

    ComponentOutput& operator+=(const ComponentOutput& rhs);
};

ComponentOutput operator+(ComponentOutput a, const ComponentOutput& b);

// ---------------------------------------------------------------------------
// Drawable geometry a component contributes to the renderer.
// Matches the main project's Vertex {pos, normal, color} + index layout.
// ---------------------------------------------------------------------------
struct ComponentVertex {
    glm::vec3 pos;
    glm::vec3 normal;
    glm::vec4 color;
};

struct Drawable {
    std::vector<ComponentVertex> vertices;
    std::vector<uint32_t>        indices;

    bool empty() const { return vertices.empty(); }
    void merge(const Drawable& other);
};

// ---------------------------------------------------------------------------
// Base class for all physics components.
//
// Composite pattern: a component can own sub-components. The default update()
// fans out to children and sums their outputs. Leaf components override
// compute() to produce their own forces.
// ---------------------------------------------------------------------------
class VehiclePhysicsComponent {
public:
    virtual ~VehiclePhysicsComponent() = default;

    explicit VehiclePhysicsComponent(std::string name,
                                     glm::vec3 attachmentPoint = glm::vec3{0.f});

    // --- accessors ----------------------------------------------------------
    const std::string& name()            const { return name_; }
    glm::vec3 attachmentPoint()          const { return attachmentPoint_; }
    void setAttachmentPoint(glm::vec3 p)       { attachmentPoint_ = p; }
    const ComponentOutput& lastOutput()  const { return lastOutput_; }

    // --- controls -----------------------------------------------------------
    void setControls(const VehicleControls* ctrl);
    const VehicleControls* controls() const { return controls_; }

    // --- init ---------------------------------------------------------------
    virtual void init();

    // --- child access -------------------------------------------------------
    // Template must remain in header
    template <typename T>
    T* findChild(const std::string& childName) {
        for (auto& c : children_)
            if (c->name() == childName) return dynamic_cast<T*>(c.get());
        return nullptr;
    }

    const std::vector<std::unique_ptr<VehiclePhysicsComponent>>& children() const;

    // --- update -------------------------------------------------------------
    virtual ComponentOutput update(const ComponentInput& input);

    // --- drawables ----------------------------------------------------------
    Drawable collectDrawables(const ComponentInput& input) const;

    // --- reset --------------------------------------------------------------
    virtual void reset();

protected:
    void addChild(std::unique_ptr<VehiclePhysicsComponent> child);

    virtual ComponentOutput compute(const ComponentInput& input);
    virtual Drawable generateDrawable(const ComponentInput& input) const;
    virtual ComponentInput makeChildInput(const ComponentInput& parentInput,
                                          const VehiclePhysicsComponent& child);

    std::string name_;
    glm::vec3   attachmentPoint_;
    ComponentOutput lastOutput_;
    std::vector<std::unique_ptr<VehiclePhysicsComponent>> children_;
    const VehicleControls* controls_ = nullptr;
};
