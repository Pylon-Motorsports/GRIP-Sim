#pragma once
#include <memory>
#include <typeindex>
#include <unordered_map>
#include <stdexcept>
#include <string>

/// Type-erased slot container for module interfaces.
/// Bind a concrete implementation to an interface type; retrieve by interface type.
/// This is the primary extension point: users swap modules by changing bindings in main().
class ModuleRegistry {
public:
    template<typename IFace>
    void bind(std::shared_ptr<IFace> impl) {
        slots_[typeid(IFace)] = std::static_pointer_cast<void>(impl);
    }

    template<typename IFace>
    [[nodiscard]] std::shared_ptr<IFace> get() const {
        auto it = slots_.find(typeid(IFace));
        if (it == slots_.end())
            throw std::runtime_error(std::string("Module not bound: ") + typeid(IFace).name());
        return std::static_pointer_cast<IFace>(it->second);
    }

    template<typename IFace>
    [[nodiscard]] bool has() const {
        return slots_.count(typeid(IFace)) > 0;
    }

private:
    std::unordered_map<std::type_index, std::shared_ptr<void>> slots_;
};
