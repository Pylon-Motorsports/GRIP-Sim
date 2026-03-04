#pragma once
#include "VulkanContext.h"

/// RAII wrapper for a device-local GPU buffer backed by a staging buffer.
/// Handles upload of vertex and index data.
struct VulkanBuffer {
    VkBuffer       buffer { VK_NULL_HANDLE };
    VkDeviceMemory memory { VK_NULL_HANDLE };
    VkDeviceSize   size   { 0 };

    /// Upload data to a device-local buffer via a staging buffer.
    bool upload(const VulkanContext& ctx, const void* data, VkDeviceSize dataSize, VkBufferUsageFlags usage);

    void destroy(VkDevice device);
};
