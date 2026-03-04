#include "VulkanBuffer.h"
#include "core/Logging.h"

bool VulkanBuffer::upload(const VulkanContext& ctx, const void* data, VkDeviceSize dataSize, VkBufferUsageFlags usage)
{
    // Create staging buffer
    VkBuffer       stagingBuf;
    VkDeviceMemory stagingMem;
    if (!ctx.createBuffer(dataSize,
            VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
            VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
            stagingBuf, stagingMem))
    {
        LOG_ERROR("Failed to create staging buffer");
        return false;
    }

    void* mapped;
    vkMapMemory(ctx.device, stagingMem, 0, dataSize, 0, &mapped);
    std::memcpy(mapped, data, dataSize);
    vkUnmapMemory(ctx.device, stagingMem);

    // Destroy old buffer if re-uploading
    if (buffer != VK_NULL_HANDLE) destroy(ctx.device);

    if (!ctx.createBuffer(dataSize,
            usage | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
            VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
            buffer, memory))
    {
        vkDestroyBuffer(ctx.device, stagingBuf, nullptr);
        vkFreeMemory(ctx.device, stagingMem, nullptr);
        return false;
    }

    ctx.copyBuffer(stagingBuf, buffer, dataSize);
    size = dataSize;

    vkDestroyBuffer(ctx.device, stagingBuf, nullptr);
    vkFreeMemory(ctx.device, stagingMem, nullptr);
    return true;
}

void VulkanBuffer::destroy(VkDevice device)
{
    if (buffer != VK_NULL_HANDLE) vkDestroyBuffer(device, buffer, nullptr);
    if (memory != VK_NULL_HANDLE) vkFreeMemory(device, memory, nullptr);
    buffer = VK_NULL_HANDLE;
    memory = VK_NULL_HANDLE;
    size   = 0;
}
