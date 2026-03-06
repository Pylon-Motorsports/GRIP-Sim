#pragma once
#include <vulkan/vulkan.h>
#include <SDL2/SDL.h>
#include <vector>
#include <cstdint>

struct VulkanContext {
    VkInstance       instance       = VK_NULL_HANDLE;
    VkSurfaceKHR     surface        = VK_NULL_HANDLE;
    VkPhysicalDevice physicalDevice = VK_NULL_HANDLE;
    VkDevice         device         = VK_NULL_HANDLE;
    VkQueue          graphicsQueue  = VK_NULL_HANDLE;
    VkQueue          presentQueue   = VK_NULL_HANDLE;
    uint32_t         graphicsFamily = 0;
    uint32_t         presentFamily  = 0;

    VkSwapchainKHR              swapchain = VK_NULL_HANDLE;
    VkFormat                    swapFormat{};
    VkExtent2D                  swapExtent{};
    std::vector<VkImage>        swapImages;
    std::vector<VkImageView>    swapViews;

    VkFormat       depthFormat = VK_FORMAT_D32_SFLOAT;
    VkImage        depthImage  = VK_NULL_HANDLE;
    VkDeviceMemory depthMem    = VK_NULL_HANDLE;
    VkImageView    depthView   = VK_NULL_HANDLE;

    VkRenderPass                renderPass = VK_NULL_HANDLE;
    std::vector<VkFramebuffer>  framebuffers;

    VkCommandPool                commandPool = VK_NULL_HANDLE;
    std::vector<VkCommandBuffer> cmdBufs;

    // One set of sync objects per swapchain image
    std::vector<VkSemaphore> imageReady;
    std::vector<VkSemaphore> renderDone;
    std::vector<VkFence>     inFlight;
    uint32_t frame  = 0;
    uint32_t imgIdx = 0;

    bool            init(SDL_Window* window);
    VkCommandBuffer beginFrame();
    void            endFrame();
    void            shutdown();

    uint32_t findMemType(uint32_t filter, VkMemoryPropertyFlags props) const;
    VkBuffer allocBuffer(VkDeviceSize size, VkBufferUsageFlags usage,
                         VkMemoryPropertyFlags props, VkDeviceMemory& mem) const;
};
