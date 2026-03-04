#pragma once
#include <vulkan/vulkan.h>
#include <SDL2/SDL.h>
#include <vector>
#include <cstdint>

/// Owns the Vulkan instance, physical/logical device, surface, swapchain,
/// command pools, and sync objects. Everything needed to render frames.
/// VulkanRoadRenderer holds a reference to VulkanContext.
struct VulkanContext {
    static constexpr int MAX_FRAMES_IN_FLIGHT = 2;

    // Core objects
    VkInstance       instance       { VK_NULL_HANDLE };
    VkSurfaceKHR     surface        { VK_NULL_HANDLE };
    VkPhysicalDevice physDevice     { VK_NULL_HANDLE };
    VkDevice         device         { VK_NULL_HANDLE };

    uint32_t graphicsFamily { UINT32_MAX };
    uint32_t presentFamily  { UINT32_MAX };
    VkQueue  graphicsQueue  { VK_NULL_HANDLE };
    VkQueue  presentQueue   { VK_NULL_HANDLE };

    // Swapchain
    VkSwapchainKHR           swapchain       { VK_NULL_HANDLE };
    VkFormat                 swapFormat      { VK_FORMAT_UNDEFINED };
    VkExtent2D               swapExtent      { 0, 0 };
    std::vector<VkImage>     swapImages;
    std::vector<VkImageView> swapViews;

    // Depth buffer
    VkImage        depthImage       { VK_NULL_HANDLE };
    VkDeviceMemory depthMemory      { VK_NULL_HANDLE };
    VkImageView    depthView        { VK_NULL_HANDLE };

    // Render pass & framebuffers
    VkRenderPass               renderPass   { VK_NULL_HANDLE };
    std::vector<VkFramebuffer> framebuffers;

    // Command pool & buffers (one per frame in flight)
    VkCommandPool                commandPool   { VK_NULL_HANDLE };
    std::vector<VkCommandBuffer> commandBuffers;

    // Sync objects
    std::vector<VkSemaphore> imageAvailable;
    std::vector<VkSemaphore> renderFinished;
    std::vector<VkFence>     inFlight;

    // Debug messenger (debug builds only)
    VkDebugUtilsMessengerEXT debugMessenger { VK_NULL_HANDLE };

    // Initialise everything; returns false on failure.
    bool initialize(SDL_Window* window);

    // Recreate swapchain after resize.
    bool recreateSwapchain(SDL_Window* window);

    // Clean shutdown.
    void shutdown();

    // Helper: find memory type index.
    uint32_t findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags props) const;

    // Helper: create a buffer + backing memory.
    bool createBuffer(VkDeviceSize size, VkBufferUsageFlags usage,
                      VkMemoryPropertyFlags props,
                      VkBuffer& buf, VkDeviceMemory& mem) const;

    // Helper: single-shot command buffer.
    VkCommandBuffer beginSingleTimeCommands() const;
    void            endSingleTimeCommands(VkCommandBuffer cmd) const;

    // Helper: copy buffer on GPU.
    void copyBuffer(VkBuffer src, VkBuffer dst, VkDeviceSize size) const;

private:
    bool createInstance(SDL_Window* window);
    bool setupDebugMessenger();
    bool createSurface(SDL_Window* window);
    bool pickPhysicalDevice();
    bool createLogicalDevice();
    bool createSwapchain(SDL_Window* window);
    bool createImageViews();
    bool createDepthResources();
    bool createRenderPass();
    bool createFramebuffers();
    bool createCommandPool();
    bool createCommandBuffers();
    bool createSyncObjects();
    void cleanupSwapchain();
};
