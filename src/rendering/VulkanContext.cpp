#include "VulkanContext.h"
#include "core/Logging.h"
#include <SDL2/SDL_vulkan.h>
#include <vector>
#include <set>
#include <string>
#include <algorithm>
#include <cstring>

// ---------------------------------------------------------------------------
// Validation layer config
// ---------------------------------------------------------------------------
#ifdef NDEBUG
static constexpr bool ENABLE_VALIDATION = false;
#else
static constexpr bool ENABLE_VALIDATION = true;
#endif

static const std::vector<const char*> VALIDATION_LAYERS = {
    "VK_LAYER_KHRONOS_validation"
};

static const std::vector<const char*> DEVICE_EXTENSIONS = {
    VK_KHR_SWAPCHAIN_EXTENSION_NAME
};

// ---------------------------------------------------------------------------
// Debug messenger
// ---------------------------------------------------------------------------
static VKAPI_ATTR VkBool32 VKAPI_CALL debugCallback(
    VkDebugUtilsMessageSeverityFlagBitsEXT severity,
    VkDebugUtilsMessageTypeFlagsEXT,
    const VkDebugUtilsMessengerCallbackDataEXT* data,
    void*)
{
    if (severity >= VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT)
        LOG_WARN("[Vulkan] %s", data->pMessage);
    return VK_FALSE;
}

static VkResult createDebugUtilsMessengerEXT(
    VkInstance instance,
    const VkDebugUtilsMessengerCreateInfoEXT* pCreateInfo,
    const VkAllocationCallbacks* pAllocator,
    VkDebugUtilsMessengerEXT* pDebugMessenger)
{
    auto func = (PFN_vkCreateDebugUtilsMessengerEXT)
        vkGetInstanceProcAddr(instance, "vkCreateDebugUtilsMessengerEXT");
    return func ? func(instance, pCreateInfo, pAllocator, pDebugMessenger)
                : VK_ERROR_EXTENSION_NOT_PRESENT;
}

static void destroyDebugUtilsMessengerEXT(
    VkInstance instance,
    VkDebugUtilsMessengerEXT messenger,
    const VkAllocationCallbacks* pAllocator)
{
    auto func = (PFN_vkDestroyDebugUtilsMessengerEXT)
        vkGetInstanceProcAddr(instance, "vkDestroyDebugUtilsMessengerEXT");
    if (func) func(instance, messenger, pAllocator);
}

// ---------------------------------------------------------------------------
// initialize
// ---------------------------------------------------------------------------
bool VulkanContext::initialize(SDL_Window* window)
{
    return createInstance(window)
        && setupDebugMessenger()
        && createSurface(window)
        && pickPhysicalDevice()
        && createLogicalDevice()
        && createSwapchain(window)
        && createImageViews()
        && createDepthResources()
        && createRenderPass()
        && createFramebuffers()
        && createCommandPool()
        && createCommandBuffers()
        && createSyncObjects();
}

// ---------------------------------------------------------------------------
// createInstance
// ---------------------------------------------------------------------------
bool VulkanContext::createInstance(SDL_Window* window)
{
    // Gather SDL-required extensions
    uint32_t extCount = 0;
    SDL_Vulkan_GetInstanceExtensions(window, &extCount, nullptr);
    std::vector<const char*> extensions(extCount);
    SDL_Vulkan_GetInstanceExtensions(window, &extCount, extensions.data());

    if (ENABLE_VALIDATION)
        extensions.push_back(VK_EXT_DEBUG_UTILS_EXTENSION_NAME);

    VkApplicationInfo appInfo{};
    appInfo.sType              = VK_STRUCTURE_TYPE_APPLICATION_INFO;
    appInfo.pApplicationName   = "GRIP-Sim";
    appInfo.applicationVersion = VK_MAKE_VERSION(0, 1, 0);
    appInfo.pEngineName        = "GRIP-Sim Engine";
    appInfo.engineVersion      = VK_MAKE_VERSION(0, 1, 0);
    appInfo.apiVersion         = VK_API_VERSION_1_2;

    VkInstanceCreateInfo ci{};
    ci.sType                   = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
    ci.pApplicationInfo        = &appInfo;
    ci.enabledExtensionCount   = static_cast<uint32_t>(extensions.size());
    ci.ppEnabledExtensionNames = extensions.data();

    if (ENABLE_VALIDATION) {
        ci.enabledLayerCount   = static_cast<uint32_t>(VALIDATION_LAYERS.size());
        ci.ppEnabledLayerNames = VALIDATION_LAYERS.data();
    }

    if (vkCreateInstance(&ci, nullptr, &instance) != VK_SUCCESS) {
        LOG_ERROR("Failed to create Vulkan instance");
        return false;
    }
    LOG_INFO("Vulkan instance created");
    return true;
}

// ---------------------------------------------------------------------------
// setupDebugMessenger
// ---------------------------------------------------------------------------
bool VulkanContext::setupDebugMessenger()
{
    if (!ENABLE_VALIDATION) return true;

    VkDebugUtilsMessengerCreateInfoEXT ci{};
    ci.sType           = VK_STRUCTURE_TYPE_DEBUG_UTILS_MESSENGER_CREATE_INFO_EXT;
    ci.messageSeverity = VK_DEBUG_UTILS_MESSAGE_SEVERITY_WARNING_BIT_EXT
                       | VK_DEBUG_UTILS_MESSAGE_SEVERITY_ERROR_BIT_EXT;
    ci.messageType     = VK_DEBUG_UTILS_MESSAGE_TYPE_GENERAL_BIT_EXT
                       | VK_DEBUG_UTILS_MESSAGE_TYPE_VALIDATION_BIT_EXT
                       | VK_DEBUG_UTILS_MESSAGE_TYPE_PERFORMANCE_BIT_EXT;
    ci.pfnUserCallback = debugCallback;

    if (createDebugUtilsMessengerEXT(instance, &ci, nullptr, &debugMessenger) != VK_SUCCESS) {
        LOG_WARN("Failed to set up debug messenger");
    }
    return true;
}

// ---------------------------------------------------------------------------
// createSurface
// ---------------------------------------------------------------------------
bool VulkanContext::createSurface(SDL_Window* window)
{
    if (!SDL_Vulkan_CreateSurface(window, instance, &surface)) {
        LOG_ERROR("SDL_Vulkan_CreateSurface failed: %s", SDL_GetError());
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// pickPhysicalDevice
// ---------------------------------------------------------------------------
static bool deviceSuitable(VkPhysicalDevice dev, VkSurfaceKHR surface,
                            uint32_t& gfxFamily, uint32_t& presFamily)
{
    uint32_t qCount = 0;
    vkGetPhysicalDeviceQueueFamilyProperties(dev, &qCount, nullptr);
    std::vector<VkQueueFamilyProperties> queues(qCount);
    vkGetPhysicalDeviceQueueFamilyProperties(dev, &qCount, queues.data());

    gfxFamily = presFamily = UINT32_MAX;
    for (uint32_t i = 0; i < qCount; ++i) {
        if (queues[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) gfxFamily = i;
        VkBool32 present = VK_FALSE;
        vkGetPhysicalDeviceSurfaceSupportKHR(dev, i, surface, &present);
        if (present) presFamily = i;
        if (gfxFamily != UINT32_MAX && presFamily != UINT32_MAX) break;
    }
    if (gfxFamily == UINT32_MAX || presFamily == UINT32_MAX) return false;

    // Check extension support
    uint32_t extCount = 0;
    vkEnumerateDeviceExtensionProperties(dev, nullptr, &extCount, nullptr);
    std::vector<VkExtensionProperties> avail(extCount);
    vkEnumerateDeviceExtensionProperties(dev, nullptr, &extCount, avail.data());
    for (const char* required : DEVICE_EXTENSIONS) {
        bool found = false;
        for (auto& e : avail) if (std::strcmp(e.extensionName, required) == 0) { found = true; break; }
        if (!found) return false;
    }

    // Check swapchain support
    uint32_t fmtCount = 0, modeCount = 0;
    vkGetPhysicalDeviceSurfaceFormatsKHR(dev, surface, &fmtCount, nullptr);
    vkGetPhysicalDeviceSurfacePresentModesKHR(dev, surface, &modeCount, nullptr);
    return fmtCount > 0 && modeCount > 0;
}

bool VulkanContext::pickPhysicalDevice()
{
    uint32_t count = 0;
    vkEnumeratePhysicalDevices(instance, &count, nullptr);
    if (count == 0) { LOG_ERROR("No Vulkan-capable GPU found"); return false; }

    std::vector<VkPhysicalDevice> devices(count);
    vkEnumeratePhysicalDevices(instance, &count, devices.data());

    for (auto dev : devices) {
        VkPhysicalDeviceProperties props;
        vkGetPhysicalDeviceProperties(dev, &props);
        uint32_t gfx = UINT32_MAX, pres = UINT32_MAX;
        if (deviceSuitable(dev, surface, gfx, pres)) {
            // Prefer discrete GPU
            if (physDevice == VK_NULL_HANDLE ||
                props.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU) {
                physDevice     = dev;
                graphicsFamily = gfx;
                presentFamily  = pres;
            }
        }
    }
    if (physDevice == VK_NULL_HANDLE) { LOG_ERROR("No suitable GPU found"); return false; }

    VkPhysicalDeviceProperties props;
    vkGetPhysicalDeviceProperties(physDevice, &props);
    LOG_INFO("Using GPU: %s", props.deviceName);
    return true;
}

// ---------------------------------------------------------------------------
// createLogicalDevice
// ---------------------------------------------------------------------------
bool VulkanContext::createLogicalDevice()
{
    std::set<uint32_t> uniqueFamilies = { graphicsFamily, presentFamily };
    std::vector<VkDeviceQueueCreateInfo> queueCIs;
    float priority = 1.f;
    for (uint32_t fam : uniqueFamilies) {
        VkDeviceQueueCreateInfo qi{};
        qi.sType            = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
        qi.queueFamilyIndex = fam;
        qi.queueCount       = 1;
        qi.pQueuePriorities = &priority;
        queueCIs.push_back(qi);
    }

    VkPhysicalDeviceFeatures features{};

    VkDeviceCreateInfo ci{};
    ci.sType                   = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
    ci.queueCreateInfoCount    = static_cast<uint32_t>(queueCIs.size());
    ci.pQueueCreateInfos       = queueCIs.data();
    ci.enabledExtensionCount   = static_cast<uint32_t>(DEVICE_EXTENSIONS.size());
    ci.ppEnabledExtensionNames = DEVICE_EXTENSIONS.data();
    ci.pEnabledFeatures        = &features;
    if (ENABLE_VALIDATION) {
        ci.enabledLayerCount   = static_cast<uint32_t>(VALIDATION_LAYERS.size());
        ci.ppEnabledLayerNames = VALIDATION_LAYERS.data();
    }

    if (vkCreateDevice(physDevice, &ci, nullptr, &device) != VK_SUCCESS) {
        LOG_ERROR("Failed to create logical device");
        return false;
    }
    vkGetDeviceQueue(device, graphicsFamily, 0, &graphicsQueue);
    vkGetDeviceQueue(device, presentFamily,  0, &presentQueue);
    return true;
}

// ---------------------------------------------------------------------------
// createSwapchain
// ---------------------------------------------------------------------------
bool VulkanContext::createSwapchain(SDL_Window* window)
{
    // Surface capabilities
    VkSurfaceCapabilitiesKHR caps;
    vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physDevice, surface, &caps);

    // Format: prefer BGRA8 SRGB
    uint32_t fmtCount = 0;
    vkGetPhysicalDeviceSurfaceFormatsKHR(physDevice, surface, &fmtCount, nullptr);
    std::vector<VkSurfaceFormatKHR> formats(fmtCount);
    vkGetPhysicalDeviceSurfaceFormatsKHR(physDevice, surface, &fmtCount, formats.data());
    VkSurfaceFormatKHR chosen = formats[0];
    for (auto& f : formats) {
        if (f.format == VK_FORMAT_B8G8R8A8_SRGB &&
            f.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) { chosen = f; break; }
    }
    swapFormat = chosen.format;

    // Present mode: prefer mailbox, fallback FIFO
    uint32_t modeCount = 0;
    vkGetPhysicalDeviceSurfacePresentModesKHR(physDevice, surface, &modeCount, nullptr);
    std::vector<VkPresentModeKHR> modes(modeCount);
    vkGetPhysicalDeviceSurfacePresentModesKHR(physDevice, surface, &modeCount, modes.data());
    VkPresentModeKHR presentMode = VK_PRESENT_MODE_FIFO_KHR;
    for (auto m : modes) if (m == VK_PRESENT_MODE_MAILBOX_KHR) { presentMode = m; break; }

    // Extent
    if (caps.currentExtent.width != UINT32_MAX) {
        swapExtent = caps.currentExtent;
    } else {
        int w, h;
        SDL_Vulkan_GetDrawableSize(window, &w, &h);
        swapExtent.width  = std::clamp((uint32_t)w, caps.minImageExtent.width,  caps.maxImageExtent.width);
        swapExtent.height = std::clamp((uint32_t)h, caps.minImageExtent.height, caps.maxImageExtent.height);
    }

    uint32_t imgCount = caps.minImageCount + 1;
    if (caps.maxImageCount > 0) imgCount = std::min(imgCount, caps.maxImageCount);

    VkSwapchainCreateInfoKHR ci{};
    ci.sType            = VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR;
    ci.surface          = surface;
    ci.minImageCount    = imgCount;
    ci.imageFormat      = chosen.format;
    ci.imageColorSpace  = chosen.colorSpace;
    ci.imageExtent      = swapExtent;
    ci.imageArrayLayers = 1;
    ci.imageUsage       = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
    ci.preTransform     = caps.currentTransform;
    ci.compositeAlpha   = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
    ci.presentMode      = presentMode;
    ci.clipped          = VK_TRUE;

    uint32_t queueFamilies[] = { graphicsFamily, presentFamily };
    if (graphicsFamily != presentFamily) {
        ci.imageSharingMode      = VK_SHARING_MODE_CONCURRENT;
        ci.queueFamilyIndexCount = 2;
        ci.pQueueFamilyIndices   = queueFamilies;
    } else {
        ci.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
    }

    if (vkCreateSwapchainKHR(device, &ci, nullptr, &swapchain) != VK_SUCCESS) {
        LOG_ERROR("Failed to create swapchain");
        return false;
    }

    uint32_t count = 0;
    vkGetSwapchainImagesKHR(device, swapchain, &count, nullptr);
    swapImages.resize(count);
    vkGetSwapchainImagesKHR(device, swapchain, &count, swapImages.data());
    return true;
}

// ---------------------------------------------------------------------------
// createImageViews
// ---------------------------------------------------------------------------
bool VulkanContext::createImageViews()
{
    swapViews.resize(swapImages.size());
    for (size_t i = 0; i < swapImages.size(); ++i) {
        VkImageViewCreateInfo ci{};
        ci.sType                           = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
        ci.image                           = swapImages[i];
        ci.viewType                        = VK_IMAGE_VIEW_TYPE_2D;
        ci.format                          = swapFormat;
        ci.subresourceRange.aspectMask     = VK_IMAGE_ASPECT_COLOR_BIT;
        ci.subresourceRange.baseMipLevel   = 0;
        ci.subresourceRange.levelCount     = 1;
        ci.subresourceRange.baseArrayLayer = 0;
        ci.subresourceRange.layerCount     = 1;
        if (vkCreateImageView(device, &ci, nullptr, &swapViews[i]) != VK_SUCCESS) {
            LOG_ERROR("Failed to create image view %zu", i);
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// createDepthResources
// ---------------------------------------------------------------------------
static VkFormat findDepthFormat(VkPhysicalDevice physDevice)
{
    for (VkFormat fmt : { VK_FORMAT_D32_SFLOAT, VK_FORMAT_D32_SFLOAT_S8_UINT, VK_FORMAT_D24_UNORM_S8_UINT }) {
        VkFormatProperties props;
        vkGetPhysicalDeviceFormatProperties(physDevice, fmt, &props);
        if (props.optimalTilingFeatures & VK_FORMAT_FEATURE_DEPTH_STENCIL_ATTACHMENT_BIT)
            return fmt;
    }
    return VK_FORMAT_D32_SFLOAT;
}

bool VulkanContext::createDepthResources()
{
    VkFormat depthFormat = findDepthFormat(physDevice);

    VkImageCreateInfo imgCI{};
    imgCI.sType         = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
    imgCI.imageType     = VK_IMAGE_TYPE_2D;
    imgCI.extent        = { swapExtent.width, swapExtent.height, 1 };
    imgCI.mipLevels     = 1;
    imgCI.arrayLayers   = 1;
    imgCI.format        = depthFormat;
    imgCI.tiling        = VK_IMAGE_TILING_OPTIMAL;
    imgCI.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    imgCI.usage         = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
    imgCI.samples       = VK_SAMPLE_COUNT_1_BIT;
    imgCI.sharingMode   = VK_SHARING_MODE_EXCLUSIVE;

    if (vkCreateImage(device, &imgCI, nullptr, &depthImage) != VK_SUCCESS) {
        LOG_ERROR("Failed to create depth image");
        return false;
    }

    VkMemoryRequirements memReqs;
    vkGetImageMemoryRequirements(device, depthImage, &memReqs);

    VkMemoryAllocateInfo allocInfo{};
    allocInfo.sType           = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    allocInfo.allocationSize  = memReqs.size;
    allocInfo.memoryTypeIndex = findMemoryType(memReqs.memoryTypeBits,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);

    vkAllocateMemory(device, &allocInfo, nullptr, &depthMemory);
    vkBindImageMemory(device, depthImage, depthMemory, 0);

    VkImageViewCreateInfo viewCI{};
    viewCI.sType                           = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
    viewCI.image                           = depthImage;
    viewCI.viewType                        = VK_IMAGE_VIEW_TYPE_2D;
    viewCI.format                          = depthFormat;
    viewCI.subresourceRange.aspectMask     = VK_IMAGE_ASPECT_DEPTH_BIT;
    viewCI.subresourceRange.baseMipLevel   = 0;
    viewCI.subresourceRange.levelCount     = 1;
    viewCI.subresourceRange.baseArrayLayer = 0;
    viewCI.subresourceRange.layerCount     = 1;

    vkCreateImageView(device, &viewCI, nullptr, &depthView);
    return true;
}

// ---------------------------------------------------------------------------
// createRenderPass
// ---------------------------------------------------------------------------
bool VulkanContext::createRenderPass()
{
    VkFormat depthFormat = findDepthFormat(physDevice);

    VkAttachmentDescription colorAtt{};
    colorAtt.format         = swapFormat;
    colorAtt.samples        = VK_SAMPLE_COUNT_1_BIT;
    colorAtt.loadOp         = VK_ATTACHMENT_LOAD_OP_CLEAR;
    colorAtt.storeOp        = VK_ATTACHMENT_STORE_OP_STORE;
    colorAtt.stencilLoadOp  = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    colorAtt.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    colorAtt.initialLayout  = VK_IMAGE_LAYOUT_UNDEFINED;
    colorAtt.finalLayout    = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

    VkAttachmentDescription depthAtt{};
    depthAtt.format         = depthFormat;
    depthAtt.samples        = VK_SAMPLE_COUNT_1_BIT;
    depthAtt.loadOp         = VK_ATTACHMENT_LOAD_OP_CLEAR;
    depthAtt.storeOp        = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depthAtt.stencilLoadOp  = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    depthAtt.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    depthAtt.initialLayout  = VK_IMAGE_LAYOUT_UNDEFINED;
    depthAtt.finalLayout    = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

    VkAttachmentReference colorRef{ 0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL };
    VkAttachmentReference depthRef{ 1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL };

    VkSubpassDescription subpass{};
    subpass.pipelineBindPoint       = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpass.colorAttachmentCount    = 1;
    subpass.pColorAttachments       = &colorRef;
    subpass.pDepthStencilAttachment = &depthRef;

    VkSubpassDependency dep{};
    dep.srcSubpass    = VK_SUBPASS_EXTERNAL;
    dep.dstSubpass    = 0;
    dep.srcStageMask  = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT
                      | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dep.srcAccessMask = 0;
    dep.dstStageMask  = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT
                      | VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
    dep.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT
                      | VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

    std::vector<VkAttachmentDescription> attachments = { colorAtt, depthAtt };
    VkRenderPassCreateInfo ci{};
    ci.sType           = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    ci.attachmentCount = static_cast<uint32_t>(attachments.size());
    ci.pAttachments    = attachments.data();
    ci.subpassCount    = 1;
    ci.pSubpasses      = &subpass;
    ci.dependencyCount = 1;
    ci.pDependencies   = &dep;

    if (vkCreateRenderPass(device, &ci, nullptr, &renderPass) != VK_SUCCESS) {
        LOG_ERROR("Failed to create render pass");
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// createFramebuffers
// ---------------------------------------------------------------------------
bool VulkanContext::createFramebuffers()
{
    framebuffers.resize(swapViews.size());
    for (size_t i = 0; i < swapViews.size(); ++i) {
        VkImageView attachments[] = { swapViews[i], depthView };
        VkFramebufferCreateInfo ci{};
        ci.sType           = VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO;
        ci.renderPass      = renderPass;
        ci.attachmentCount = 2;
        ci.pAttachments    = attachments;
        ci.width           = swapExtent.width;
        ci.height          = swapExtent.height;
        ci.layers          = 1;
        if (vkCreateFramebuffer(device, &ci, nullptr, &framebuffers[i]) != VK_SUCCESS) {
            LOG_ERROR("Failed to create framebuffer %zu", i);
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// createCommandPool / Buffers
// ---------------------------------------------------------------------------
bool VulkanContext::createCommandPool()
{
    VkCommandPoolCreateInfo ci{};
    ci.sType            = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
    ci.queueFamilyIndex = graphicsFamily;
    ci.flags            = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
    if (vkCreateCommandPool(device, &ci, nullptr, &commandPool) != VK_SUCCESS) {
        LOG_ERROR("Failed to create command pool");
        return false;
    }
    return true;
}

bool VulkanContext::createCommandBuffers()
{
    commandBuffers.resize(MAX_FRAMES_IN_FLIGHT);
    VkCommandBufferAllocateInfo ai{};
    ai.sType              = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    ai.commandPool        = commandPool;
    ai.level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    ai.commandBufferCount = static_cast<uint32_t>(commandBuffers.size());
    return vkAllocateCommandBuffers(device, &ai, commandBuffers.data()) == VK_SUCCESS;
}

// ---------------------------------------------------------------------------
// createSyncObjects
// ---------------------------------------------------------------------------
bool VulkanContext::createSyncObjects()
{
    imageAvailable.resize(MAX_FRAMES_IN_FLIGHT);
    renderFinished.resize(MAX_FRAMES_IN_FLIGHT);
    inFlight.resize(MAX_FRAMES_IN_FLIGHT);

    VkSemaphoreCreateInfo si{ VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO };
    VkFenceCreateInfo     fi{ VK_STRUCTURE_TYPE_FENCE_CREATE_INFO };
    fi.flags = VK_FENCE_CREATE_SIGNALED_BIT;

    for (int i = 0; i < MAX_FRAMES_IN_FLIGHT; ++i) {
        if (vkCreateSemaphore(device, &si, nullptr, &imageAvailable[i]) != VK_SUCCESS ||
            vkCreateSemaphore(device, &si, nullptr, &renderFinished[i]) != VK_SUCCESS ||
            vkCreateFence    (device, &fi, nullptr, &inFlight[i])       != VK_SUCCESS) {
            LOG_ERROR("Failed to create sync objects");
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// recreateSwapchain
// ---------------------------------------------------------------------------
bool VulkanContext::recreateSwapchain(SDL_Window* window)
{
    vkDeviceWaitIdle(device);
    cleanupSwapchain();
    return createSwapchain(window)
        && createImageViews()
        && createDepthResources()
        && createRenderPass()
        && createFramebuffers();
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
uint32_t VulkanContext::findMemoryType(uint32_t typeFilter, VkMemoryPropertyFlags props) const
{
    VkPhysicalDeviceMemoryProperties memProps;
    vkGetPhysicalDeviceMemoryProperties(physDevice, &memProps);
    for (uint32_t i = 0; i < memProps.memoryTypeCount; ++i) {
        if ((typeFilter & (1 << i)) &&
            (memProps.memoryTypes[i].propertyFlags & props) == props)
            return i;
    }
    GRIP_ASSERT(false, "Failed to find suitable memory type");
    return 0;
}

bool VulkanContext::createBuffer(VkDeviceSize size, VkBufferUsageFlags usage,
                                 VkMemoryPropertyFlags props,
                                 VkBuffer& buf, VkDeviceMemory& mem) const
{
    VkBufferCreateInfo bi{};
    bi.sType       = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
    bi.size        = size;
    bi.usage       = usage;
    bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    if (vkCreateBuffer(device, &bi, nullptr, &buf) != VK_SUCCESS) return false;

    VkMemoryRequirements memReqs;
    vkGetBufferMemoryRequirements(device, buf, &memReqs);

    VkMemoryAllocateInfo ai{};
    ai.sType           = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
    ai.allocationSize  = memReqs.size;
    ai.memoryTypeIndex = findMemoryType(memReqs.memoryTypeBits, props);
    if (vkAllocateMemory(device, &ai, nullptr, &mem) != VK_SUCCESS) return false;
    vkBindBufferMemory(device, buf, mem, 0);
    return true;
}

VkCommandBuffer VulkanContext::beginSingleTimeCommands() const
{
    VkCommandBufferAllocateInfo ai{};
    ai.sType              = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
    ai.level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
    ai.commandPool        = commandPool;
    ai.commandBufferCount = 1;

    VkCommandBuffer cmd;
    vkAllocateCommandBuffers(device, &ai, &cmd);

    VkCommandBufferBeginInfo bi{};
    bi.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
    vkBeginCommandBuffer(cmd, &bi);
    return cmd;
}

void VulkanContext::endSingleTimeCommands(VkCommandBuffer cmd) const
{
    vkEndCommandBuffer(cmd);
    VkSubmitInfo si{};
    si.sType              = VK_STRUCTURE_TYPE_SUBMIT_INFO;
    si.commandBufferCount = 1;
    si.pCommandBuffers    = &cmd;
    vkQueueSubmit(graphicsQueue, 1, &si, VK_NULL_HANDLE);
    vkQueueWaitIdle(graphicsQueue);
    vkFreeCommandBuffers(device, commandPool, 1, &cmd);
}

void VulkanContext::copyBuffer(VkBuffer src, VkBuffer dst, VkDeviceSize size) const
{
    VkCommandBuffer cmd = beginSingleTimeCommands();
    VkBufferCopy region{ 0, 0, size };
    vkCmdCopyBuffer(cmd, src, dst, 1, &region);
    endSingleTimeCommands(cmd);
}

// ---------------------------------------------------------------------------
// cleanupSwapchain / shutdown
// ---------------------------------------------------------------------------
void VulkanContext::cleanupSwapchain()
{
    for (auto fb : framebuffers)  vkDestroyFramebuffer(device, fb, nullptr);
    framebuffers.clear();
    vkDestroyRenderPass(device, renderPass, nullptr);
    renderPass = VK_NULL_HANDLE;
    vkDestroyImageView(device, depthView, nullptr);
    vkDestroyImage(device, depthImage, nullptr);
    vkFreeMemory(device, depthMemory, nullptr);
    depthView = VK_NULL_HANDLE; depthImage = VK_NULL_HANDLE; depthMemory = VK_NULL_HANDLE;
    for (auto iv : swapViews) vkDestroyImageView(device, iv, nullptr);
    swapViews.clear();
    vkDestroySwapchainKHR(device, swapchain, nullptr);
    swapchain = VK_NULL_HANDLE;
}

void VulkanContext::shutdown()
{
    if (device) vkDeviceWaitIdle(device);
    for (int i = 0; i < MAX_FRAMES_IN_FLIGHT; ++i) {
        vkDestroySemaphore(device, imageAvailable[i], nullptr);
        vkDestroySemaphore(device, renderFinished[i], nullptr);
        vkDestroyFence    (device, inFlight[i], nullptr);
    }
    vkDestroyCommandPool(device, commandPool, nullptr);
    cleanupSwapchain();
    vkDestroyDevice(device, nullptr);
    if (debugMessenger) destroyDebugUtilsMessengerEXT(instance, debugMessenger, nullptr);
    vkDestroySurfaceKHR(instance, surface, nullptr);
    vkDestroyInstance(instance, nullptr);
}
