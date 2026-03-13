#include "VulkanContext.hpp"
#include <SDL2/SDL_vulkan.h>
#include <vector>
#include <algorithm>
#include <cstdio>

bool VulkanContext::init(SDL_Window* window)
{
    // ---- Instance ----
    {
        VkApplicationInfo ai{VK_STRUCTURE_TYPE_APPLICATION_INFO};
        ai.pApplicationName = "GRIP-Sim";
        ai.apiVersion       = VK_API_VERSION_1_0;

        unsigned sdlExtN = 0;
        SDL_Vulkan_GetInstanceExtensions(window, &sdlExtN, nullptr);
        std::vector<const char*> exts(sdlExtN);
        SDL_Vulkan_GetInstanceExtensions(window, &sdlExtN, exts.data());

        VkInstanceCreateInfo ci{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
        ci.pApplicationInfo        = &ai;
        ci.enabledExtensionCount   = (uint32_t)exts.size();
        ci.ppEnabledExtensionNames = exts.data();
#ifndef NDEBUG
        const char* layers[] = {"VK_LAYER_KHRONOS_validation"};
        ci.enabledLayerCount   = 1;
        ci.ppEnabledLayerNames = layers;
#endif
        if (vkCreateInstance(&ci, nullptr, &instance) != VK_SUCCESS) return false;
    }

    // ---- Surface ----
    if (!SDL_Vulkan_CreateSurface(window, instance, &surface)) return false;

    // ---- Physical device ----
    {
        uint32_t n = 0;
        vkEnumeratePhysicalDevices(instance, &n, nullptr);
        std::vector<VkPhysicalDevice> devs(n);
        vkEnumeratePhysicalDevices(instance, &n, devs.data());

        for (auto d : devs) {
            uint32_t qn = 0;
            vkGetPhysicalDeviceQueueFamilyProperties(d, &qn, nullptr);
            std::vector<VkQueueFamilyProperties> qf(qn);
            vkGetPhysicalDeviceQueueFamilyProperties(d, &qn, qf.data());

            bool gfx = false, prs = false;
            for (uint32_t i = 0; i < qn; ++i) {
                if (qf[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) { graphicsFamily = i; gfx = true; }
                VkBool32 sup = VK_FALSE;
                vkGetPhysicalDeviceSurfaceSupportKHR(d, i, surface, &sup);
                if (sup) { presentFamily = i; prs = true; }
            }
            if (gfx && prs) { physicalDevice = d; break; }
        }
        if (!physicalDevice) return false;
    }

    // ---- Logical device ----
    {
        float prio = 1.f;
        std::vector<VkDeviceQueueCreateInfo> qcis;
        VkDeviceQueueCreateInfo qci{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
        qci.queueCount       = 1;
        qci.pQueuePriorities = &prio;

        qci.queueFamilyIndex = graphicsFamily; qcis.push_back(qci);
        if (presentFamily != graphicsFamily) { qci.queueFamilyIndex = presentFamily; qcis.push_back(qci); }

        const char* devExts[] = { VK_KHR_SWAPCHAIN_EXTENSION_NAME };
        VkDeviceCreateInfo dci{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
        dci.queueCreateInfoCount    = (uint32_t)qcis.size();
        dci.pQueueCreateInfos       = qcis.data();
        dci.enabledExtensionCount   = 1;
        dci.ppEnabledExtensionNames = devExts;
        if (vkCreateDevice(physicalDevice, &dci, nullptr, &device) != VK_SUCCESS) return false;

        vkGetDeviceQueue(device, graphicsFamily, 0, &graphicsQueue);
        vkGetDeviceQueue(device, presentFamily,  0, &presentQueue);
    }

    // ---- Swapchain ----
    {
        VkSurfaceCapabilitiesKHR caps;
        vkGetPhysicalDeviceSurfaceCapabilitiesKHR(physicalDevice, surface, &caps);

        uint32_t fn = 0;
        vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, surface, &fn, nullptr);
        std::vector<VkSurfaceFormatKHR> fmts(fn);
        vkGetPhysicalDeviceSurfaceFormatsKHR(physicalDevice, surface, &fn, fmts.data());

        swapFormat = fmts[0].format;
        VkColorSpaceKHR cs = fmts[0].colorSpace;
        for (auto& f : fmts)
            if (f.format == VK_FORMAT_B8G8R8A8_SRGB &&
                f.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR)
            { swapFormat = f.format; cs = f.colorSpace; break; }

        swapExtent = caps.currentExtent;
        if (swapExtent.width == 0xFFFFFFFF) {
            int w, h; SDL_Vulkan_GetDrawableSize(window, &w, &h);
            swapExtent = { (uint32_t)w, (uint32_t)h };
        }

        uint32_t imgN = caps.minImageCount + 1;
        if (caps.maxImageCount > 0) imgN = std::min(imgN, caps.maxImageCount);

        VkSwapchainCreateInfoKHR sci{VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR};
        sci.surface          = surface;
        sci.minImageCount    = imgN;
        sci.imageFormat      = swapFormat;
        sci.imageColorSpace  = cs;
        sci.imageExtent      = swapExtent;
        sci.imageArrayLayers = 1;
        sci.imageUsage       = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
        sci.preTransform     = caps.currentTransform;
        sci.compositeAlpha   = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR;
        sci.presentMode      = VK_PRESENT_MODE_FIFO_KHR;
        sci.clipped          = VK_TRUE;

        uint32_t fams[] = { graphicsFamily, presentFamily };
        if (graphicsFamily != presentFamily) {
            sci.imageSharingMode      = VK_SHARING_MODE_CONCURRENT;
            sci.queueFamilyIndexCount = 2;
            sci.pQueueFamilyIndices   = fams;
        } else {
            sci.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE;
        }

        if (vkCreateSwapchainKHR(device, &sci, nullptr, &swapchain) != VK_SUCCESS) return false;

        vkGetSwapchainImagesKHR(device, swapchain, &imgN, nullptr);
        swapImages.resize(imgN);
        vkGetSwapchainImagesKHR(device, swapchain, &imgN, swapImages.data());

        swapViews.resize(imgN);
        for (uint32_t i = 0; i < imgN; ++i) {
            VkImageViewCreateInfo iv{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
            iv.image    = swapImages[i];
            iv.viewType = VK_IMAGE_VIEW_TYPE_2D;
            iv.format   = swapFormat;
            iv.subresourceRange = { VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1 };
            vkCreateImageView(device, &iv, nullptr, &swapViews[i]);
        }
    }

    // ---- Depth buffer ----
    {
        VkImageCreateInfo ici{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
        ici.imageType   = VK_IMAGE_TYPE_2D;
        ici.format      = depthFormat;
        ici.extent      = { swapExtent.width, swapExtent.height, 1 };
        ici.mipLevels   = 1;
        ici.arrayLayers = 1;
        ici.samples     = VK_SAMPLE_COUNT_1_BIT;
        ici.tiling      = VK_IMAGE_TILING_OPTIMAL;
        ici.usage       = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
        vkCreateImage(device, &ici, nullptr, &depthImage);

        VkMemoryRequirements mr;
        vkGetImageMemoryRequirements(device, depthImage, &mr);
        VkMemoryAllocateInfo mai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        mai.allocationSize  = mr.size;
        mai.memoryTypeIndex = findMemType(mr.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        vkAllocateMemory(device, &mai, nullptr, &depthMem);
        vkBindImageMemory(device, depthImage, depthMem, 0);

        VkImageViewCreateInfo iv{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
        iv.image    = depthImage;
        iv.viewType = VK_IMAGE_VIEW_TYPE_2D;
        iv.format   = depthFormat;
        iv.subresourceRange = { VK_IMAGE_ASPECT_DEPTH_BIT, 0, 1, 0, 1 };
        vkCreateImageView(device, &iv, nullptr, &depthView);
    }

    // ---- Render pass ----
    {
        VkAttachmentDescription att[2]{};
        att[0].format         = swapFormat;
        att[0].samples        = VK_SAMPLE_COUNT_1_BIT;
        att[0].loadOp         = VK_ATTACHMENT_LOAD_OP_CLEAR;
        att[0].storeOp        = VK_ATTACHMENT_STORE_OP_STORE;
        att[0].stencilLoadOp  = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        att[0].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        att[0].initialLayout  = VK_IMAGE_LAYOUT_UNDEFINED;
        att[0].finalLayout    = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;

        att[1].format         = depthFormat;
        att[1].samples        = VK_SAMPLE_COUNT_1_BIT;
        att[1].loadOp         = VK_ATTACHMENT_LOAD_OP_CLEAR;
        att[1].storeOp        = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        att[1].stencilLoadOp  = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
        att[1].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
        att[1].initialLayout  = VK_IMAGE_LAYOUT_UNDEFINED;
        att[1].finalLayout    = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;

        VkAttachmentReference colRef{ 0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL };
        VkAttachmentReference depRef{ 1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL };

        VkSubpassDescription sub{};
        sub.pipelineBindPoint       = VK_PIPELINE_BIND_POINT_GRAPHICS;
        sub.colorAttachmentCount    = 1;
        sub.pColorAttachments       = &colRef;
        sub.pDepthStencilAttachment = &depRef;

        VkSubpassDependency dep{};
        dep.srcSubpass    = VK_SUBPASS_EXTERNAL;
        dep.dstSubpass    = 0;
        dep.srcStageMask  = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT |
                            VK_PIPELINE_STAGE_EARLY_FRAGMENT_TESTS_BIT;
        dep.dstStageMask  = dep.srcStageMask;
        dep.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT |
                            VK_ACCESS_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT;

        VkRenderPassCreateInfo rp{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
        rp.attachmentCount = 2;
        rp.pAttachments    = att;
        rp.subpassCount    = 1;
        rp.pSubpasses      = &sub;
        rp.dependencyCount = 1;
        rp.pDependencies   = &dep;
        vkCreateRenderPass(device, &rp, nullptr, &renderPass);
    }

    // ---- Framebuffers ----
    framebuffers.resize(swapImages.size());
    for (size_t i = 0; i < framebuffers.size(); ++i) {
        VkImageView views[] = { swapViews[i], depthView };
        VkFramebufferCreateInfo fb{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
        fb.renderPass      = renderPass;
        fb.attachmentCount = 2;
        fb.pAttachments    = views;
        fb.width           = swapExtent.width;
        fb.height          = swapExtent.height;
        fb.layers          = 1;
        vkCreateFramebuffer(device, &fb, nullptr, &framebuffers[i]);
    }

    // ---- Command pool + buffers ----
    {
        VkCommandPoolCreateInfo cp{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
        cp.flags            = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        cp.queueFamilyIndex = graphicsFamily;
        vkCreateCommandPool(device, &cp, nullptr, &commandPool);

        uint32_t imgCount = (uint32_t)swapImages.size();
        cmdBufs.resize(imgCount);
        VkCommandBufferAllocateInfo ca{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        ca.commandPool        = commandPool;
        ca.level              = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        ca.commandBufferCount = imgCount;
        vkAllocateCommandBuffers(device, &ca, cmdBufs.data());
    }

    // ---- Sync (one set per swapchain image) ----
    {
        uint32_t n = (uint32_t)swapImages.size();
        imageReady.resize(n);
        renderDone.resize(n);
        inFlight.resize(n);
        VkSemaphoreCreateInfo si{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
        VkFenceCreateInfo     fi{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
        fi.flags = VK_FENCE_CREATE_SIGNALED_BIT;
        for (uint32_t i = 0; i < n; ++i) {
            vkCreateSemaphore(device, &si, nullptr, &imageReady[i]);
            vkCreateSemaphore(device, &si, nullptr, &renderDone[i]);
            vkCreateFence    (device, &fi, nullptr, &inFlight[i]);
        }
    }

    return true;
}

VkCommandBuffer VulkanContext::beginFrame()
{
    vkWaitForFences(device, 1, &inFlight[frame], VK_TRUE, UINT64_MAX);

    VkResult res = vkAcquireNextImageKHR(device, swapchain, UINT64_MAX,
                                         imageReady[frame], VK_NULL_HANDLE, &imgIdx);
    if (res == VK_ERROR_OUT_OF_DATE_KHR) return VK_NULL_HANDLE;

    vkResetFences(device, 1, &inFlight[frame]);

    VkCommandBuffer cmd = cmdBufs[frame];
    vkResetCommandBuffer(cmd, 0);

    VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
    vkBeginCommandBuffer(cmd, &bi);

    VkClearValue clears[2]{};
    clears[0].color        = {{ 0.12f, 0.12f, 0.14f, 1.f }};
    clears[1].depthStencil = { 1.f, 0 };

    VkRenderPassBeginInfo rp{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
    rp.renderPass        = renderPass;
    rp.framebuffer       = framebuffers[imgIdx];
    rp.renderArea.extent = swapExtent;
    rp.clearValueCount   = 2;
    rp.pClearValues      = clears;
    vkCmdBeginRenderPass(cmd, &rp, VK_SUBPASS_CONTENTS_INLINE);

    return cmd;
}

void VulkanContext::endFrame()
{
    VkCommandBuffer cmd = cmdBufs[frame];
    vkCmdEndRenderPass(cmd);
    vkEndCommandBuffer(cmd);

    VkPipelineStageFlags wait = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
    si.waitSemaphoreCount   = 1;
    si.pWaitSemaphores      = &imageReady[frame];
    si.pWaitDstStageMask    = &wait;
    si.commandBufferCount   = 1;
    si.pCommandBuffers      = &cmd;
    si.signalSemaphoreCount = 1;
    si.pSignalSemaphores    = &renderDone[frame];
    vkQueueSubmit(graphicsQueue, 1, &si, inFlight[frame]);

    VkPresentInfoKHR pi{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};
    pi.waitSemaphoreCount = 1;
    pi.pWaitSemaphores    = &renderDone[frame];
    pi.swapchainCount     = 1;
    pi.pSwapchains        = &swapchain;
    pi.pImageIndices      = &imgIdx;
    vkQueuePresentKHR(presentQueue, &pi);

    frame = (frame + 1) % (uint32_t)imageReady.size();
}

void VulkanContext::shutdown()
{
    for (size_t i = 0; i < imageReady.size(); ++i) {
        vkDestroySemaphore(device, imageReady[i], nullptr);
        vkDestroySemaphore(device, renderDone[i], nullptr);
        vkDestroyFence    (device, inFlight[i],   nullptr);
    }
    vkDestroyCommandPool(device, commandPool, nullptr);
    for (auto fb : framebuffers) vkDestroyFramebuffer(device, fb, nullptr);
    vkDestroyRenderPass(device, renderPass, nullptr);
    vkDestroyImageView (device, depthView,  nullptr);
    vkDestroyImage     (device, depthImage, nullptr);
    vkFreeMemory       (device, depthMem,   nullptr);
    for (auto v : swapViews) vkDestroyImageView(device, v, nullptr);
    vkDestroySwapchainKHR(device, swapchain, nullptr);
    vkDestroyDevice      (device, nullptr);
    vkDestroySurfaceKHR  (instance, surface, nullptr);
    vkDestroyInstance    (instance, nullptr);
}

uint32_t VulkanContext::findMemType(uint32_t filter, VkMemoryPropertyFlags props) const
{
    VkPhysicalDeviceMemoryProperties mp;
    vkGetPhysicalDeviceMemoryProperties(physicalDevice, &mp);
    for (uint32_t i = 0; i < mp.memoryTypeCount; ++i)
        if ((filter & (1u << i)) && (mp.memoryTypes[i].propertyFlags & props) == props)
            return i;
    return 0;
}

VkBuffer VulkanContext::allocBuffer(VkDeviceSize size, VkBufferUsageFlags usage,
                                    VkMemoryPropertyFlags props, VkDeviceMemory& mem) const
{
    VkBufferCreateInfo ci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    ci.size        = size;
    ci.usage       = usage;
    ci.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

    VkBuffer buf;
    vkCreateBuffer(device, &ci, nullptr, &buf);

    VkMemoryRequirements mr;
    vkGetBufferMemoryRequirements(device, buf, &mr);

    VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    ai.allocationSize  = mr.size;
    ai.memoryTypeIndex = findMemType(mr.memoryTypeBits, props);
    vkAllocateMemory(device, &ai, nullptr, &mem);
    vkBindBufferMemory(device, buf, mem, 0);
    return buf;
}
