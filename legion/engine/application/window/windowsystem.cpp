#include <application/window/windowsystem.hpp>
#include <rendering/debugrendering.hpp>

namespace legion::application
{
    sparse_map<GLFWwindow*, ecs::component_handle<window>> WindowSystem::m_windowComponents;
    async::spinlock WindowSystem::m_creationLock;

    async::spinlock WindowSystem::m_creationRequestLock;
    std::vector<WindowSystem::window_request> WindowSystem::m_creationRequests;

    async::spinlock WindowSystem::m_fullscreenRequestLock;
    std::vector<WindowSystem::fullscreen_toggle_request> WindowSystem::m_fullscreenRequests;

    async::spinlock WindowSystem::m_iconRequestLock;
    std::vector<WindowSystem::icon_request> WindowSystem::m_iconRequests;

    void WindowSystem::closeWindow(GLFWwindow* window)
    {
        if (!ContextHelper::initialized())
            return;

        {
            std::lock_guard guard(m_creationLock); // Lock all creation sensitive data.

            auto handle = m_windowComponents[window];

            async::spinlock* lock = nullptr;
            if (handle.valid())
            {
                m_eventBus->raiseEvent<window_close>(m_windowComponents[window]); // Trigger any callbacks that want to know about any windows closing.

                if (!ContextHelper::windowShouldClose(window)) // If a callback canceled the window destruction then we should cancel.
                    return;

                {
                    lock = handle.read().lock;
                    std::lock_guard guard(*lock); // "deleting" the window is technically writing, so we copy the pointer and use that to lock it.
                    handle.write(invalid_window); // We mark the window as deleted without deleting it yet. It can cause users to find invalid windows,                                                    
                                                  // but at least they won't use a destroyed component after the lock unlocks.

                    handle.destroy();
                    m_windowComponents.erase(window);
                }

                if (handle.entity.get_id() == world_entity_id)
                {
                    m_eventBus->raiseEvent<events::exit>(); // If the current window we're closing is the main window we want to close the application.
                }                                           // (we might want to leave this up to the user at some point.)

                ContextHelper::destroyWindow(window); // After all traces of the window throughout the engine have been erased we actually close the window.
            }
            if (lock)
                delete lock;
        }
    }

    void WindowSystem::onWindowMoved(GLFWwindow* window, int x, int y)
    {
        if (m_windowComponents.contains(window))
            m_eventBus->raiseEvent<window_move>(m_windowComponents[window], math::ivec2(x, y));
    }

    void WindowSystem::onWindowResize(GLFWwindow* win, int width, int height)
    {
        if (m_windowComponents.contains(win))
        {
            auto wincomp = m_windowComponents[win].read();
            wincomp.m_size = math::ivec2(width, height);
            m_windowComponents[win].write(wincomp);
            m_eventBus->raiseEvent<window_resize>(m_windowComponents[win], wincomp.m_size);
        }
    }

    void WindowSystem::onWindowRefresh(GLFWwindow* window)
    {
        if (m_windowComponents.contains(window))
            m_eventBus->raiseEvent<window_refresh>(m_windowComponents[window]);
    }

    void WindowSystem::onWindowFocus(GLFWwindow* window, int focused)
    {
        if (m_windowComponents.contains(window))
            m_eventBus->raiseEvent<window_focus>(m_windowComponents[window], focused);
    }

    void WindowSystem::onWindowIconify(GLFWwindow* window, int iconified)
    {
        if (m_windowComponents.contains(window))
            m_eventBus->raiseEvent<window_iconified>(m_windowComponents[window], iconified);
    }

    void WindowSystem::onWindowMaximize(GLFWwindow* window, int maximized)
    {
        if (m_windowComponents.contains(window))
            m_eventBus->raiseEvent<window_maximized>(m_windowComponents[window], maximized);
    }

    void WindowSystem::onWindowFrameBufferResize(GLFWwindow* window, int width, int height)
    {
        if (m_windowComponents.contains(window))
            m_eventBus->raiseEvent<window_framebuffer_resize>(m_windowComponents[window], math::ivec2(width, height));
    }

    void WindowSystem::onWindowContentRescale(GLFWwindow* window, float xscale, float yscale)
    {
        if (m_windowComponents.contains(window))
            m_eventBus->raiseEvent<window_content_rescale>(m_windowComponents[window], math::fvec2(xscale, xscale));
    }

    void WindowSystem::onItemDroppedInWindow(GLFWwindow* window, int count, const char** paths)
    {
        if (m_windowComponents.contains(window))
            m_eventBus->raiseEvent<window_item_dropped>(m_windowComponents[window], count, paths);
    }

    void WindowSystem::onMouseEnterWindow(GLFWwindow* window, int entered)
    {
        if (m_windowComponents.contains(window))
            m_eventBus->raiseEvent<mouse_enter_window>(m_windowComponents[window], entered);
    }

    void WindowSystem::onKeyInput(GLFWwindow* window, int key, int scancode, int action, int mods)
    {
        if (m_windowComponents.contains(window))
            m_eventBus->raiseEvent<key_input>(m_windowComponents[window], key, scancode, action, mods);
    }

    void WindowSystem::onCharInput(GLFWwindow* window, uint codepoint)
    {
        if (m_windowComponents.contains(window))
            m_eventBus->raiseEvent<char_input>(m_windowComponents[window], codepoint);
    }

    void WindowSystem::onMouseMoved(GLFWwindow* window, double xpos, double ypos)
    {
        if (m_windowComponents.contains(window))
            m_eventBus->raiseEvent<mouse_moved>(m_windowComponents[window], math::dvec2(xpos, ypos) / (math::dvec2)ContextHelper::getFramebufferSize(window));
    }

    void WindowSystem::onMouseButton(GLFWwindow* window, int button, int action, int mods)
    {
        if (m_windowComponents.contains(window))
            m_eventBus->raiseEvent<mouse_button>(m_windowComponents[window], button, action, mods);
    }

    void WindowSystem::onMouseScroll(GLFWwindow* window, double xoffset, double yoffset)
    {
        if (m_windowComponents.contains(window))
            m_eventBus->raiseEvent<mouse_scrolled>(m_windowComponents[window], math::dvec2(xoffset, yoffset));
    }

    void WindowSystem::onExit(events::exit* event)
    {
        std::lock_guard guard(m_creationLock);
        for (auto entity : m_windowQuery)
        {
            auto handle = entity.get_component_handle<window>();
            window win = handle.read();

            async::spinlock* lock = nullptr;
            if (handle.valid())
            {
                m_eventBus->raiseEvent<window_close>(handle); // Trigger any callbacks that want to know about any windows closing.

                {
                    lock = handle.read().lock;
                    std::lock_guard guard(*lock); // "deleting" the window is technically writing, so we copy the pointer and use that to lock it.
                    handle.write(invalid_window); // We mark the window as deleted without deleting it yet. It can cause users to find invalid windows,                                                    
                                                  // but at least they won't use a destroyed component after the lock unlocks.
                    handle.destroy();
                    m_windowComponents.erase(win);
                }

                ContextHelper::destroyWindow(win); // After all traces of the window throughout the engine have been erased we actually close the window.
            }
            if (lock)
                delete lock;
        }

        m_exit = true;
        ContextHelper::terminate();
    }

    bool WindowSystem::windowStillExists(GLFWwindow* win)
    {
        return m_windowComponents.contains(win);
    }

    void WindowSystem::requestIconChange(id_type entityId, image_handle icon)
    {
        if (entityId)
        {
            std::lock_guard guard(m_iconRequestLock);
            m_iconRequests.emplace_back(entityId, icon);
        }
        else
            log::warn("Icon change denied, invalid entity given.");
    }

    void WindowSystem::requestIconChange(id_type entityId, const std::string& iconName)
    {
        if (entityId)
        {
            std::lock_guard guard(m_iconRequestLock);
            m_iconRequests.emplace_back(entityId, iconName);
        }
        else
            log::warn("Icon change denied, invalid entity given.");
    }

    void WindowSystem::requestFullscreenToggle(id_type entityId, math::ivec2 position, math::ivec2 size)
    {
        if (entityId)
        {
            std::lock_guard guard(m_fullscreenRequestLock);
            m_fullscreenRequests.emplace_back(entityId, position, size);
        }
        else
            log::warn("Fullscreen toggle denied, invalid entity given.");
    }

    void WindowSystem::requestWindow(id_type entityId, math::ivec2 size, const std::string& name, image_handle icon, GLFWmonitor* monitor, GLFWwindow* share, int swapInterval, const std::vector<std::pair<int, int>>& hints)
    {
        if (entityId)
        {
            std::lock_guard guard(m_creationRequestLock);
            m_creationRequests.emplace_back(entityId, size, name, icon, monitor, share, swapInterval, hints);
        }
        else
            log::warn("Window creation denied, invalid entity given.");
    }

    void WindowSystem::requestWindow(id_type entityId, math::ivec2 size, const std::string& name, image_handle icon, GLFWmonitor* monitor, GLFWwindow* share, int swapInterval)
    {
        if (entityId)
        {
            std::lock_guard guard(m_creationRequestLock);
            m_creationRequests.emplace_back(entityId, size, name, icon, monitor, share, swapInterval);
        }
        else
            log::warn("Window creation denied, invalid entity given.");
    }

    void WindowSystem::requestWindow(id_type entityId, math::ivec2 size, const std::string& name, const std::string& iconName, GLFWmonitor* monitor, GLFWwindow* share, int swapInterval, const std::vector<std::pair<int, int>>& hints)
    {
        if (entityId)
        {
            std::lock_guard guard(m_creationRequestLock);
            m_creationRequests.emplace_back(entityId, size, name, iconName, monitor, share, swapInterval);
        }
        else
            log::warn("Window creation denied, invalid entity given.");
    }

    void WindowSystem::requestWindow(id_type entityId, math::ivec2 size, const std::string& name, const std::string& iconName, GLFWmonitor* monitor, GLFWwindow* share, int swapInterval)
    {
        if (entityId)
        {
            std::lock_guard guard(m_creationRequestLock);
            m_creationRequests.emplace_back(entityId, size, name, iconName, monitor, share, swapInterval);
        }
        else
            log::warn("Window creation denied, invalid entity given.");
    }

    void WindowSystem::setup()
    {
        using namespace filesystem::literals;
        m_defaultIcon = ImageCache::create_image("Legion Icon", "engine://resources/legion/icon"_view, { channel_format::eight_bit, image_components::rgba, false });

        m_windowQuery = createQuery<window>();
        bindToEvent<events::exit, &WindowSystem::onExit>();

        if (m_creationRequests.empty() || (std::find_if(m_creationRequests.begin(), m_creationRequests.end(), [](window_request& r) { return r.entityId == world_entity_id; }) == m_creationRequests.end()))
            requestWindow(world_entity_id, math::ivec2(1360, 768), "LEGION Engine", invalid_image_handle, nullptr, nullptr, 1); // Create the request for the main window.

        m_scheduler->sendCommand(m_scheduler->getChainThreadId("Input"), [](void* param) // We send a command to the input thread before the input process chain starts.
            {                                                                            // This way we can create the main window before the rest of the engine get initialised.
                WindowSystem* self = reinterpret_cast<WindowSystem*>(param);

                if (!ContextHelper::initialized()) // Initialize context.
                    if (!ContextHelper::init())
                    {
                        self->exit();
                        return; // If we can't initialize we can't create any windows, not creating the main window means the engine should shut down.
                    }
                log::trace("Creating main window.");
                self->createWindows();
                self->showMainWindow();
            }, this);

        createProcess<&WindowSystem::refreshWindows>("Rendering");
        createProcess<&WindowSystem::handleWindowEvents>("Input");
    }

    void WindowSystem::createWindows()
    {
        OPTICK_EVENT();
        if (m_exit) // If the engine is exiting then we can't create new windows.
            return;

        std::lock_guard guard(m_creationRequestLock);
        for (auto& request : m_creationRequests)
        {
            if (!m_ecs->validateEntity(request.entityId))
            {
                log::warn("Window creation denied, entity {} doesn't exist.", request.entityId);
                continue;
            }

            log::trace("creating a window");

            if (request.hints.size())
            {
                for (auto& [hint, value] : request.hints)
                    ContextHelper::windowHint(hint, value);
            }
            else // Default window hints.
            {
                ContextHelper::windowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
                ContextHelper::windowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
                ContextHelper::windowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_FALSE);
                ContextHelper::windowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

                GLFWmonitor* monitor = request.monitor;

                if (!request.monitor)
                    monitor = ContextHelper::getPrimaryMonitor();

                const GLFWvidmode* mode = ContextHelper::getVideoMode(monitor);

                ContextHelper::windowHint(GLFW_RED_BITS, mode->redBits);
                ContextHelper::windowHint(GLFW_GREEN_BITS, mode->greenBits);
                ContextHelper::windowHint(GLFW_BLUE_BITS, mode->blueBits);
                ContextHelper::windowHint(GLFW_REFRESH_RATE, mode->refreshRate);
            }

            if (request.size == math::ivec2(0, 0))
                request.size = { 400, 400 };

            if (request.name.empty())
                request.name = "LEGION Engine";

            image_handle icon = request.icon;
            if (icon == invalid_image_handle)
                icon = m_defaultIcon;

            window win = ContextHelper::createWindow(request.size, request.name.c_str(), request.monitor, request.share);

            auto [lock, image] = icon.get_raw_image();
            {
                async::readonly_guard guard(lock);
                if (image.components == image_components::rgba && image.format == channel_format::eight_bit)
                {
                    GLFWimage icon{ image.size.x, image.size.y, image.get_raw_data<byte>() };

                    ContextHelper::setWindowIcon(win, 1, &icon);
                }
            }

            win.m_title = request.name;
            win.m_isFullscreen = (request.monitor != nullptr);
            win.m_swapInterval = request.swapInterval;
            win.m_size = request.size;

            auto setCallbacks = [](const window& win)
            {
                ContextHelper::makeContextCurrent(win);
                ContextHelper::swapInterval(win.m_swapInterval);
                ContextHelper::setWindowCloseCallback(win, &WindowSystem::closeWindow);
                ContextHelper::setWindowPosCallback(win, &WindowSystem::onWindowMoved);
                ContextHelper::setWindowSizeCallback(win, &WindowSystem::onWindowResize);
                ContextHelper::setWindowRefreshCallback(win, &WindowSystem::onWindowRefresh);
                ContextHelper::setWindowFocusCallback(win, &WindowSystem::onWindowFocus);
                ContextHelper::setWindowIconifyCallback(win, &WindowSystem::onWindowIconify);
                ContextHelper::setWindowMaximizeCallback(win, &WindowSystem::onWindowMaximize);
                ContextHelper::setFramebufferSizeCallback(win, &WindowSystem::onWindowFrameBufferResize);
                ContextHelper::setWindowContentScaleCallback(win, &WindowSystem::onWindowContentRescale);
                ContextHelper::setDropCallback(win, &WindowSystem::onItemDroppedInWindow);
                ContextHelper::setCursorEnterCallback(win, &WindowSystem::onMouseEnterWindow);
                ContextHelper::setKeyCallback(win, &WindowSystem::onKeyInput);
                ContextHelper::setCharCallback(win, &WindowSystem::onCharInput);
                ContextHelper::setCursorPosCallback(win, &WindowSystem::onMouseMoved);
                ContextHelper::setMouseButtonCallback(win, &WindowSystem::onMouseButton);
                ContextHelper::setScrollCallback(win, &WindowSystem::onMouseScroll);
                ContextHelper::makeContextCurrent(nullptr);
            };

            ecs::component_handle<window> handle(request.entityId);
            if (!m_ecs->hasComponent<window>(request.entityId))
            {
                win.lock = new async::spinlock();

                std::lock_guard wguard(*win.lock);     // This is the only code that has access to win.lock right now, so there's no deadlock risk.
                std::lock_guard cguard(m_creationLock);// Locking them both separately is faster than using a multilock.
                m_windowComponents.insert(win, handle);
                handle = m_ecs->createComponent<window>(request.entityId, win);

                log::trace("created window: {}", request.name);


                // Set all callbacks.
                setCallbacks(win);
            }
            else
            {
                handle = m_ecs->getComponent<window>(request.entityId);
                window oldWindow = handle.read();

                std::scoped_lock wguard(*oldWindow.lock, m_creationLock);

                m_windowComponents.erase(oldWindow);
                ContextHelper::destroyWindow(oldWindow);

                win.lock = oldWindow.lock;
                handle.write(win);
                m_windowComponents.insert(win, handle);

                log::trace("replaced window: {}", request.name);


                // Set all callbacks.
                setCallbacks(win);
            }
        }

        m_creationRequests.clear();
    }

    void WindowSystem::fullscreenWindows()
    {
        OPTICK_EVENT();
        if (m_exit) // If the engine is exiting then we can't change any windows.
            return;

        std::lock_guard guard(m_fullscreenRequestLock);
        for (auto& request : m_fullscreenRequests)
        {
            if (!m_ecs->validateEntity(request.entityId))
            {
                log::warn("Fullscreen toggle denied, entity {} doesn't exist.", request.entityId);
                continue;
            }

            auto handle = m_ecs->getComponent<window>(request.entityId);
            if (!handle)
            {
                log::warn("Fullscreen toggle denied, entity {} doesn't have a window.", request.entityId);
                continue;
            }

            window win = handle.read();
            std::lock_guard wguard(*win.lock);

            if (win.m_isFullscreen)
            {
                GLFWmonitor* monitor = ContextHelper::getPrimaryMonitor();
                const GLFWvidmode* mode = ContextHelper::getVideoMode(monitor);

                ContextHelper::setWindowMonitor(win, nullptr, request.position, request.size, mode->refreshRate);
                win.m_size = request.size;
                log::trace("Set window {} to windowed.", request.entityId);
            }
            else
            {
                GLFWmonitor* monitor = ContextHelper::getCurrentMonitor(win);
                const GLFWvidmode* mode = ContextHelper::getVideoMode(monitor);

                ContextHelper::setWindowMonitor(win, monitor, { 0 ,0 }, math::ivec2(mode->width, mode->height), mode->refreshRate);
                win.m_size = math::ivec2(mode->width, mode->height);
                ContextHelper::makeContextCurrent(win);
                ContextHelper::swapInterval(win.m_swapInterval);
                ContextHelper::makeContextCurrent(nullptr);
                log::trace("Set window {} to fullscreen.", request.entityId);
            }

            win.m_isFullscreen = !win.m_isFullscreen;
            handle.write(win);
        }
        m_fullscreenRequests.clear();
    }

    void WindowSystem::updateWindowIcons()
    {
        OPTICK_EVENT();
        if (m_exit) // If the engine is exiting then we can't change any windows.
            return;

        std::lock_guard guard(m_iconRequestLock);
        for (auto& request : m_iconRequests)
        {

            if (!m_ecs->validateEntity(request.entityId))
            {
                log::warn("Icon change denied, entity {} doesn't exist.", request.entityId);
                continue;
            }

            auto handle = m_ecs->getComponent<window>(request.entityId);
            if (!handle)
            {
                log::warn("Icon change denied, entity {} doesn't have a window.", request.entityId);
                continue;
            }

            auto [lock, image] = request.icon.get_raw_image();
            async::readonly_guard guard(lock);
            if (image.components != image_components::rgba || image.format != channel_format::eight_bit)
            {
                log::warn("Icon change denied, image {} has the wrong format. The needed format is 4 channels with 8 bits per channel.", request.icon.id);
                continue;
            }

            GLFWimage icon{ image.size.x, image.size.y, image.get_raw_data<byte>() };

            ContextHelper::setWindowIcon(handle.read(), 1, &icon);
        }
        m_iconRequests.clear();
    }

    void WindowSystem::refreshWindows(time::time_span<fast_time> deltaTime)
    {
        OPTICK_EVENT();
        if (!ContextHelper::initialized())
            return;

        std::lock_guard guard(m_creationLock);
        m_windowQuery.queryEntities();
        for (auto entity : m_windowQuery)
        {
            window win = entity.get_component_handle<window>().read();
            {
                std::lock_guard guard(*win.lock);
                ContextHelper::makeContextCurrent(win);
                ContextHelper::swapBuffers(win);
                ContextHelper::makeContextCurrent(nullptr);
            }
        }
    }

    void WindowSystem::handleWindowEvents(time::time_span<fast_time> deltaTime)
    {
        OPTICK_EVENT();
        createWindows();
        updateWindowIcons();
        fullscreenWindows();

        if (!ContextHelper::initialized())
            return;

        ContextHelper::pollEvents();
        ContextHelper::updateWindowFocus();
    }

}
