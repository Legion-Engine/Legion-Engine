#include <application/context/contexthelper.hpp>

namespace legion::application
{
    std::atomic_bool ContextHelper::m_initialized;
    atomic_sparse_map<GLFWwindow*, bool> ContextHelper::m_windowInitialized;

    async::rw_spinlock ContextHelper::m_initCallbackLock;
    multicast_delegate<void()> ContextHelper::m_onInit;

    std::atomic<GLFWwindow*> ContextHelper::newFocus;

    bool ContextHelper::initialized()
    {
        return m_initialized.load(std::memory_order_acquire);
    }

    bool ContextHelper::init()
    {
        glfwSetErrorCallback([](int code, cstring desc)
            {
                log::error("GLFW ERROR {}: {}", code, desc);
            });

        bool success = glfwInit();
        if (success)
        {
            m_initialized.store(true, std::memory_order_release);
            async::readonly_guard guard(m_initCallbackLock);
            m_onInit.invoke();
        }

        return success;
    }

    bool ContextHelper::addOnInitCallback(delegate<void()> callback)
    {
        if (initialized()) {
            callback();
            return false;
        }

        async::readwrite_guard guard(m_initCallbackLock);
        m_onInit += callback;
        return true;
    }

    void ContextHelper::terminate()
    {
        m_initialized.store(false, std::memory_order_release);
        glfwTerminate();
    }

    int ContextHelper::getError(cstring* desc)
    {
        return glfwGetError(desc);
    }

    GLFWmonitor* ContextHelper::getPrimaryMonitor()
    {
        return glfwGetPrimaryMonitor();
    }

    GLFWmonitor* ContextHelper::getCurrentMonitor(GLFWwindow* window)
    {
        int nmonitors, i;
        int wx, wy, ww, wh;
        int mx, my, mw, mh;
        int overlap, bestoverlap;
        GLFWmonitor* bestmonitor;
        GLFWmonitor** monitors;
        const GLFWvidmode* mode;

        bestoverlap = 0;
        bestmonitor = NULL;

        glfwGetWindowPos(window, &wx, &wy);
        glfwGetWindowSize(window, &ww, &wh);
        monitors = glfwGetMonitors(&nmonitors);

        for (i = 0; i < nmonitors; i++) {
            mode = glfwGetVideoMode(monitors[i]);
            glfwGetMonitorPos(monitors[i], &mx, &my);
            mw = mode->width;
            mh = mode->height;

            overlap =
                math::max(0, math::min(wx + ww, mx + mw) - math::max(wx, mx)) *
                math::max(0, math::min(wy + wh, my + mh) - math::max(wy, my));

            if (bestoverlap < overlap) {
                bestoverlap = overlap;
                bestmonitor = monitors[i];
            }
        }

        return bestmonitor;
    }

    void ContextHelper::setWindowMonitor(GLFWwindow* window, GLFWmonitor* monitor, math::ivec2 pos, math::ivec2 size, int refreshRate)
    {
        glfwSetWindowMonitor(window, monitor, pos.x, pos.y, size.x, size.y, refreshRate);
    }

    const GLFWvidmode* ContextHelper::getPrimaryVideoMode()
    {
        return glfwGetVideoMode(glfwGetPrimaryMonitor());
    }

    const GLFWvidmode* ContextHelper::getVideoMode(GLFWmonitor* monitor)
    {
        return glfwGetVideoMode(monitor);
    }

    void ContextHelper::windowHint(int hint, int value)
    {
        glfwWindowHint(hint, value);
    }

    GLFWwindow* ContextHelper::createWindow(math::ivec2 dim, const char* title, GLFWmonitor* monitor, GLFWwindow* share)
    {
        return glfwCreateWindow(dim.x, dim.y, title, monitor, share);
    }

    GLFWwindow* ContextHelper::createWindow(int width, int height, const char* title, GLFWmonitor* monitor, GLFWwindow* share)
    {
        return glfwCreateWindow(width, height, title, monitor, share);
    }

    void ContextHelper::showWindow(GLFWwindow* window)
    {
        newFocus.store(window, std::memory_order_release);
    }

    void ContextHelper::updateWindowFocus()
    {
        GLFWwindow* focus = newFocus.load(std::memory_order_acquire);
        if (!focus)
            return;

        while (!newFocus.compare_exchange_weak(focus, nullptr, std::memory_order_release, std::memory_order_relaxed))
            ;

        glfwHideWindow(focus);
        glfwShowWindow(focus);
        glfwFocusWindow(focus);
    }

    GLFWglproc ContextHelper::getProcAddress(cstring procname)
    {
        return glfwGetProcAddress(procname);
    }

    void ContextHelper::setWindowShouldClose(GLFWwindow* window, int value)
    {
        if (window)
            glfwSetWindowShouldClose(window, value);
    }

    int ContextHelper::windowShouldClose(GLFWwindow* window)
    {
        return glfwWindowShouldClose(window);
    }

    void ContextHelper::setWindowIcon(GLFWwindow* window, int count, const GLFWimage* images)
    {
        glfwSetWindowIcon(window, count, images);
    }

    void ContextHelper::setWindowAttrib(GLFWwindow* window, int attrib, int value)
    {
        glfwSetWindowAttrib(window, attrib, value);
    }

    void ContextHelper::setWindowPos(GLFWwindow* window, int x, int y)
    {
        glfwSetWindowPos(window, x, y);
    }

    void ContextHelper::setWindowPos(GLFWwindow* window, math::ivec2 pos)
    {
        glfwSetWindowPos(window, pos.x, pos.y);
    }

    math::ivec2 ContextHelper::getWindowPos(GLFWwindow* window)
    {
        math::ivec2 pos;
        glfwGetWindowPos(window, &pos.x, &pos.y);
        return pos;
    }

    void ContextHelper::destroyWindow(GLFWwindow* window)
    {
        if (initialized())
            glfwDestroyWindow(window);
    }

    math::ivec2 ContextHelper::getFramebufferSize(GLFWwindow* window)
    {
        math::ivec2 size;
        glfwGetFramebufferSize(window, &size.x, &size.y);
        return size;
    }

    void ContextHelper::swapBuffers(GLFWwindow* window)
    {
        OPTICK_EVENT();
        if (initialized())
            glfwSwapBuffers(window);
    }

    void ContextHelper::swapInterval(int interval)
    {
        glfwSwapInterval(interval);
    }

    void ContextHelper::pollEvents()
    {
        glfwPollEvents();
    }

    bool ContextHelper::makeContextCurrent(GLFWwindow* window)
    {
        if (m_initialized.load(std::memory_order_acquire))
        {
            glfwMakeContextCurrent(window);
            return true;
        }
        return false;
    }

    GLFWwindow* ContextHelper::getCurrentContext()
    {
        return glfwGetCurrentContext();
    }

    GLFWkeyfun ContextHelper::setKeyCallback(GLFWwindow* window, GLFWkeyfun callback)
    {
        return glfwSetKeyCallback(window, callback);
    }

    GLFWcharfun ContextHelper::setCharCallback(GLFWwindow* window, GLFWcharfun callback)
    {
        return glfwSetCharCallback(window, callback);
    }

    GLFWcursorposfun ContextHelper::setCursorPosCallback(GLFWwindow* window, GLFWcursorposfun callback)
    {
        return glfwSetCursorPosCallback(window, callback);
    }

    GLFWmousebuttonfun ContextHelper::setMouseButtonCallback(GLFWwindow* window, GLFWmousebuttonfun callback)
    {
        return glfwSetMouseButtonCallback(window, callback);
    }

    GLFWscrollfun ContextHelper::setScrollCallback(GLFWwindow* window, GLFWscrollfun callback)
    {
        return glfwSetScrollCallback(window, callback);
    }

    GLFWdropfun ContextHelper::setDropCallback(GLFWwindow* window, GLFWdropfun callback)
    {
        return glfwSetDropCallback(window, callback);
    }

    GLFWcursorenterfun ContextHelper::setCursorEnterCallback(GLFWwindow* window, GLFWcursorenterfun callback)
    {
        return glfwSetCursorEnterCallback(window, callback);
    }

    GLFWwindowclosefun ContextHelper::setWindowCloseCallback(GLFWwindow* window, GLFWwindowclosefun callback)
    {
        return glfwSetWindowCloseCallback(window, callback);
    }

    GLFWwindowposfun ContextHelper::setWindowPosCallback(GLFWwindow* window, GLFWwindowposfun callback)
    {
        return glfwSetWindowPosCallback(window, callback);
    }

    GLFWwindowsizefun ContextHelper::setWindowSizeCallback(GLFWwindow* window, GLFWwindowsizefun callback)
    {
        return glfwSetWindowSizeCallback(window, callback);
    }

    GLFWwindowrefreshfun ContextHelper::setWindowRefreshCallback(GLFWwindow* window, GLFWwindowrefreshfun callback)
    {
        return glfwSetWindowRefreshCallback(window, callback);
    }

    GLFWwindowfocusfun ContextHelper::setWindowFocusCallback(GLFWwindow* window, GLFWwindowfocusfun callback)
    {
        return glfwSetWindowFocusCallback(window, callback);
    }

    GLFWwindowiconifyfun ContextHelper::setWindowIconifyCallback(GLFWwindow* window, GLFWwindowiconifyfun callback)
    {
        return glfwSetWindowIconifyCallback(window, callback);
    }

    GLFWwindowmaximizefun ContextHelper::setWindowMaximizeCallback(GLFWwindow* window, GLFWwindowmaximizefun callback)
    {
        return glfwSetWindowMaximizeCallback(window, callback);
    }

    GLFWframebuffersizefun ContextHelper::setFramebufferSizeCallback(GLFWwindow* window, GLFWframebuffersizefun callback)
    {
        return glfwSetFramebufferSizeCallback(window, callback);
    }

    GLFWwindowcontentscalefun ContextHelper::setWindowContentScaleCallback(GLFWwindow* window, GLFWwindowcontentscalefun callback)
    {
        return glfwSetWindowContentScaleCallback(window, callback);
    }

    GLFWjoystickfun ContextHelper::setJoystickCallback(GLFWjoystickfun callback)
    {
        return glfwSetJoystickCallback(callback);
    }

    int ContextHelper::getGamepadSate(int jid, GLFWgamepadstate* state)
    {
        if (initialized())
            return glfwGetGamepadState(jid, state);
        return 0;
    }

    void ContextHelper::updateGamepadMappings(const char* name)
    {
        (void)glfwUpdateGamepadMappings(name);
    }

    bool ContextHelper::joystickPresent(int jid)
    {
        return glfwJoystickPresent(jid);
    }

    void ContextHelper::setInputMode(GLFWwindow* window, int mode, int value)
    {
        glfwSetInputMode(window, mode, value);
    }
}
