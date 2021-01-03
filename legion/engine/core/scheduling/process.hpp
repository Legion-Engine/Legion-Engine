#pragma once
#include <core/platform/platform.hpp>
#include <core/types/types.hpp>
#include <core/containers/containers.hpp>
#include <core/time/time.hpp>

#include <Optick/optick.h>

/**@file process.hpp
 */

namespace legion::core::scheduling
{
    /**@class Process
     * @brief Operation encapsulating object that keeps track of intervals and hooks into process-chains.
     */
    class Process
    {
        friend class ProcessChain;
    private:
        std::string m_name;
        id_type m_nameHash;
        hashed_sparse_set<id_type> m_hooks;

        delegate<void(time::time_span<fast_time>)> m_operation;
        time::time_span<fast_time> m_interval;
        time::time_span<fast_time> m_timeBuffer;
        time::clock<fast_time> m_clock;
        bool m_fixedTimeStep;
        bool firstStep = true;
    public:

        template<size_type charc>
        Process(const char(&name)[charc], time::time_span<fast_time> interval = 0) : m_name(name), m_nameHash(nameHash<charc>(name)) { setInterval(interval); }

        Process(const std::string& name, id_type nameHash, time::time_span<fast_time> interval = 0) : m_name(name), m_nameHash(nameHash) { setInterval(interval); }

        Process() = default;
        Process(Process&&) = default;
        Process& operator=(Process&&) = default;
        ~Process() = default;

        /**@brief Returns the hash of the name of the process.
         */
        id_type id() const { return m_nameHash; }

        bool inUse() const { return m_hooks.size(); }

        /**@brief Set the operation for the process to execute at the set interval.
         */
        void setOperation(delegate<void(time::time_span<fast_time>)>&& operation)
        {
            m_operation = operation;
        }

        /**@brief Set the interval at which to execute the set operation.
         */
        void setInterval(time::time_span<fast_time> interval)
        {
            m_fixedTimeStep = interval != time::time_span<fast_time>::zero();

            m_interval = interval;
        }

        /**@brief Update the process' internal time measurements and execute the operation if necessary.
         * @param timeScale Scalar to scale deltaTime with.
         * @return bool True if the operation was executed and has completed the amount of executions in order to sate the interval, otherwise false.
         */
        bool execute(float timeScale)
        {
            time::time_span<fast_time> deltaTime = m_clock.restart();

            if (firstStep)
            {
                deltaTime = 0;
                firstStep = false;
            }

            deltaTime *= timeScale;

            if (deltaTime < 0)
                deltaTime = 0;

            if (!m_fixedTimeStep)
            {
                OPTICK_EVENT("Execute process");
                OPTICK_TAG("Process", m_name.c_str());
                m_operation.invoke(deltaTime);
                return true;
            }

            m_timeBuffer += deltaTime;

            if (m_timeBuffer >= m_interval)
            {
                OPTICK_EVENT("Execute process");
                OPTICK_TAG("Process", m_name.c_str());
                m_timeBuffer -= m_interval;
                m_operation.invoke(m_interval);
            }

            return true;
        }
    };
}
