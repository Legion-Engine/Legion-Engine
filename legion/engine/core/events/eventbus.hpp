#pragma once
#include <core/containers/delegate.hpp>
#include <core/containers/sparse_map.hpp>
#include <core/containers/hashed_sparse_set.hpp>
#include <core/types/types.hpp>
#include <core/events/event.hpp>

#include <Optick/optick.h>

#include <memory>

/**@file eventbus.hpp
 */

namespace legion::core::events
{
    /**@class EventBus
     * @brief Central communication channel for events and messages.
     */
    class EventBus
    {
        sparse_map<id_type, hashed_sparse_set<std::shared_ptr<event_base>>> m_events;
        sparse_map<id_type, multicast_delegate<void(event_base*)>> m_eventCallbacks;

    public:

        /**@brief Insert event into bus and notify all subscribers.
         * @tparam event_type Event type to raise.
         * @param arguments Arguments to pass to the constructor of the event.
         */
        template<typename event_type, typename... Args, typename = inherits_from<event_type, event<event_type>>>
        void raiseEvent(Args&&... arguments)
        {
            OPTICK_EVENT();
            event_type* eventptr;

            event_type event(arguments...); // Create new event.
            if (event.persistent() && !(event.unique() && m_events[event_type::id].size()))
            {
                eventptr = new event_type(std::move(event));
                m_events[event_type::id].emplace(eventptr); // If it's persistent keep the event stored. (Or at least keep it somewhere fetch able.)
            }
            else
                eventptr = &event;

            if (m_eventCallbacks.contains(event_type::id))
            {
                OPTICK_EVENT("Event callbacks");
                OPTICK_TAG("Event", typeName<event_type>());
                force_value_cast<multicast_delegate<void(event_type*)>>(m_eventCallbacks[event_type::id]).invoke(eventptr); // Notify.
            }
        }

        void raiseEvent(std::unique_ptr<event_base>&& value)
        {
            OPTICK_EVENT();
            if (value->persistent() && !(value->unique() && m_events[value->get_id()].size()))
            {
                m_events[value->get_id()].emplace(value.release()); // If it's persistent keep the event stored. (Or at least keep it somewhere fetch able.)
            }

            if (m_eventCallbacks.contains(value->get_id()))
            {
                OPTICK_EVENT("Event callbacks");
                OPTICK_TAG("Event", detail::eventNames[value->get_id()].c_str());
                m_eventCallbacks[value->get_id()].invoke(value.get());
            }
        }

        void raiseEventUnsafe(std::unique_ptr<event_base>&& value, id_type id)
        {
            OPTICK_EVENT();

            if (value->persistent() && !(value->unique() && m_events[id].size()))
            {
                m_events[id].emplace(value.release()); // If it's persistent keep the event stored. (Or at least keep it somewhere fetch able.)
            }

            if (m_eventCallbacks.contains(id))
            {
                OPTICK_EVENT("Event callbacks");
                OPTICK_TAG("Event", detail::eventNames[value->get_id()].c_str());
                m_eventCallbacks[id].invoke(value.get());
            }
        }

        /**@brief Check if an event is active.
         * @tparam event_type Event type to check for.
         */
        template<typename event_type, typename = inherits_from<event_type, event<event_type>>>
        bool checkEvent() const
        {
            OPTICK_EVENT();
            return m_events.contains(event_type::id) && m_events[event_type::id].size();
        }

        /**@brief Get the amount of events/messages that are currently in the bus.
         * @tparam event_type Event type to get the amount of.
         */
        template<typename event_type, typename = inherits_from<event_type, event<event_type>>>
        size_type getEventCount() const
        {
            OPTICK_EVENT();
            if (m_events.contains(event_type::id))
                return m_events[event_type::id].size();
            return 0;
        }

        /**@brief Get a reference to an event/message of a certain index.
         * @tparam event_type Event type to fetch.
         */
        template<typename event_type, typename = inherits_from<event_type, event<event_type>>>
        const event_type& getEvent(index_type index = 0) const
        {
            OPTICK_EVENT();
            if (checkEvent<event_type>())
                return *static_cast<event_type*>(m_events[event_type::id][index]); // Static cast because we already know that the types are the same.
            return nullptr;
        }

        /**@brief Get a reference to the most recently raised event of this type.
         * @tparam event_type Event type to fetch.
         */
        template<typename event_type, typename = inherits_from<event_type, event<event_type>>>
        const event_type& getLastEvent() const
        {
            OPTICK_EVENT();
            if (checkEvent<event_type>())
            {
                size_type size = m_events[event_type::id].size();
                return *static_cast<event_type*>(m_events[event_type::id][size - 1]); // Static cast because we already know that the types are the same.
            }
            return nullptr;
        }

        /**@brief Removes a certain event from the bus.
         * @tparam event_type Event type to clear.
         */
        template<typename event_type, typename = inherits_from<event_type, event<event_type>>>
        void clearEvent(index_type index = 0)
        {
            OPTICK_EVENT();
            if (checkEvent<event_type>())
            {
                auto* event = m_events[event_type::id][index];
                m_events[event_type::id].erase(event);
                delete event;
            }
        }

        /**@brief Removes a the most recent event from the bus.
         * @tparam event_type Event type to clear.
         */
        template<typename event_type, typename = inherits_from<event_type, event<event_type>>>
        void clearLastEvent()
        {
            OPTICK_EVENT();
            if (checkEvent<event_type>())
            {
                auto* event = m_events[event_type::id][m_events[event_type::id].size() - 1];
                m_events[event_type::id].erase(event);
                delete event;
            }
        }

        /**@brief Link a callback to an event type in order to get notified whenever one gets raised.
         * @tparam event_type Event type to subscribe to.
         */
        template<typename event_type, typename = inherits_from<event_type, event<event_type>>>
        void bindToEvent(delegate<void(event_type*)> callback)
        {
            OPTICK_EVENT();
            m_eventCallbacks[event_type::id] += force_value_cast<delegate<void(event_base*)>>(callback);
        }

        void bindToEventUnsafe(id_type id, delegate<void(event_base*)> callback)
        {
            m_eventCallbacks[id] += callback;
        }
    };
}
