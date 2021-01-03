#pragma once
#include <core/async/async.hpp>
#include <core/containers/containers.hpp>
#include <core/types/primitives.hpp>
#include <core/ecs/entityquery.hpp>
#include <core/ecs/archetype.hpp>

/**
 * @file queryregistry.hpp
 */

namespace legion::core::ecs
{
    class EcsRegistry;
    class entity_handle;

    using entity_set = hashed_sparse_set<entity_handle>;

    /**@class QueryRegistry
     * @brief Main manager and owner of all queries and query related objects.
     */
    class QueryRegistry
    {
        friend class EntityQuery;
    private:
        EcsRegistry& m_registry;
        mutable async::rw_spinlock m_entityLock;
        sparse_map<id_type, std::unique_ptr<entity_set>> m_entityLists;
        mutable async::rw_spinlock m_referenceLock;
        sparse_map<id_type, size_type> m_references;
        mutable async::rw_spinlock m_componentLock;
        sparse_map<id_type, hashed_sparse_set<id_type>> m_componentTypes;

        id_type m_lastQueryId = 1;

        static hashed_sparse_set<QueryRegistry*> m_validRegistries;

        /**@brief Add a new combination of components to query for.
         * @param componentTypes Type ids of all components to query for.
         * @return Query id of the newly added query.
         * @note A query for your combination of components might already exist. Please use createQuery() to prevent duplicates.
         * @note Already adds 1 to the reference count of the returned id.
         */
        id_type addQuery(const hashed_sparse_set<id_type>& componentTypes);

    public:
        static bool isValid(QueryRegistry* reg) { return m_validRegistries.contains(reg); }

        QueryRegistry(EcsRegistry& registry) : m_registry(registry), m_entityLists(), m_componentTypes() { m_validRegistries.insert(this); }

        ~QueryRegistry()
        {
            m_validRegistries.erase(this);
        }

        /**@brief Add a certain component type to a query.
         * @warning Changing the components for this query id will also change it for any other references that may not want the same change.
         *			For safety first check the amount of references to this query id and only change it if there's only 1. otherwise generate a new query.
         * @warning A query with the components you want might already exist, adding them to this query might create duplicates.
         *			For safety and efficiency first check if there isn't another query already using getQueryId.
         * @note Changes components to query for all other queries that used to query the same components.
         * @note Not the fastest thing in the world, don't do it with loads of components. In that case create a new query instead.
         */
        void addComponentType(id_type queryId, id_type componentTypeId);

        /**@brief Remove a certain component type to a query.
         * @warning Changing the components for this query id will also change it for any other references that may not want the same change.
         *			For safety first check the amount of references to this query id and only change it if there's only 1. otherwise generate a new query.
         * @warning A query with the components you want might already exist, removing others from this query might create duplicates.
         *			For safety and efficiency first check if there isn't another query already using getQueryId.
         * @note Changes components to query for all other queries that used to query the same components.
         * @note Not the fastest thing in the world, don't do it with loads of components. In that case create a new query instead.
         */
        void removeComponentType(id_type queryId, id_type componentTypeId);

        /**@brief Mark a change in component composition of an entity.
         * @param entityId Id of entity in question.
         * @param componentTypeId Type id of component that was added or removed.
         * @param removal Whether the component was added or removed.
         */
        void evaluateEntityChange(id_type entityId, id_type componentTypeId, bool removal);

        /**@brief Mark an entity destruction. (removes entity from all queries.
         * @param entityId Id of the entity in question.
         */
        void markEntityDestruction(id_type entityId);

        /**@brief Get query id of a query that requests a certain component combination.
         * @param componentTypes Sparse map containing all component type ids that would need to be queried.
         * @return id_type Id of the matching query or invalid_id if none was found.
         */
        id_type getQueryId(const hashed_sparse_set<id_type>& componentTypes);

        /**@brief Creates an entity query for a certain component combination.
         * @note Will not always create a new query id. If another query exists with the same component type the query handle will get the same id as that one.
         * @tparam component_types Types of components that need to be queried by the new query.
         * @return EntityQuery Query handle with the id that will query the requested component types.
         */
        template<typename... component_types>
        EntityQuery createQuery()
        {
            hashed_sparse_set<id_type> componentTypeIds;
            (createQuerySingle<component_types>(componentTypeIds), ...);
            return createQuery(componentTypeIds);
        }
    private:
        template <typename T>
        void createQuerySingle(hashed_sparse_set<id_type>& h)
        {
            if constexpr (std::is_base_of_v<archetype_base,std::remove_all_extents_t<std::decay_t<T>>>)
            {
                createQueryArchetype<T>(h,std::make_index_sequence<std::tuple_size<typename T::handleGroup>::value>{});
            } else
            {
                h.insert(typeHash<T>());
            }
        }

        template <typename T,size_t ... I>
        void createQueryArchetype(hashed_sparse_set<id_type>& h,std::index_sequence<I...>)
        {
            (h.insert(typeHash<typename ch_yield_type<std::tuple_element_t<I,typename T::handleGroup>>::type>()),...);
        }

    public:
        /**@brief Creates an entity query for a certain component combination.
         * @note Will not always create a new query id. If another query exists with the same component type the query handle will get the same id as that one.
         * @param componentTypes Sparse map with type ids of components that need to be queried by the new query.
         * @return EntityQuery Query handle with the id that will query the requested component types.
         */
        EntityQuery createQuery(const hashed_sparse_set<id_type>& componentTypes);

        /**@brief Gets component types a query with a certain id would query for.
         * @param queryId Id of the query to check.
         */
        const hashed_sparse_set<id_type>& getComponentTypes(id_type queryId);

        /**@brief Get the ids and handles of all entities that have all components of the query.
         * @param queryId Id of the query to get the entities from.
         * @return sparse_map<id_type, entity_handle>& Sparse map with the ids as the key and the handles as the value.
         */
        entity_set getEntities(id_type queryId) const;

        /**@brief Add to reference count of a query.
         * @param queryId Id of query to increase reference count of.
         */
        void addReference(id_type queryId);

        /**@brief Subtract from reference count of a query.
         * @param queryId Id of query to decrease reference count of.
         */
        void removeReference(id_type queryId);

        /**@brief Get reference count of a certain query.
         * @param queryId Id of query to fetch reference count of.
         */
        size_type getReferenceCount(id_type queryId);
    };

}
