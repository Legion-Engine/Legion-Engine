#pragma once
#include <core/types/types.hpp>
#include <core/containers/sparse_set.hpp>
#include <core/containers/sparse_map.hpp>

/**
 * @file entity.hpp
 */

namespace args::core::ecs
{
	class ARGS_API EcsRegistry;

	class component_handle_base;

	template<typename component_type>
	class component_handle;

	/**@class entity
	 * @brief Serializable handle for executing operations on entities.
	 *		  This class only stores a reference to the registry and the id of the entity.
	 */

	class ARGS_API entity
	{
		friend class EcsRegistry;
	private:
		id_type m_id;
		EcsRegistry* m_registry;

	public:
		/**@brief Main constructor for constructing a valid entity handle.
		 */
		entity(id_type id, EcsRegistry* registry) : m_id(id) { m_registry = registry; }

		/**@brief Constructor for constructing an invalid entity handle.
		 * @note Should only be used to create temporary handles. Allows use of entity handle in containers together with copy constructor.
		 */
		entity() : m_id(invalid_id) { m_registry = nullptr; }

		/**@brief Copy constructor (DOES NOT CREATE NEW ENTITY, both handles will reference the same entity).
		 * @note Allows use of entity handle in containers together with default invalid entity constructor.
		 */
		entity(const entity& other) : m_id(other.m_id) { m_registry = other.m_registry; }

		/**@brief Copy assignment. Exists for the same reasons as the copy constructor.
		 * @ref args::core::ecs::entity::entity(const args::core::ecs::entity& other)
		 */
		entity& operator=(const entity& other);

		/**@brief Returns the type id's of the components this entity contains.
		 * @returns sparse_map<id_type, id_type>& Sparse map with component type id as both the key as well as the value. (behaves as sparse_set with hash table)
		 * @throws args_invalid_entity_error Thrown when handle's registry reference is invalid.
		 * @throws args_entity_not_found_error Thrown when handle's id is invalid.
		 */
		A_NODISCARD sparse_map<id_type, id_type>& component_composition();

		/**@brief Cast to id_type returns the id of the entity this handle references.
		 * @returns id_type If the handle is valid it will return the entity id, otherwise invalid_id.
		 */
		operator id_type() const { return get_id(); }

		/**@brief Returns the id of the entity this handle references.
		 * @returns id_type If the handle is valid it will return the entity id, otherwise invalid_id.
		 */
		A_NODISCARD id_type get_id() const;

		/**@brief Iterator to first child entity. (dereferences to entity handle)
		 * @throws args_invalid_entity_error Thrown when handle's registry reference is invalid.
		 * @throws args_entity_not_found_error Thrown when handle's id is invalid.
		 */
		A_NODISCARD sparse_map<id_type, entity>::iterator begin();

		/**@brief Const iterator to first child entity. (dereferences to entity handle)
		 * @throws args_invalid_entity_error Thrown when handle's registry reference is invalid.
		 * @throws args_entity_not_found_error Thrown when handle's id is invalid.
		 */
		A_NODISCARD sparse_map<id_type, entity>::const_iterator begin() const;

		/**@brief Iterator to last child entity. (dereferences to entity handle)
		 * @throws args_invalid_entity_error Thrown when handle's registry reference is invalid.
		 * @throws args_entity_not_found_error Thrown when handle's id is invalid.
		 */
		A_NODISCARD sparse_map<id_type, entity>::iterator end();

		/**@brief Const iterator to last child entity. (dereferences to entity handle)
		 * @throws args_invalid_entity_error Thrown when handle's registry reference is invalid.
		 * @throws args_entity_not_found_error Thrown when handle's id is invalid.
		 */
		A_NODISCARD sparse_map<id_type, entity>::const_iterator end() const;

		/**@brief Returns entity handle to parent entity.
		 * @returns entity Entity handle that either points to the parent entity or is invalid if the entity doesn't have a parent.
		 * @throws args_invalid_entity_error Thrown when handle's registry reference is invalid.
		 * @throws args_entity_not_found_error Thrown when handle's id is invalid.
		 */
		A_NODISCARD entity get_parent() const;

		/**@brief Set parent of entity.
		 * @param newParent Id of the entity you wish to set as the parent. (invalid_id if you wish to remove parent)
		 * @note Will remove as child from parent.
		 * @throws args_invalid_entity_error Thrown when handle's registry reference is invalid.
		 * @throws args_entity_not_found_error Thrown when handle's id is invalid.
		 */
		void set_parent(id_type newParent);

		/**@brief Get child of the entity at a certain index.
		 * @throws std::out_of_range Thrown when index is more than or equal to the child count.
		 * @throws args_invalid_entity_error Thrown when handle's registry reference is invalid.
		 * @throws args_entity_not_found_error Thrown when handle's id is invalid.
		 */
		A_NODISCARD entity operator[] (index_type index) const;

		/**@brief Get child of the entity at a certain index.
		 * @throws std::out_of_range Thrown when index is more than or equal to the child count.
		 * @throws args_invalid_entity_error Thrown when handle's registry reference is invalid.
		 * @throws args_entity_not_found_error Thrown when handle's id is invalid.
		 */
		A_NODISCARD entity get_child(index_type index) const;

		/**@brief Get amount of children the referenced entity has.
		 * @returns size_type
		 * @throws args_invalid_entity_error Thrown when handle's registry reference is invalid.
		 * @throws args_entity_not_found_error Thrown when handle's id is invalid.
		 */
		A_NODISCARD size_type child_count() const;

		/**@brief Add child to entity.
		 * @param childId Id of the entity you wish to add as a child.
		 * @note Will change parent of the entity to add as child.
		 * @throws args_invalid_entity_error Thrown when handle's registry reference is invalid.
		 * @throws args_entity_not_found_error Thrown when handle's id is invalid.
		 */
		void add_child(id_type childId);

		/**@brief Remove child from entity.
		 * @param childId Id of the entity you wish to remove as a child.
		 * @note Will remove parent of the entity to remove as child.
		 * @throws args_invalid_entity_error Thrown when handle's registry reference is invalid.
		 * @throws args_entity_not_found_error Thrown when handle's id is invalid.
		 */
		void remove_child(id_type childId);

		/**@brief Check if entity contains a certain component.
		 * @tparam component_type Type of component to check for.
		 * @throws args_invalid_entity_error Thrown when handle's registry reference is invalid.
		 * @throws args_entity_not_found_error Thrown when handle's id is invalid.
		 */
		template<typename component_type>
		A_NODISCARD bool has_component() const { return has_component(typeHash<component_type>()); }

		/**@brief Check if entity contains a certain component.
		 * @param componentTypeId Type id of component to check for.
		 * @throws args_invalid_entity_error Thrown when handle's registry reference is invalid.
		 * @throws args_entity_not_found_error Thrown when handle's id is invalid.
		 */
		A_NODISCARD bool has_component(id_type componentTypeId) const;

		/**@brief Get component handle of a certain component.
		 * @param componentTypeId Type id of component to fetch.
		 * @throws args_invalid_entity_error Thrown when handle's registry reference is invalid.
		 * @throws args_entity_not_found_error Thrown when handle's id is invalid.
		 * @returns component_handle_base Valid component handle if the entity has the component, invalid handle if the entity doesn't have the component.
		 * @note component_handle_base needs to be force_cast to component_handle<T> in order to be usable.
		 */
		A_NODISCARD component_handle_base get_component(id_type componentTypeId) const;

		/**@brief Get component handle of a certain component.
		 * @tparam component_type Type of component to fetch.
		 * @throws args_invalid_entity_error Thrown when handle's registry reference is invalid.
		 * @throws args_entity_not_found_error Thrown when handle's id is invalid.
		 * @returns component_handle<component_type> Valid component handle if the entity has the component, invalid handle if the entity doesn't have the component.
		 */
		template<typename component_type>
		A_NODISCARD component_handle<component_type> get_component() const
		{
			return force_value_cast<component_handle<component_type>>(get_component(typeHash<component_type>()));
		}

		/**@brief Add component to the entity.
		 * @param componentTypeId Type id of component to add.
		 * @throws args_invalid_entity_error Thrown when handle's registry reference is invalid.
		 * @throws args_entity_not_found_error Thrown when handle's id is invalid.
		 * @throws args_unknown_component_error Thrown when the component type is unknown.
		 * @returns component_handle_base Valid component handle base for the newly created component.
		 * @note component_handle_base needs to be force_cast to component_handle<T> in order to be usable.
		 */
		component_handle_base add_component(id_type componentTypeId);

		/**@brief Add component to the entity.
		 * @tparam component_type Type of component to add.
		 * @throws args_invalid_entity_error Thrown when handle's registry reference is invalid.
		 * @throws args_entity_not_found_error Thrown when handle's id is invalid.
		 * @throws args_unknown_component_error Thrown when the component type is unknown.
		 * @returns component_handle<component_type> Valid component handle for the newly created component.
		 */
		template<typename component_type>
		component_handle<component_type> add_component()
		{
			return force_value_cast<component_handle<component_type>>(add_component(typeHash<component_type>()));
		}

		/**@brief Remove component from entity.
		 * @param componentTypeId Type id of component to remove.
		 * @throws args_invalid_entity_error Thrown when handle's registry reference is invalid.
		 * @throws args_entity_not_found_error Thrown when handle's id is invalid.
		 * @throws args_unknown_component_error Thrown when the component type is unknown.
		 * @note Nothing will happen if the entity doesn't have this component.
		 */
		void remove_component(id_type componentTypeId);

		/**@brief Remove component from entity.
		 * @tparam component_type Type of component to remove.
		 * @throws args_invalid_entity_error Thrown when handle's registry reference is invalid.
		 * @throws args_entity_not_found_error Thrown when handle's id is invalid.
		 * @throws args_unknown_component_error Thrown when the component type is unknown.
		 * @note Nothing will happen if the entity doesn't have this component.
		 */
		template<typename component_type>
		void remove_component()
		{
			remove_component(typeHash<component_type>());
		}

		/**@brief Destroy this entity. Destroys entity and invalidates handle. (also destroys all of it's components)
		 * @param recurse Destroy all children and children of children as well? Default value is true.
		 */
		void destroy(bool recurse = true);

		/**@brief Check whether this entity handle is valid or not.
		 * @returns bool True if the handle is pointing to valid entity and the registry reference is also valid.
		 */
		bool valid() const;
	};
}