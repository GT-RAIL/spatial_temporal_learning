/*!
 * \file World.h
 * \brief World configuration information.
 *
 * A world consists of a series of rooms, surfaces, and items. Surfaces can have points of interest as well.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 22, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_WORLD_WORLD_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_WORLD_WORLD_H_

// worldlib
#include "Item.h"
#include "Room.h"

// C++ Standard Library
#include <string>
#include <vector>

namespace rail
{
namespace spatial_temporal_learning
{
namespace worldlib
{
namespace world
{

/*!
 * \class World
 * \brief World configuration information.
 *
 * A world consists of a series of rooms, surfaces, and items. Surfaces can have points of interest as well.
 */
class World
{
public:
  /*!
   * \brief Create a new World.
   *
   * Creates a new empty World with an optional fixed frame ID.
   *
   * \param fixed_frame_id The fixed frame ID of the World (defaults to the empty string).
   */
  World(const std::string &fixed_frame_id = "");

  /*!
   * \brief Fixed frame ID accessor.
   *
   * Get the fixed frame ID this World.
   *
   * \return The fixed frame ID of this World.
   */
  const std::string &getFixedFrameID() const;

  /*!
   * \brief Fixed frame ID mutator.
   *
   * Set the fixed frame ID this World to the given value.
   *
   * \param The new fixed frame ID value of this World.
   */
  void setFixedFrameID(const std::string &fixed_frame_id);

  /*!
   * \brief Rooms value accessor (immutable).
   *
   * Get the rooms of this World.
   *
   * \return The rooms.
   */
  const std::vector<Room> &getRooms() const;

  /*!
   * \brief Rooms value accessor.
   *
   * Get the rooms of this World.
   *
   * \return The rooms.
   */
  std::vector<Room> &getRooms();

  /*!
   * \brief Rooms size accessor.
   *
   * Get the number of rooms of this World.
   *
   * \return The number of rooms of this World.
   */
  size_t getNumRooms() const;

  /*!
   * \brief Room value accessor (immutable).
   *
   * Get the Room of this World at the given index.
   *
   * \param i The index of the Room to get.
   * \return The Room at the given index.
   * \throws std::out_of_range Thrown if the Room at the given index does not exist.
   */
  const Room &getRoom(const size_t index) const;

  /*!
   * \brief Room value accessor.
   *
   * Get the Room of this World at the given index.
   *
   * \param i The index of the Room to get.
   * \return The Room at the given index.
   * \throws std::out_of_range Thrown if the Room at the given index does not exist.
   */
  Room &getRoom(const size_t index);

  /*!
   * \brief Room adder.
   *
   * Add the Room to this World.
   *
   * \param room The new Room to add.
   */
  void addRoom(const Room &room);

  /*!
   * \brief Room remover.
   *
   * Remove the Room at the given index. An invalid index results in no effect.
   *
   * \param i The index of the Room pose to remove.
   * \throws std::out_of_range Thrown if the Room at the given index does not exist.
   */
  void removeRoom(const size_t index);

  /*!
   * \brief Check for the existence of a Room.
   *
   * Check for the existence of a Room in the World. This will also check the aliases. Case is not important.
   *
   * \param name The name or alias of the Room to find.
   * \throws std::out_of_range Thrown if no Room with the given name exists.
   */
  bool roomExists(const std::string &name) const;

  /*!
   * \brief Room finder (immutable).
   *
   * Find a Room with the given name. This will also check the aliases. Case is not important. If multiple rooms exist
   * with the given name, the first Room is returned.
   *
   * \param name The name or alias of the Room to find.
   * \throws std::out_of_range Thrown if no PointOfInterest with the given name exists.
   */
  const Room &findRoom(const std::string &name) const;

  /*!
   * \brief Room finder.
   *
   * Find a Room with the given name. This will also check the aliases. Case is not important. If multiple rooms exist
   * with the given name, the first Room is returned.
   *
   * \param name The name or alias of the Room to find.
   * \throws std::out_of_range Thrown if no Room with the given name exists.
   */
  Room &findRoom(const std::string &name);

  /*!
   * \brief Items value accessor (immutable).
   *
   * Get the items of this World.
   *
   * \return The items.
   */
  const std::vector<Item> &getItems() const;

  /*!
   * \brief Items value accessor (immutable).
   *
   * Get the items of this World.
   *
   * \return The items.
   */
  std::vector<Item> &getItems();

  /*!
   * \brief Items size accessor.
   *
   * Get the number of items of this World.
   *
   * \return The number of items of this World.
   */
  size_t getNumItems() const;

  /*!
   * \brief Item value accessor (immutable).
   *
   * Get the Item of this World at the given index.
   *
   * \param i The index of the Item to get.
   * \return The Item at the given index.
   * \throws std::out_of_range Thrown if the Item at the given index does not exist.
   */
  const Item &getItem(const size_t index) const;

  /*!
   * \brief Item value accessor.
   *
   * Get the Item of this World at the given index.
   *
   * \param i The index of the Item to get.
   * \return The Item at the given index.
   * \throws std::out_of_range Thrown if the Item at the given index does not exist.
   */
  Item &getItem(const size_t index);

  /*!
   * \brief Item adder.
   *
   * Add the Item to this World.
   *
   * \param item The new Item to add.
   */
  void addItem(const Item &item);

  /*!
   * \brief Item remover.
   *
   * Remove the Item at the given index. An invalid index results in no effect.
   *
   * \param i The index of the Item pose to remove.
   * \throws std::out_of_range Thrown if the Item at the given index does not exist.
   */
  void removeItem(const size_t index);

  /*!
   * \brief Check for the existence of an Item.
   *
   * Check for the existence of an Item in the World. This will also check the aliases. Case is not important.
   *
   * \param name The name or alias of the Item to find.
   * \throws std::out_of_range Thrown if no Room with the given name exists.
   */
  bool itemExists(const std::string &name) const;

  /*!
   * \brief Item finder (immutable).
   *
   * Find an Item with the given name. This will also check the aliases. Case is not important. If multiple items exist
   * with the given name, the first Item is returned.
   *
   * \param name The name or alias of the Item to find.
   * \throws std::out_of_range Thrown if no Item with the given name exists.
   */
  const Item &findItem(const std::string &name) const;

  /*!
   * \brief Item finder.
   *
   * Find an Item with the given name. This will also check the aliases. Case is not important. If multiple items exist
   * with the given name, the first Item is returned.
   *
   * \param name The name or alias of the Item to find.
   * \throws std::out_of_range Thrown if no Item with the given name exists.
   */
  Item &findItem(const std::string &name);

private:
  /*! The fixed frame of the world. */
  std::string fixed_frame_id_;
  /*! Room information for the world. */
  std::vector<Room> rooms_;
  /*! Item information for the world. */
  std::vector<Item> items_;
};

}
}
}
}

#endif
