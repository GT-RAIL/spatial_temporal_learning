/*!
 * \file World.h
 * \brief World configuration information.
 *
 * A world consists of a series of rooms and surfaces. Surfaces can have points of interest as well.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 22, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_WORLD_WORLD_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_WORLD_WORLD_H_

// worldlib
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
 * A world consists of a series of rooms and surfaces. Surfaces can have points of interest as well.
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
   * \brief Rooms value accessor.
   *
   * Get the rooms of this World.
   *
   * \return The rooms.
   */
  const std::vector<Room> &getRooms() const;

  /*!
   * \brief Rooms size accessor.
   *
   * Get the number of rooms of this World.
   *
   * \return The number of rooms of this World.
   */
  size_t getNumRooms() const;

  /*!
   * \brief Room value accessor.
   *
   * Get the Room of this World at the given index.
   *
   * \param i The index of the Room to get.
   * \return The Room at the given index.
   * \throws std::out_of_range Thrown if the Room at the given index does not exist.
   */
  const Room &getRoom(const size_t index) const;

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

private:
  /*! The fixed frame of the world. */
  std::string fixed_frame_id_;
  /*! Room information for the world. */
  std::vector<Room> rooms_;
};

}
}
}
}

#endif
