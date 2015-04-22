/*!
 * \file World.cpp
 * \brief World configuration information.
 *
 * A world consists of a series of rooms and surfaces. Surfaces can have points of interest as well.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 22, 2015
 */

// worldlib
#include "worldlib/world/World.h"

using namespace std;
using namespace rail::spatial_temporal_learning::worldlib::world;

World::World(const string &fixed_frame_id) : fixed_frame_id_(fixed_frame_id)
{
}

const string &World::getFixedFrameID() const
{
  return fixed_frame_id_;
}

void World::setFixedFrameID(const string &fixed_frame_id)
{
  fixed_frame_id_ = fixed_frame_id;
}

const vector<Room> &World::getRooms() const
{
  return rooms_;
}

size_t World::getNumRooms() const
{
  return rooms_.size();
}

const Room &World::getRoom(const size_t index) const
{
  // check the index value first
  if (index < rooms_.size())
  {
    return rooms_[index];
  } else
  {
    throw out_of_range("World::getRoom : Room index does not exist.");
  }
}

void World::addRoom(const Room &room)
{
  rooms_.push_back(room);
}

void World::removeRoom(const size_t index)
{
  // check the index value first
  if (index < rooms_.size())
  {
    rooms_.erase(rooms_.begin() + index);
  } else
  {
    throw out_of_range("World::removeRoom : Room index does not exist.");
  }
}
