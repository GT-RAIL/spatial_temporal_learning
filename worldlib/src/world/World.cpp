/*!
 * \file World.cpp
 * \brief World configuration information.
 *
 * A world consists of a series of rooms, surfaces, and items. Surfaces can have points of interest as well.
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

vector<Room> &World::getRooms()
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

Room &World::getRoom(const size_t index)
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

bool World::roomExists(const string &name) const
{
  // check if we can find it and catch any exceptions
  try
  {
    this->findRoom(name);
    return true;
  } catch (out_of_range &e)
  {
    return false;
  }
}

const Room &World::findRoom(const string &name) const
{
  // check each surface
  for (size_t i = 0; i < rooms_.size(); i++)
  {
    // perform a check
    if (rooms_[i].checkName(name))
    {
      return rooms_[i];
    }
  }
  // no match found
  throw out_of_range("World::findRoom : Room name does not exist.");
}


Room &World::findRoom(const string &name)
{
  // check each surface
  for (size_t i = 0; i < rooms_.size(); i++)
  {
    // perform a check
    if (rooms_[i].checkName(name))
    {
      return rooms_[i];
    }
  }
  // no match found
  throw out_of_range("World::findRoom : Room name does not exist.");
}

const vector<Item> &World::getItems() const
{
  return items_;
}

vector<Item> &World::getItems()
{
  return items_;
}

size_t World::getNumItems() const
{
  return items_.size();
}

const Item &World::getItem(const size_t index) const
{
  // check the index value first
  if (index < items_.size())
  {
    return items_[index];
  } else
  {
    throw out_of_range("World::getItem : Item index does not exist.");
  }
}

Item &World::getItem(const size_t index)
{
  // check the index value first
  if (index < items_.size())
  {
    return items_[index];
  } else
  {
    throw out_of_range("World::getItem : Item index does not exist.");
  }
}

void World::addItem(const Item &item)
{
  items_.push_back(item);
}

void World::removeItem(const size_t index)
{
  // check the index value first
  if (index < items_.size())
  {
    items_.erase(items_.begin() + index);
  } else
  {
    throw out_of_range("World::removeItem : Item index does not exist.");
  }
}

bool World::itemExists(const string &name) const
{
  // check if we can find it and catch any exceptions
  try
  {
    this->findItem(name);
    return true;
  } catch (out_of_range &e)
  {
    return false;
  }
}

const Item &World::findItem(const string &name) const
{
  // check each surface
  for (size_t i = 0; i < items_.size(); i++)
  {
    // perform a check
    if (items_[i].checkName(name))
    {
      return items_[i];
    }
  }
  // no match found
  throw out_of_range("World::findItem : Item name does not exist.");
}


Item &World::findItem(const string &name)
{
  // check each surface
  for (size_t i = 0; i < items_.size(); i++)
  {
    // perform a check
    if (items_[i].checkName(name))
    {
      return items_[i];
    }
  }
  // no match found
  throw out_of_range("World::findItem : Item name does not exist.");
}
