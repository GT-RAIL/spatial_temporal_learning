/*!
 * \file SpatialWorldClient.cpp
 * \brief The main MySQL client connection to the spatial world database.
 *
 * The spatial world SQL client can communicate with a MySQL database containing the spatial world database.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 22, 2015
 */

// worldlib
#include "worldlib/remote/SpatialWorldClient.h"

// ROS
#include <ros/ros.h>

using namespace std;
using namespace rail::spatial_temporal_learning::worldlib::geometry;
using namespace rail::spatial_temporal_learning::worldlib::remote;
using namespace rail::spatial_temporal_learning::worldlib::world;

SpatialWorldClient::SpatialWorldClient(const SpatialWorldClient &client) : SqlClient(client)
{
}

SpatialWorldClient::SpatialWorldClient(const string &host, const uint16_t port, const string &user,
    const string &password, const string &database) : SqlClient(host, port, user, password, database)
{
}

bool SpatialWorldClient::connect()
{
  bool success = SqlClient::connect();
  // attempt to create the tables
  this->createTable();
  return success;
}

void SpatialWorldClient::createTable()
{
  if (this->connected())
  {
    // SQL the create the spatial world (sw) table
    string sql = "CREATE TABLE IF NOT EXISTS `sws` (" \
                   "`id` int(10) unsigned NOT NULL AUTO_INCREMENT, " \
                   "`surface` varchar(255) NOT NULL, " \
                   "`surface_frame_id` varchar(255) NOT NULL, " \
                   "`placement_surface_frame_id` varchar(255) NOT NULL, " \
                   "`item` varchar(255) NOT NULL, " \
                   "`x` double NOT NULL, " \
                   "`y` double NOT NULL, " \
                   "`z` double NOT NULL, " \
                   "`theta` double NOT NULL, " \
                   "`time` timestamp NOT NULL DEFAULT CURRENT_TIMESTAMP, " \
                   "`removed_estimate` timestamp NOT NULL DEFAULT '0000-00-00 00:00:00', " \
                   "`removed_observed` timestamp NOT NULL DEFAULT '0000-00-00 00:00:00', " \
                   "PRIMARY KEY (`id`) " \
                 ") ENGINE=InnoDB DEFAULT CHARSET=latin1 AUTO_INCREMENT=1;";

    // the success will not return a result for create table
    this->query(sql);
  }
}

void SpatialWorldClient::clearAllEntities() const
{
  if (this->connected())
  {
    string sql = "TRUNCATE TABLE `sws`;";
    // the success will not return a result for truncate (very fast)
    this->query(sql);
  }
}

void SpatialWorldClient::addObservation(const Item &item, const Surface &surface, const Pose &pose) const
{
  if (this->connected())
  {
    // check if we have a placement surface
    string placement_surface_frame_id;
    if (surface.getNumPlacementSurfaces() > 0)
    {
      const PlacementSurface closest = surface.findClosestPlacementSurface(pose.getPosition());
      placement_surface_frame_id = closest.getFrameID();
    }

    // build the SQL statement
    stringstream ss;
    ss << "INSERT INTO `sws` "
    << "(`surface`, `surface_frame_id`, `placement_surface_frame_id`, `item`, `x`, `y`, `z`, `theta`)"
    << "VALUES ("
    << "\"" << surface.getName() << "\", "
    << "\"" << surface.getFrameID() << "\", "
    << "\"" << placement_surface_frame_id << "\", "
    << "\"" << item.getName() << "\", "
    << pose.getPosition().getX() << ", "
    << pose.getPosition().getY() << ", "
    << pose.getPosition().getZ() << ", "
    << pose.getOrientation().getTheta()
    << ");";

    // execute the query (no result returned for an insert)
    this->query(ss.str());
  } else
  {
    ROS_WARN("Attempted to add an observation when no connection has been made.");
  }
}
