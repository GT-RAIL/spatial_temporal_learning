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

// Boost
#include <boost/algorithm/string.hpp>

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
    string sql = "CREATE TABLE IF NOT EXISTS `observations` (" \
                   "`id` int(10) unsigned NOT NULL AUTO_INCREMENT, " \
                   "`item_name` varchar(255) NOT NULL, " \
                   "`surface_name` varchar(255) NOT NULL, " \
                   "`surface_frame_id` varchar(255) NOT NULL, " \
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
    string sql = "TRUNCATE TABLE `observations`;";
    // the success will not return a result for truncate (very fast)
    this->query(sql);
  }
}

void SpatialWorldClient::addObservations(const vector<Observation> &observations) const
{
  // add each observation
  for (size_t i = 0; i < observations.size(); i++)
  {
    this->addObservation(observations[i]);
  }
}

void SpatialWorldClient::addObservation(const Item &item, const Surface &surface, const Pose &pose) const
{
  // create an Observation object and add it
  Observation observation(item, surface, pose);
  this->addObservation(observation);
}

void SpatialWorldClient::addObservation(const Observation &observation) const
{
  if (this->connected())
  {
    // to make function calls shorter
    const Item item = observation.getItem();
    const Surface surface = observation.getSurface();
    const Pose pose = observation.getPose();

    // build the SQL statement
    stringstream ss;
    ss << "INSERT INTO `observations` (`item_name`, `surface_name`, `surface_frame_id`, `x`, `y`, `z`, `theta`, "
    << "`time`, `removed_estimate`, `removed_observed`) VALUES ("
    << "\"" << item.getName() << "\", "
    << "\"" << surface.getName() << "\", "
    << "\"" << surface.getFrameID() << "\", "
    << pose.getPosition().getX() << ", "
    << pose.getPosition().getY() << ", "
    << pose.getPosition().getZ() << ", "
    << pose.getOrientation().getTheta() << ", "
    << "FROM_UNIXTIME(" << observation.getTime().sec << ")" << ", "
    << "FROM_UNIXTIME(" << observation.getRemovedEstimate().sec << ")" << ", "
    << "FROM_UNIXTIME(" << observation.getRemovedObserved().sec << ")"
    << ");";

    // execute the query (no result returned for an insert)
    this->query(ss.str());
  } else
  {
    ROS_WARN("Attempted to add an observation when no connection has been made.");
  }
}

void SpatialWorldClient::getObservationsByItemName(const string &item_name,
    vector<SpatialWorldObservation> &observations) const
{
  // build the SQL where clause and ignore case
  string where_clause = "UPPER(item_name)=\"" + boost::to_upper_copy(item_name) + "\"";
  this->getObservationsHelper(observations, where_clause, 0, "`time` DESC");
}

void SpatialWorldClient::getObservationsBySurfaceName(const string &surface_name,
    vector<SpatialWorldObservation> &observations) const
{
  // build the SQL where clause and ignore case
  string where_clause = "UPPER(surface_name)=\"" + boost::to_upper_copy(surface_name) + "\"";
  this->getObservationsHelper(observations, where_clause, 0, "`time` DESC");
}

void SpatialWorldClient::getObservationsBySurfaceFrameID(const string &surface_frame_id,
    vector<SpatialWorldObservation> &observations) const
{
  // build the SQL where clause
  string where_clause = "UPPER(surface_frame_id)=\"" + surface_frame_id + "\"";
  this->getObservationsHelper(observations, where_clause, 0, "`time` DESC");
}

void SpatialWorldClient::updateObservation(const SpatialWorldObservation &observation) const
{
  // create the SQL statement
  stringstream ss;
  ss << "UPDATE `observations` SET "
  << "`id`=" << observation.getID() << ", "
  << "`item_name`='" << observation.getItemName() << "', "
  << "`surface_name`='" << observation.getSurfaceName() << "', "
  << "`surface_frame_id`='" << observation.getSurfaceFrameID() << "', "
  << "`x`=" << observation.getPose().getPosition().getX() << ", "
  << "`y`=" << observation.getPose().getPosition().getY() << ", "
  << "`z`=" << observation.getPose().getPosition().getZ() << ", "
  << "`theta`=" << observation.getPose().getOrientation().getTheta() << ", "
  << "`time`=FROM_UNIXTIME(" << observation.getTime().sec << "), "
  << "`removed_estimate`=FROM_UNIXTIME(" << observation.getRemovedEstimate().sec << "), "
  << "`removed_observed`=FROM_UNIXTIME(" << observation.getRemovedObserved().sec << ") "
  << "WHERE `id`=" << observation.getID() << ";";
  // execute the update (no result returned)
  this->query(ss.str());
}

bool SpatialWorldClient::itemExistsOnSurface(const string &item_name, const string &surface_name) const
{
  if (this->connected())
  {
    // build the SQL where clause
    string where_clause = "((UPPER(item_name)=\"" + item_name + "\") AND "
                          + "(UPPER(surface_name)=\"" + surface_name + "\"))";
    vector<SpatialWorldObservation> observations;
    // attempt to find the latest observation that meets our criteria
    this->getObservationsHelper(observations, where_clause, 1);
    // check if we have our match and if has not been seen leaving
    return observations.size() == 1 && observations[0].getRemovedObserved() == ros::Time(0);
  } else
  {
    ROS_WARN("Attempted to check if a %s exists on the %s when no connection has been made.", item_name.c_str(),
             surface_name.c_str());
    return false;
  }
}

bool SpatialWorldClient::itemObservedOnSurface(const string &item_name, const string &surface_name) const
{
  if (this->connected())
  {
    // build the SQL where clause
    string where_clause = "((UPPER(item_name)=\"" + item_name + "\") AND "
                          + "(UPPER(surface_name)=\"" + surface_name + "\"))";
    vector<SpatialWorldObservation> observations;
    // attempt to find the latest observation that meets our criteria
    this->getObservationsHelper(observations, where_clause, 1);
    // check if we have our match and if has not been seen leaving
    return observations.size() == 1;
  } else
  {
    ROS_WARN("Attempted to check if a %s has have been observed on the %s when no connection has been made.",
             item_name.c_str(), surface_name.c_str());
    return false;
  }
}

void SpatialWorldClient::markObservationsAsRemoved(const std::string &item_name, const std::string &surface_name) const
{
  if (this->connected())
  {
    // check if the item actually is still on the surface
    if (this->itemExistsOnSurface(item_name, surface_name))
    {
      // grab all the entities that have not been marked
      string where_clause = "((UPPER(item_name)=\"" + item_name + "\") AND "
                            + "(UPPER(surface_name)=\"" + surface_name + "\") AND "
                            + "(`removed_observed`='0000-00-00 00:00:00'))";
      vector<SpatialWorldObservation> observations;
      this->getObservationsHelper(observations, where_clause);

      // update the observed removed for everything but the first (we will also update the estimated time there)
      ros::Time now = ros::Time::now();
      for (size_t i = 1; i < observations.size(); i++)
      {
        observations[i].setRemovedObserved(now);
        this->updateObservation(observations[i]);
      }

      // create an estimated time the item was removed using a Guassian distribution
      SpatialWorldObservation &persistent = observations[0];
      SpatialWorldObservation &latest = observations[observations.size() - 1];
      double delta = now.toSec() - persistent.getTime().toSec();
      double mu = delta / 2.0;
      double sigma = delta / 5.0;
      // normal distribution
      boost::normal_distribution<> distribution(mu, sigma);
      boost::variate_generator<boost::mt19937, boost::normal_distribution<> > generator(random_, distribution);
      // randomly seed
      time_t seed;
      time(&seed);
      generator.engine().seed(seed);
      // get the random number
      double rand = max(0.0, min(delta, generator()));

      // set the estimated and observed time and update
      persistent.setRemovedObserved(now);
      persistent.setRemovedEstimate(latest.getTime() + ros::Duration(rand));
      this->updateObservation(persistent);
    } else
    {
      ROS_WARN("Attempted to mark the %s on the %s as removed when it was not still on that surface.",
               item_name.c_str(), surface_name.c_str());
    }
  } else
  {
    ROS_WARN("Attempted to mark the %s on the %s as removed when no connection has been made.",
             item_name.c_str(), surface_name.c_str());
  }
}

void SpatialWorldClient::getObservationsHelper(vector<SpatialWorldObservation> &observations,
    const string &where_clause, const uint32_t limit, const string &order_by) const
{
  if (this->connected())
  {
    // build the SQL
    string sql = "SELECT `id`, `item_name`, `surface_name`, `surface_frame_id`, `x`, `y`, `z`, `theta`,  " \
                 "UNIX_TIMESTAMP(time), UNIX_TIMESTAMP(removed_estimate), UNIX_TIMESTAMP(removed_observed) "\
                 "FROM `observations` WHERE " + where_clause + " ORDER BY " + order_by;
    if (limit > 0)
    {
      stringstream ss;
      ss << sql << " LIMIT " << limit;
      sql = ss.str();
    }
    sql += ";";

    MYSQL_RES *result = this->query(sql);
    if (result != NULL)
    {
      // parse each result
      MYSQL_ROW entity;
      while ((entity = mysql_fetch_row(result)) != NULL)
      {
        // basic information about the observation
        uint32_t id = atol(entity[0]);
        string item_name(entity[1]);
        string surface_name(entity[2]);
        string surface_frame_id(entity[3]);
        // position information
        Position position(atof(entity[4]), atof(entity[5]), atof(entity[6]));
        Orientation orientation(atof(entity[7]));
        Pose pose(position, orientation);
        // time information
        ros::Time time(atol(entity[8]));
        ros::Time removed_estimate(atol(entity[9]));
        ros::Time removed_observed(atol(entity[10]));

        // create and add
        SpatialWorldObservation observation(id, item_name, surface_name, surface_frame_id, pose, time, removed_estimate,
                                            removed_observed);
        observations.push_back(observation);
      }
    } else
    {
      ROS_WARN("Loading observations did not return a result.");
    }
  } else
  {
    ROS_WARN("Attempted to load observations for when no connection has been made.");
  }
}
