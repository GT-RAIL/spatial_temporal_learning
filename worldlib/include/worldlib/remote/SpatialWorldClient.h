/*!
 * \file SpatialWorldClient.h
 * \brief The main MySQL client connection to the spatial world database.
 *
 * The spatial world SQL client can communicate with a MySQL database containing the spatial world database.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 22, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_REMOTE_SPATIAL_WORLD_CLIENT_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_REMOTE_SPATIAL_WORLD_CLIENT_H_

// worldlib
#include "SqlClient.h"
#include "../geometry/Pose.h"
#include "../world/Item.h"
#include "../world/Surface.h"

namespace rail
{
namespace spatial_temporal_learning
{
namespace worldlib
{
namespace remote
{

/*!
 * \class SpatialWorldClient
 * \brief The main MySQL client connection to the spatial world database.
 *
 * The spatial world SQL client can communicate with a MySQL database containing the spatial world database.
 */
class SpatialWorldClient : public SqlClient
{
public:
  /*!
   * \brief Create a new SpatialWorldClient.
   *
   * Creates a new Client by copying the values from the given SpatialWorldClient. A new connection is made if one
   * exists.
   *
   * \param client The SpatialWorldClient to copy.
   */
  SpatialWorldClient(const SpatialWorldClient &client);

  /*!
   * \brief Create a new SpatialWorldClient.
   *
   * Creates a new SpatialWorldClient with the given connection information. A connection is not made by default.
   *
   * \param host The host of the database.
   * \param port The host port of the database.
   * \param user The user of the database.
   * \param password The password for the user of the database.
   * \param database The database name.
   */
  SpatialWorldClient(const std::string &host, const uint16_t port, const std::string &user, const std::string &password,
      const std::string &database);

  /*!
   * \brief Create a connection to the spatial world database.
   *
   * Attempts to create a connection to the spatial world database. Tables are created if they do not exist yet. A flag
   * is returned to indicate the success.
   *
   * \return True if a connection has been successfully made.
   */
  virtual bool connect();

  /*!
   * \brief Clear all entries in the spatial world database.
   *
   * Attempts to clear all entries in the spatial world database. If no connection is made, no effect is made.
   */
  void clearAllEntities() const;

  /*!
   * \brief Add a new observation to the spatial world database.
   *
   * Attempts to add an observation to the spatial world database. The placement surface will be set to the closest
   * PlacementSurface in the Surface if any exist. If no connection is made, no effect is made.
   *
   * \param item The Item observed in the world.
   * \param surface The Surface the item was observed on.
   * \param pose The Pose of the Item with respect to the surface.
   */
  void addObservation(const world::Item &item, const world::Surface &surface, const geometry::Pose &pose) const;

private:
  /*!
   * \brief Create the spatial world table (sws).
   *
   * Attempts to create the spatial world table (sws). If no connection is made, no effect is made.
   */
  void createTable();
};

}
}
}
}

#endif
