/*!
 * \file OfflineItemSearcher.h
 * \brief An offline object search example node.
 *
 * The offline item search is an example item search node. This node will fake observation data and build models from
 * the faked data. This is useful for model verification methods and data collection for ongoing research.
 *
 * Note that this node will clear the spatial world database several times. You should only run this node on a test
 * database, not your robot's persistent database.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 27, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_OFFLINE_ITEM_SEARCHER_H_
#define SPATIAL_TEMPORAL_LEARNING_OFFLINE_ITEM_SEARCHER_H_

// worldlib
#include "worldlib/remote/Node.h"
#include "worldlib/model/TaskModel.h"
#include "worldlib/world/World.h"

// C++ Standard Library
#include <vector>

namespace rail
{
namespace spatial_temporal_learning
{

/*!
 * \class OfflineItemSearcher
 * \brief An offline object search example node.
 *
 * The offline item search is an example item search node. This node will fake observation data and build models from
 * the faked data. This is useful for model verification methods and data collection for ongoing research.
 */
class OfflineItemSearcher : public worldlib::remote::Node
{
public:
  /*!
   * \brief Create a OfflineItemSearcher and associated ROS information.
   *
   * Creates the ROS node handle and creates clients to the worldlib databases.
   */
  OfflineItemSearcher();

  /*!
   * \brief Cleans up a OfflineItemSearcher.
   *
   * Cleans up any connections used by the OfflineItemSearcher.
   */
  virtual ~OfflineItemSearcher();

  /*!
   * \brief Run the offline item search process.
   *
   * Run the simulated item search process and print out the output.
   */
  void run() const;

private:
  /*!
   * \brief Load GeoLife PLT log files.
   *
   * Load GeoLife trajectory files as a means of model verification testing. This will load all PLT files from the
   * given directory and place them in the searcher's vector to utilize later.
   *
   * \param directory The directory to search for PLT files in.
   */
  void loadGeoLife(const std::string &directory);

  /*!
   * \brief Print the list of items to standard out.
   *
   * Prints the given list of items to standard out.
   *
   * \param items The item list to print.
   */
  void printItemList(const std::vector<worldlib::world::Item> &objects) const;

  /*!
   * \brief Print the list of surfaces to standard out.
   *
   * Prints the given list of surfaces to standard out.
   *
   * \param surfaces The surface list to print.
   */
  void printSurfaceList(const std::vector<worldlib::world::Surface> &surfaces) const;

  /*! The interactive world model client */
  worldlib::remote::InteractiveWorldModelClient *interactive_world_model_client_;
  /*! The spatial world database client */
  worldlib::remote::SpatialWorldClient *spatial_world_client_;

  /*! Our "worlds". */
  worldlib::world::World world_, interactive_world_;

  /*! The interactive world task model for putting items away. */
  worldlib::model::TaskModel interactive_world_task_model_;
};

}
}

#endif