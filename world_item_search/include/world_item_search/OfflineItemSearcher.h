/*!
 * \file OfflineItemSearcher.h
 * \brief An offline object search example node.
 *
 * The offline item search is an example item search node. This node will fake observation data and build models from
 * the faked data. This is useful for model verification methods and data collection for ongoing research.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 27, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_OFFLINE_ITEM_SEARCHER_H_
#define SPATIAL_TEMPORAL_LEARNING_OFFLINE_ITEM_SEARCHER_H_

// worldlib
#include "worldlib/remote/Node.h"
#include "../../../worldlib/include/worldlib/remote/SpatialWorldClient.h"

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
   * \brief A check for a valid OfflineItemSearcher.
   *
   * This function will return true if the appropriate connections were created successfully during initialization.
   *
   * \return True if the appropriate connections were created successfully during initialization.
   */
  bool okay() const;

private:
  /*! The interactive world model client */
  worldlib::remote::InteractiveWorldModelClient *interactive_world_model_client_;
  /*! The spatial world database client */
  worldlib::remote::SpatialWorldClient *spatial_world_client_;
  /*! The okay check flag. */
  bool okay_;
};

}
}

#endif