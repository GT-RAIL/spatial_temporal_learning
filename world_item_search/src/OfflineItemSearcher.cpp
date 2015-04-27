/*!
 * \file OfflineItemSearcher.cpp
 * \brief An offline object search example node.
 *
 * The offline item search is an example item search node. This node will fake observation data and build models from
 * the faked data. This is useful for model verification methods and data collection for ongoing research.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 27, 2015
 */

// World Item Search
#include "world_item_search/OfflineItemSearcher.h"

// ROS
#include <ros/ros.h>

using namespace rail::spatial_temporal_learning;

OfflineItemSearcher::OfflineItemSearcher() : worldlib::remote::Node()
{
  // create the clients we need
  interactive_world_model_client_ = this->createInteractiveWorldModelClient();
  spatial_world_client_ = this->createSpatialWorldClient();

  // attempt to connect
  okay_ = spatial_world_client_->connect();

  if (okay_)
  {
    ROS_INFO("Offline Item Searcher Initialized");
  }
}


OfflineItemSearcher::~OfflineItemSearcher()
{
  // clean up clients
  delete interactive_world_model_client_;
  delete spatial_world_client_;
}

bool OfflineItemSearcher::okay() const
{
  return okay_;
}
