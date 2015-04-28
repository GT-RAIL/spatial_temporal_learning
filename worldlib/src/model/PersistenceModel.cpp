/*!
 * \file PersistenceModel.cpp
 * \brief Persistence model information.
 *
 * A persistence model contains information about the estimated time an item will disappear from the surface.
 * Persistence models are in the granularity of hours.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 28, 2015
 */

// worldlib
#include "worldlib/model/PersistenceModel.h"

using namespace std;
using namespace rail::spatial_temporal_learning::worldlib::model;
using namespace rail::spatial_temporal_learning::worldlib::world;

PersistenceModel::PersistenceModel(const Item &item, const Surface &surface, const double lambda,
    const ros::Time &last_seen) : item_(item), surface_(surface), last_seen_(last_seen)
{
  lambda_ = lambda;
}

const Item &PersistenceModel::getItem() const
{
  return item_;
}

Item &PersistenceModel::getItem()
{
  return item_;
}

void PersistenceModel::setItem(const Item &item)
{
  item_ = item;
}

const Surface &PersistenceModel::getSurface() const
{
  return surface_;
}

Surface &PersistenceModel::getSurface()
{
  return surface_;
}

void PersistenceModel::setSurface(const Surface &surface)
{
  surface_ = surface;
}

double PersistenceModel::getLambda() const
{
  return lambda_;
}

void PersistenceModel::setLambda(const double lambda)
{
  lambda_ = lambda;
}

const ros::Time &PersistenceModel::getLastSeen() const
{
  return last_seen_;
}

ros::Time &PersistenceModel::getLastSeen()
{
  return last_seen_;
}

void PersistenceModel::setLastSeen(const ros::Time &last_seen)
{
  last_seen_ = last_seen;
}
