/*!
 * \file PersistenceModel.h
 * \brief Persistence model information.
 *
 * A persistence model contains information about the estimated time an item will disappear from the surface.
 * Persistence models are in the granularity of hours.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 28, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_MODEL_PERSISTENCE_MODEL_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_MODEL_PERSISTENCE_MODEL_H_

// worldlib
#include "../world/Item.h"
#include "../world/Surface.h"

// ROS
#include <ros/time.h>

namespace rail
{
namespace spatial_temporal_learning
{
namespace worldlib
{
namespace model
{

/*!
 * \class PersistenceModel
 * \brief Persistence model information.
 *
 * A persistence model contains information about the estimated time an item will disappear from the surface.
 * Persistence models are in the granularity of hours.
 */
class PersistenceModel
{
public:
  /*!
   * \brief Create a new PersistenceModel.
   *
   * Create a new PersistenceModel with the given parameters.
   *
   * \param item The Item of this PersistenceModel.
   * \param surface The Surface of this PersistenceModel.
   * \param lambda The exponential distribution's lambda value of this PersistenceModel.
   * \param last_seen The last time the item was seen.
   */
  PersistenceModel(const world::Item &item, const world::Surface &surface, const double lambda,
      const ros::Time &last_seen);

  /*!
   * \brief Item value accessor (immutable).
   *
   * Get the Item value of this PersistenceModel.
   *
   * \return The Item value of this PersistenceModel.
   */
  const world::Item &getItem() const;

  /*!
   * \brief Item value accessor.
   *
   * Get the Item value of this PersistenceModel.
   *
   * \return The Item value of this PersistenceModel.
   */
  world::Item &getItem();

  /*!
   * \brief Item value mutator.
   *
   * Set the Item value of this PersistenceModel.
   *
   * \param item The new Item value of this PersistenceModel.
   */
  void setItem(const world::Item &item);

  /*!
   * \brief Surface value accessor (immutable).
   *
   * Get the Surface value of this PersistenceModel.
   *
   * \return The Surface value of this PersistenceModel.
   */
  const world::Surface &getSurface() const;

  /*!
   * \brief Surface value accessor.
   *
   * Get the Surface value of this PersistenceModel.
   *
   * \return The Surface value of this PersistenceModel.
   */
  world::Surface &getSurface();

  /*!
   * \brief Surface value mutator.
   *
   * Set the Surface value of this PersistenceModel.
   *
   * \param surface The new Surface value of this PersistenceModel.
   */
  void setSurface(const world::Surface &surface);

  /*!
   * \brief Lambda value accessor.
   *
   * Get the lambda value of this PersistenceModel.
   *
   * \return The lambda value of this PersistenceModel.
   */
  double getLambda() const;

  /*!
   * \brief Lambda value mutator.
   *
   * Set the lambda value of this PersistenceModel.
   *
   * \param lambda The new lambda value of this PersistenceModel.
   */
  void setLambda(const double lambda);

  /*!
   * \brief Last seen time value accessor (immutable).
   *
   * Get the time value of this PersistenceModel.
   *
   * \return The last seen time value of this PersistenceModel.
   */
  const ros::Time &getLastSeen() const;

  /*!
   * \brief Last seen time value accessor.
   *
   * Get the last seen time value of this PersistenceModel.
   *
   * \return The last seen time value of this PersistenceModel.
   */
  ros::Time &getLastSeen();

  /*!
   * \brief Last seen time value mutator.
   *
   * Set the time value of this PersistenceModel.
   *
   * \param last_seen The new last seen time value of this PersistenceModel.
   */
  void setLastSeen(const ros::Time &last_seen);

private:
  /*! The Item value for the model. */
  world::Item item_;
  /*! The Surface value for the model. */
  world::Surface surface_;
  /*! The exponential distribution's lambda value. */
  double lambda_;
  /*! The last seen time the Item was observed on the Surface. */
  ros::Time last_seen_;
};

}
}
}
}

#endif
