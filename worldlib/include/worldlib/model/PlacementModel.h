/*!
 * \file PlacementModel.h
 * \brief Placement model information.
 *
 * A placement model contains information about the strength of an item in reference to some object.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 25, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_MODEL_PLACEMENT_MODEL_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_MODEL_PLACEMENT_MODEL_H_

// worldlib
#include "../world/Placement.h"

namespace rail
{
namespace spatial_temporal_learning
{
namespace worldlib
{
namespace model
{

/*!
 * \class PlacementModel
 * \brief Placement model information.
 *
 * A placement model contains information about the strength of an item in reference to some object.
 */
class PlacementModel
{
public:
  /*!
   * \brief Create a new PlacementModel.
   *
   * Create a new PlacementModel with the given parameters.
   *
   * \param placement The Placement for this PlacementModel.
   * \param decision_value The decision value for this PlacementModel.
   * \param sigma_x The standard deviation in the x-coordinate for this PlacementModel.
   * \param sigma_y The standard deviation in the y-coordinate for this PlacementModel.
   * \param sigma_theta The standard deviation in the angle (about the z-axis) for this PlacementModel.
   */
  PlacementModel(const world::Placement &placement, const double decision_value, const double sigma_x,
      const double sigma_y, const double sigma_theta);

  // TODO geters and setters

private:
  /*! The Placement value for the model. */
  world::Placement placement_;
  /*! The numeric attributes of this model. */
  double decision_value_, sigma_x_, sigma_y_, sigma_theta_;
};

}
}
}
}

#endif
