/*!
 * \file PlacementModel.h
 * \brief Placement model information.
 *
 * A placement model contains information about the strength of an item in reference to some object.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 25, 2015
 */

// worldlib
#include "worldlib/model/PlacementModel.h"

using namespace std;
using namespace rail::spatial_temporal_learning::worldlib::model;
using namespace rail::spatial_temporal_learning::worldlib::world;

PlacementModel::PlacementModel(const Placement &placement, const double decision_value, const double sigma_x,
    const double sigma_y, const double sigma_theta) : placement_(placement)
{
  decision_value_ = decision_value;
  sigma_x_ = sigma_x;
  sigma_y_ = sigma_y;
  sigma_theta_ = sigma_theta;
}
