/*!
 * \file PlacementSurface.h
 * \brief Placement surface configuration information.
 *
 * A placement surface represents a section of a Surface that objects can be placed on. For example, the Surface object
 * "Shelving Unit" might have multiple placement surfaces representing each shelf.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 22, 2015
 */

#ifndef SPATIAL_TEMPORAL_LEARNING_WORLDLIB_WORLD_PLACEMENT_SURFACE_H_
#define SPATIAL_TEMPORAL_LEARNING_WORLDLIB_WORLD_PLACEMENT_SURFACE_H_

// worldlib
#include "Object.h"

namespace rail
{
namespace spatial_temporal_learning
{
namespace worldlib
{
namespace world
{

/*!
 * \class PlacementSurface
 * \brief Placement surface configuration information.
 *
 * A placement surface represents a section of a Surface that objects can be placed on. For example, the Surface object
 * "Shelving Unit" might have multiple placement surfaces representing each shelf.
 */
class PlacementSurface : public Object
{
public:
  /*!
   * \brief Create a new PlacementSurface.
   *
   * Create a new empty PlacementSurface with the given name, frame ID, Pose (in reference to the surfaces' frame ID)
   * and dimensions.
   *
   * \param name The name of the PlacementSurface (defaults to the empty string).
   * \param frame_id The frame ID of the PlacementSurface (defaults to the empty string).
   * \param pose The Pose of the PlacementSurface with respect to the Surface (defaults to 0 Pose).
   * \param width The width of the PlacementSurface (along the x-axis) (defaults to 0).
   * \param depth The width of the PlacementSurface (along the y-axis) (defaults to 0).
   * \param height The height of the PlacementSurface (along the z-axis) (defaults to 0).
   */
  PlacementSurface(const std::string &name = "", const std::string &frame_id = "",
      const geometry::Pose &pose = geometry::Pose(), const double width = 0, const double depth = 0,
      const double height = 0);
};

}
}
}
}

#endif
