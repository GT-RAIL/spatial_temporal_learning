/*!
 * \file PlacementSurface.cpp
 * \brief Placement surface configuration information.
 *
 * A placement surface represents a section of a Surface that objects can be placed on. For example, the Surface object
 * "Shelving Unit" might have multiple placement surfaces representing each shelf.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 22, 2015
 */

// worldlib
#include "worldlib/world/PlacementSurface.h"

using namespace std;
using namespace rail::spatial_temporal_learning::worldlib::world;

PlacementSurface::PlacementSurface(const string &name, const string &frame_id, const geometry::Pose &pose,
    const double width, const double depth, const double height) : Object(name, frame_id, pose, width, depth, height)
{
}
