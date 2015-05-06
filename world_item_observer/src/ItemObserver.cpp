/*!
 * \file ItemObserver.cpp
 * \brief A persistent observer of items in the world for the spatial world database.
 *
 * The world item observer will store item observations in a remote spatial world database by listening to a
 * rail_manipulation_msgs/SegmentedObjectList message.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date May 5, 2015
 */

// World Item Observer
#include "world_item_observer/ItemObserver.h"

using namespace std;
using namespace rail::spatial_temporal_learning;

ItemObserver::ItemObserver() : worldlib::remote::Node()
{
  // load the config
  okay_ &= this->loadWorldYamlFile();

  // create the client we need
  spatial_world_client_ = this->createSpatialWorldClient();
  okay_ &= spatial_world_client_->connect();

  // connect to the grasp model database to check for items


  // connect to the segmented objects topic
  string recognized_objects_topic("/object_recognition_listener/recognized_objects");
  private_node_.getParam("recognized_objects_topic", recognized_objects_topic);
  recognized_objects_sub_ = node_.subscribe(recognized_objects_topic, 1, &ItemObserver::recognizedObjectsCallback,
                                            this);

  if (okay_)
  {
    ROS_INFO("Item Observer Initialized");
  }
}

ItemObserver::~ItemObserver()
{
  // clean up the client
  delete spatial_world_client_;
}

void ItemObserver::recognizedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectListConstPtr &objects) const
{
  // only work on a non-cleared list
  if (!objects->cleared)
  {
    // only utilize recognized objects
    for (size_t i = 0; i < objects->objects.size(); i++)
    {
      const rail_manipulation_msgs::SegmentedObject &o = objects->objects[i];
      if (o.recognized)
      {
        // transform the centroid to the fixed frame (shift down the Z)
        worldlib::geometry::Position offset(o.centroid.x, o.centroid.y, o.centroid.z - (o.height / 2.0));
        const worldlib::geometry::Pose centroid(offset);
        const string &frame_id = o.point_cloud.header.frame_id;
        const worldlib::geometry::Pose p_centroid_world = this->transformToWorld(centroid, frame_id);

        // check if it is on a surface
        size_t room_i, surface_i, placement_surface_i;
        if (world_.findPlacementSurface(p_centroid_world.getPosition(), room_i, surface_i, placement_surface_i))
        {
          // determine the position of the item on the surface
          const worldlib::world::Room &room = world_.getRoom(room_i);
          const worldlib::geometry::Pose p_centroid_room = room.fromParentFrame(p_centroid_world);
          const worldlib::world::Surface &surface = room.getSurface(surface_i);
          const worldlib::geometry::Pose p_centroid_surface = surface.fromParentFrame(p_centroid_room);

          // create and store the Observation
          const worldlib::world::Item item(o.name, "", p_centroid_surface, o.width, o.depth, o.height);
          spatial_world_client_->addObservation(item, surface, p_centroid_surface);
        }

      }
    }
  }
}
