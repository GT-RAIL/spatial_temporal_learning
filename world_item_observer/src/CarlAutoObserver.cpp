// World Item Observer
#include "world_item_observer/CarlAutoObserver.h"

// ROS
#include <carl_dynamixel/LookAtFrame.h>
#include <std_srvs/Empty.h>

using namespace rail::spatial_temporal_learning;

CarlAutoObserver::CarlAutoObserver()
    : worldlib::remote::Node(), nav_ac_("/move_base", true), ac_wait_time_(AC_WAIT_TIME)
{
  // load the config
  okay_ = this->loadWorldYamlFile();
  segment_srv_ = node_.serviceClient<std_srvs::Empty>("/rail_segmentation/segment");
  look_at_frame_srv_ = node_.serviceClient<carl_dynamixel::LookAtFrame>("/asus_controller/look_at_frame");

  if (okay_) ROS_INFO("CARL Automatic Observer Initialized");
}

void CarlAutoObserver::run()
{
  // seed the rand
  srand(time(NULL));

  while (ros::ok())
  {
    // pick a random location
    const size_t r = world_.getNumRooms() > 0 ? rand() % world_.getNumRooms() : 0;
    const worldlib::world::Room &room = world_.getRoom(r);
    const size_t s = room.getNumSurfaces() > 0 ? rand() % room.getNumSurfaces() : 0;
    const worldlib::world::Surface &surface = room.getSurface(s);
    const std::string &nav = surface.getPlacementSurfaces().back().getNavFrameID();

    // drive to a random location
    ROS_INFO_STREAM("Driving to " << surface.getName() << "...");
    move_base_msgs::MoveBaseGoal nav_goal;
    nav_goal.target_pose.header.frame_id = nav;
    nav_goal.target_pose.pose.orientation.w = QUATERNION_90_ROTATE;
    nav_goal.target_pose.pose.orientation.z = QUATERNION_90_ROTATE;
    nav_ac_.sendGoal(nav_goal);
    const bool completed = nav_ac_.waitForResult(ac_wait_time_);
    const bool succeeded = (nav_ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED);
    if (completed && succeeded)
    {
      // point the camera
      carl_dynamixel::LookAtFrame look;
      look.request.frame = surface.getPlacementSurfaces().back().getFrameID();
      if (look_at_frame_srv_.call(look))
      {
        // segment
        ROS_INFO("Segmenting surface...");
        std_srvs::Empty segment;
        if (!segment_srv_.call(segment)) ROS_WARN("Could not segment...");
      } else
      {
        ROS_WARN("Could not look...");
      }
    } else
    {
      ROS_WARN("Could not drive...");
    }

    // sleep for a random time between 1 minute to an hour
    double seconds = rand() % 3600 + 1;
    ROS_INFO_STREAM("Sleeping for " << seconds << " seconds...");
    ros::Rate rate(1.0 / seconds);
    rate.sleep();

    // rinse and repeat!
  }
}
