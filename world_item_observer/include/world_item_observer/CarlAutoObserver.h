#ifndef SPATIAL_TEMPORAL_LEARNING_CARL_AUTO_OBSERVER_H_
#define SPATIAL_TEMPORAL_LEARNING_CARL_AUTO_OBSERVER_H_

// worldlib
#include "worldlib/remote/Node.h"

// ROS
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

namespace rail
{
namespace spatial_temporal_learning
{

class CarlAutoObserver : public worldlib::remote::Node
{
public:
  /*!
   * \brief Create a CarlAutoObserver and associated ROS information.
   *
   * Creates the ROS node handle.
   */
  CarlAutoObserver();

  /*!
   * Run CARL around the room at random intervals until ctrl-c is caught.
   */
  void run();

private:
  /*! Constant used for the W and Z components of a quaternion for a 90 degree rotation about the Z axis. */
  static const double QUATERNION_90_ROTATE = 0.7071067811865476;
  /*! The default wait time for action servers in seconds. */
  static const int AC_WAIT_TIME = 90;

  /*! The action client timeout. */
  ros::Duration ac_wait_time_;

  /*! The navigation action client. */
  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> nav_ac_;
  /*! The camera and segmentation service clients. */
  ros::ServiceClient look_at_frame_srv_, segment_srv_;
};

}
}

#endif