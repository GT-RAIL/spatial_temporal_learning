#include "world_item_observer/CarlAutoObserver.h"

using namespace std;
using namespace rail::spatial_temporal_learning;

/*!
 * Creates and runs the carl_auto_observer node.
 *
 * \param argc argument count that is passed to ros::init.
 * \param argv arguments that are passed to ros::init.
 * \return EXIT_SUCCESS if the node runs correctly or EXIT_FAILURE if an error occurs.
 */
int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "carl_auto_observer");
  CarlAutoObserver observer;
  // check if everything started okay
  if (observer.okay())
  {
    ros::spin();
    observer.run();
    return EXIT_SUCCESS;
  } else
  {
    return EXIT_FAILURE;
  }
}
