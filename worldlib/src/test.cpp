#include "worldlib/worldlib.h"
#include "ros/ros.h"

using namespace std;
using namespace rail::spatial_temporal_learning::worldlib;

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "test");

  sql::SpatialWorldClient c("localhost", sql::Client::DEFAULT_PORT, "testros", "YKetQVGz5GVwrShc", "rms");
  if (c.connect())
  {
    cout << "connection!" << endl;
  }

  world::Item i("test item");
  world::Surface surface("test surface", "test frame");
  geometry::Pose pose(geometry::Position(1, 2, 3));

  //c.clearAllEntities();

  c.addObservation(i, surface, pose);

  c.disconnect();

  ros::spin();
}
