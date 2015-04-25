#include "worldlib/worldlib.h"
#include "ros/ros.h"

using namespace std;
using namespace rail::spatial_temporal_learning::worldlib;

int main(int argc, char **argv)
{
  ros::Time::init();
  // initialize ROS and the node
  ros::init(argc, argv, "test");

//  remote::SpatialWorldClient c("localhost", remote::Client::DEFAULT_PORT, "testros", "YKetQVGz5GVwrShc", "rms");
//  if (c.connect())
//  {
//    cout << "connection!" << endl;
//  }
//
//  world::Item i("new");
//  world::Surface surface("test surface", "test frame");
//  surface.addPlacementSurface(world::PlacementSurface("test placement", "test placement frame"));
//  geometry::Pose pose(geometry::Position(1, 2, 3), geometry::Orientation(1.8));
//
//  //c.clearAllEntities();
//
//  //c.addObservation(i, surface, pose);
//
//  vector<remote::SpatialWorldObservation> observations;
//  c.getObservationsBySurfaceFrameID("test frame", observations);
//
//  for (size_t i = 0; i < observations.size(); i++)
//  {
//    cout << observations[i].getItemName() << endl;
//    cout << observations[i].getSurfaceName() << endl;
//    cout << observations[i].getSurfaceFrameID() << endl;
//    cout << observations[i].getPose().getPosition().getX() << endl;
//    cout << observations[i].getPose().getPosition().getY() << endl;
//    cout << observations[i].getPose().getPosition().getZ() << endl;
//    cout << observations[i].getPose().getOrientation().getTheta() << endl;
//    cout << observations[i].getTime().sec << endl;
//    cout << observations[i].getRemovedEstimate().sec << endl;
//    cout << observations[i].getRemovedObserved().sec << endl << endl;
//  }
//
//  if (c.itemObservedOnSurface("nedw", "test surface"))
//  {
//    cout << "YES" << endl;
//  } else
//  {
//    cout << "NO" << endl;
//  }
//
//  c.markObservationsAsRemoved("new", "test surface");
//  c.disconnect();

  remote::InteractiveWorldModelClient client;
  client.getModel(1);
  client.getModel(remote::InteractiveWorldModelClient::TASK_ID_MAGAZINE_PLACEMENT);
}
