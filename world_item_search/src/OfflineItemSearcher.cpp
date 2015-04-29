/*!
 * \file OfflineItemSearcher.cpp
 * \brief An offline object search example node.
 *
 * The offline item search is an example item search node. This node will fake observation data and build models from
 * the faked data. This is useful for model verification methods and data collection for ongoing research.
 *
 * Note that this node will clear the spatial world database several times. You should only run this node on a test
 * database, not your robot's persistent database.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 27, 2015
 */

// World Item Search
#include "world_item_search/OfflineItemSearcher.h"

// ROS
#include <ros/package.h>

// Boost
#include <boost/filesystem.hpp>

// C++ Standard Library
#include <fstream>

using namespace std;
using namespace rail::spatial_temporal_learning;

OfflineItemSearcher::OfflineItemSearcher() : worldlib::remote::Node()
{
  // location of the GeoLife files
  string geolife(ros::package::getPath("world_item_search") + "/geolife");
  private_node_.getParam("geolife", geolife);
  // open the Geolife files for model generation
  this->loadGeoLife(geolife);
  if (!okay_)
  {
    ROS_ERROR("Unable to load any GeoLife PLT files in '%s'.", geolife.c_str());
  }

  // create the clients we need
  interactive_world_model_client_ = this->createInteractiveWorldModelClient();
  spatial_world_client_ = this->createSpatialWorldClient();

  // our default "room"
  world_.addRoom(worldlib::world::Room("default"));
  interactive_world_.addRoom(worldlib::world::Room("default"));

  // grab the interactive world models
  uint32_t task_id = worldlib::remote::InteractiveWorldModelClient::TASK_ID_PUT_AWAY_GENERAL;
  okay_ &= interactive_world_model_client_->getTaskModel(task_id, interactive_world_task_model_);
  interactive_world_task_model_.getUniqueItems(interactive_world_.getItems());
  interactive_world_task_model_.getUniqueSurfaces(interactive_world_.getRoom(0).getSurfaces());

  // attempt to connect to the spatial world database
  okay_ &= spatial_world_client_->connect();

  // clear the initial database
  spatial_world_client_->clearAllEntities();

  // initialize the objects we have in our "world"
  world_.addItem(worldlib::world::Item("spoon"));
  world_.addItem(worldlib::world::Item("fork"));
  world_.addItem(worldlib::world::Item("bowl"));
  world_.addItem(worldlib::world::Item("keys"));
  world_.getRoom(0).addSurface(worldlib::world::Surface("Coffee Table"));
  world_.getRoom(0).addSurface(worldlib::world::Surface("Sink Unit"));
  world_.getRoom(0).addSurface(worldlib::world::Surface("Dining Table with Chairs"));
  world_.getRoom(0).addSurface(worldlib::world::Surface("Dresser"));

  // create some observations
  ros::Time now = ros::Time::now();
  worldlib::world::Observation o1(world_.findItem("spoon"), world_.getRoom(0).findSurface("Coffee Table"),
                                  worldlib::geometry::Pose(), now - ros::Duration(120));
  worldlib::world::Observation o2(world_.findItem("spoon"), world_.getRoom(0).findSurface("Coffee Table"),
                                  worldlib::geometry::Pose(), now - ros::Duration(60));
  worldlib::world::Observation o3(world_.findItem("spoon"), world_.getRoom(0).findSurface("Coffee Table"),
                                  worldlib::geometry::Pose(), now - ros::Duration(45));
  spatial_world_client_->addObservation(o1);
  spatial_world_client_->addObservation(o2);
  spatial_world_client_->addObservation(o3);
  spatial_world_client_->markObservationsAsRemoved("spoon", "Coffee Table");

  if (okay_)
  {
    ROS_INFO("Offline Item Searcher Initialized");
  }
}

OfflineItemSearcher::~OfflineItemSearcher()
{
  // clean up clients
  delete interactive_world_model_client_;
  delete spatial_world_client_;
}

void OfflineItemSearcher::loadGeoLife(const std::string &directory)
{
  // use Boost to search the directory
  boost::filesystem::path path(directory.c_str());
  if (boost::filesystem::exists(path))
  {
    // the order is not defined, so we will sort a list instead
    vector<string> files;
    for (boost::filesystem::directory_iterator itr(path); itr != boost::filesystem::directory_iterator(); ++itr)
    {
      // check for a .plt file
      if (itr->path().extension().string() == ".plt")
      {
        files.push_back(itr->path().string());
      }
    }
    std::sort(files.begin(), files.end());
    for (size_t i = 0; i < files.size(); i++)
    {
      // open the file and read each line
      ifstream myfile(files[i].c_str());
      string line;
      uint32_t line_count = 0;
      while (std::getline(myfile, line))
      {
        // first 6 lines don't contain data
        if (++line_count > 6)
        {
          // split the line based on ','
          stringstream ss(line);
          string token;
          uint32_t token_count = 0;
          while (std::getline(ss, token, ','))
          {
            //double lat = boost::lexical_cast<double>(token);
            if (token_count == 4)
            {
              // date is "days since 12/30/1899" -- concert to posix time (move to 1/1/1970 then convert to seconds)
              double days = boost::lexical_cast<double>(token) - 25569;
              ros::Time time(round(days * 86400));
            }
            token_count++;
          }
        }
      }
    }
  }
}

void OfflineItemSearcher::run() const
{
  cout << "=== Begining Simulated Item Search Experiments ===" << endl;

  // print our world info
  cout << "World Items: ";
  this->printItemList(world_.getItems());
  cout << "World Surfaces: ";
  this->printSurfaceList(world_.getRoom(0).getSurfaces());
  cout << "Interactive World Items: ";
  this->printItemList(interactive_world_.getItems());
  cout << "Interactive World Surfaces: ";
  this->printSurfaceList(interactive_world_.getRoom(0).getSurfaces());

  // todo
  worldlib::model::PersistenceModel model = spatial_world_client_->getPersistenceModel("spoon", "Coffee Table");

  cout << "=== Simulated Item Search Experiments Finished ===" << endl;

}

void OfflineItemSearcher::printItemList(const vector<worldlib::world::Item> &items) const
{
  cout << "[";
  for (size_t i = 0; i < items.size(); i++)
  {
    cout << items[i].getName();
    (i < items.size() - 1) ? cout << ", " : cout << "";
  }
  cout << "]" << endl;
}

void OfflineItemSearcher::printSurfaceList(const vector<worldlib::world::Surface> &surfaces) const
{
  cout << "[";
  for (size_t i = 0; i < surfaces.size(); i++)
  {
    cout << surfaces[i].getName();
    (i < surfaces.size() - 1) ? cout << ", " : cout << "";
  }
  cout << "]" << endl;
}

