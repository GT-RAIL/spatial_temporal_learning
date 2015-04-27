/*!
 * \file TaskModel.cpp
 * \brief Task placement model information.
 *
 * A task model contains a set of placement models representing the strength of reference frames for a given task.
 *
 * \author Russell Toris, WPI - rctoris@wpi.edu
 * \date April 27, 2015
 */

// worldlib
#include "worldlib/model/TaskModel.h"

using namespace std;
using namespace rail::spatial_temporal_learning::worldlib::model;

TaskModel::TaskModel(const uint32_t task_id, const string &name) : name_(name)
{
  task_id_ = task_id;
}

uint32_t TaskModel::getTaskID()
{
  return task_id_;
}

void TaskModel::setTaskID(const uint32_t task_id)
{
  task_id_ = task_id;
}

const string &TaskModel::getName() const
{
  return name_;
}

void TaskModel::setName(const std::string &name)
{
  name_ = name;
}

const vector<PlacementModel> &TaskModel::getPlacementModels() const
{
  return placement_models_;
}

size_t TaskModel::getNumPlacementModels() const
{
  return placement_models_.size();
}

const PlacementModel &TaskModel::getPlacementModel(const size_t index) const
{
  // check the index value first
  if (index < placement_models_.size())
  {
    return placement_models_[index];
  } else
  {
    throw out_of_range("TaskModel::getPlacementModel : PlacementModel index does not exist.");
  }
}

void TaskModel::addPlacementModel(const PlacementModel &room)
{
  placement_models_.push_back(room);
}

void TaskModel::removePlacementModel(const size_t index)
{
  // check the index value first
  if (index < placement_models_.size())
  {
    placement_models_.erase(placement_models_.begin() + index);
  } else
  {
    throw out_of_range("TaskModel::removePlacementModel : PlacementModel index does not exist.");
  }
}
