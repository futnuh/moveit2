/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Mario Prats, Ioan Sucan */
#include <moveit/warehouse/state_storage.h>

#include <moveit/motion_planning_rviz_plugin/motion_planning_frame.h>
#include <moveit/motion_planning_rviz_plugin/motion_planning_display.h>
#include <moveit/robot_state/conversions.h>

#include <QMessageBox>
#include <QInputDialog>

#include <cmath>

#include "ui_motion_planning_rviz_plugin_frame.h"

namespace moveit_rviz_plugin
{
static const rclcpp::Logger LOGGER = rclcpp::get_logger("moveit_ros_visualization.motion_planning_frame_states");

void MotionPlanningFrame::populateRobotStatesList()
{
  ui_->list_states->clear();
  for (std::pair<const std::string, moveit_msgs::msg::RobotState>& robot_state : robot_states_)
  {
    QListWidgetItem* item = new QListWidgetItem(QString(robot_state.first.c_str()));
    ui_->list_states->addItem(item);
  }
}

void MotionPlanningFrame::loadStateButtonClicked()
{
  if (robot_state_storage_)
  {
    bool ok;

    QString text =
        QInputDialog::getText(this, tr("Robot states to load"), tr("Pattern:"), QLineEdit::Normal, ".*", &ok);
    if (ok && !text.isEmpty())
    {
      loadStoredStates(text.toStdString());
    }
  }
  else
  {
    QMessageBox::warning(this, "Warning", "Not connected to a database.");
  }
}

void MotionPlanningFrame::loadStoredStates(const std::string& pattern)
{
  std::vector<std::string> names;
  try
  {
    robot_state_storage_->getKnownRobotStates(pattern, names);
  }
  catch (std::exception& ex)
  {
    QMessageBox::warning(this, "Cannot query the database",
                         QString("Wrongly formatted regular expression for robot states: ").append(ex.what()));
    return;
  }
  // Clear the current list
  clearStatesButtonClicked();

  for (const std::string& name : names)
  {
    moveit_warehouse::RobotStateWithMetadata rs;
    bool got_state = false;
    try
    {
      got_state = robot_state_storage_->getRobotState(rs, name);
    }
    catch (std::exception& ex)
    {
      RCLCPP_ERROR(LOGGER, "%s", ex.what());
    }
    if (!got_state)
      continue;

    // Overwrite if exists.
    if (robot_states_.find(name) != robot_states_.end())
    {
      robot_states_.erase(name);
    }

    // Store the current start state
    robot_states_.insert(RobotStatePair(name, *rs));
  }
  populateRobotStatesList();
}

void MotionPlanningFrame::saveRobotStateButtonClicked(const moveit::core::RobotState& state)
{
  bool ok = false;

  std::stringstream ss;
  ss << planning_display_->getRobotModel()->getName().c_str() << "_state_" << std::setfill('0') << std::setw(4)
     << robot_states_.size();

  QString text = QInputDialog::getText(this, tr("Choose a name"), tr("State name:"), QLineEdit::Normal,
                                       QString(ss.str().c_str()), &ok);

  std::string name;
  if (ok)
  {
    if (!text.isEmpty())
    {
      name = text.toStdString();
      if (robot_states_.find(name) != robot_states_.end())
        QMessageBox::warning(this, "Name already exists",
                             QString("The name '").append(name.c_str()).append("' already exists. Not creating state."));
      else
      {
        // Store the current start state
        moveit_msgs::msg::RobotState msg;
        moveit::core::robotStateToRobotStateMsg(state, msg);
        robot_states_.insert(RobotStatePair(name, msg));

        // Save to the database if connected
        if (robot_state_storage_)
        {
          try
          {
            robot_state_storage_->addRobotState(msg, name, planning_display_->getRobotModel()->getName());
          }
          catch (std::exception& ex)
          {
            RCLCPP_ERROR(LOGGER, "Cannot save robot state on the database: %s", ex.what());
          }
        }
        else
        {
          QMessageBox::warning(this, "Warning", "Not connected to a database. The state will be created but not stored");
        }
      }
    }
    else
      QMessageBox::warning(this, "Start state not saved", "Cannot use an empty name for a new start state.");
  }
  populateRobotStatesList();
}

void MotionPlanningFrame::saveStartStateButtonClicked()
{
  saveRobotStateButtonClicked(*planning_display_->getQueryStartState());
}

void MotionPlanningFrame::saveGoalStateButtonClicked()
{
  saveRobotStateButtonClicked(*planning_display_->getQueryGoalState());
}

void MotionPlanningFrame::setAsStartStateButtonClicked()
{
  QListWidgetItem* item = ui_->list_states->currentItem();

  if (item)
  {
    moveit::core::RobotState robot_state(*planning_display_->getQueryStartState());
    moveit::core::robotStateMsgToRobotState(robot_states_[item->text().toStdString()], robot_state);
    planning_display_->setQueryStartState(robot_state);
  }
}

void MotionPlanningFrame::setAsGoalStateButtonClicked()
{
  QListWidgetItem* item = ui_->list_states->currentItem();

  if (item)
  {
    moveit::core::RobotState robot_state(*planning_display_->getQueryGoalState());
    moveit::core::robotStateMsgToRobotState(robot_states_[item->text().toStdString()], robot_state);
    planning_display_->setQueryGoalState(robot_state);
  }
}

void MotionPlanningFrame::removeStateButtonClicked()
{
  if (robot_state_storage_)
  {
    // Warn the user
    QMessageBox msg_box;
    msg_box.setText("All the selected states will be removed from the database");
    msg_box.setInformativeText("Do you want to continue?");
    msg_box.setStandardButtons(QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
    msg_box.setDefaultButton(QMessageBox::No);
    int ret = msg_box.exec();

    switch (ret)
    {
      case QMessageBox::Yes:
      {
        QList<QListWidgetItem*> found_items = ui_->list_states->selectedItems();
        for (QListWidgetItem* found_item : found_items)
        {
          const std::string& name = found_item->text().toStdString();
          try
          {
            robot_state_storage_->removeRobotState(name);
            robot_states_.erase(name);
          }
          catch (std::exception& ex)
          {
            RCLCPP_ERROR(LOGGER, "%s", ex.what());
          }
        }
        break;
      }
    }
  }
  populateRobotStatesList();
}

void MotionPlanningFrame::renameStateButtonClicked()
{
  QListWidgetItem* item = ui_->list_states->currentItem();
  if (!item)
  {
    QMessageBox::warning(this, "No state selected", "Please select a stored robot state to rename.");
    return;
  }

  const std::string old_name = item->text().toStdString();
  auto state_it = robot_states_.find(old_name);
  if (state_it == robot_states_.end())
  {
    QMessageBox::warning(this, "State not found", "The selected state could not be found in memory.");
    return;
  }

  bool ok = false;
  QString new_name_qstr =
      QInputDialog::getText(this, tr("Rename stored state"), tr("State name:"), QLineEdit::Normal, item->text(), &ok);
  if (!ok)
  {
    return;
  }
  if (new_name_qstr.isEmpty())
  {
    QMessageBox::warning(this, "Invalid name", "State name cannot be empty.");
    return;
  }

  const std::string new_name = new_name_qstr.toStdString();
  if (new_name == old_name)
  {
    return;
  }

  if (robot_states_.find(new_name) != robot_states_.end())
  {
    QMessageBox::warning(this, "Name already exists",
                         QString("The name '").append(new_name_qstr).append("' already exists. Choose a new name."));
    return;
  }

  const moveit_msgs::msg::RobotState stored_state = state_it->second;

  // Attempt to persist the rename in the warehouse database first
  if (robot_state_storage_)
  {
    try
    {
      robot_state_storage_->renameRobotState(old_name, new_name, planning_display_->getRobotModel()->getName());
    }
    catch (const std::exception& ex)
    {
      QMessageBox::warning(this, "Failed to rename state in database",
                           QString("Unable to rename the state in the warehouse: ").append(ex.what()));
      return;
    }
  }

  robot_states_.erase(state_it);
  robot_states_.insert(RobotStatePair(new_name, stored_state));

  populateRobotStatesList();
  setItemSelectionInList(new_name, true, ui_->list_states);
}

void MotionPlanningFrame::updateStateFromGoalButtonClicked()
{
  QListWidgetItem* item = ui_->list_states->currentItem();
  if (!item)
  {
    QMessageBox::warning(this, "No state selected", "Please select a stored robot state to update.");
    return;
  }

  const std::string state_name = item->text().toStdString();
  auto state_it = robot_states_.find(state_name);
  if (state_it == robot_states_.end())
  {
    QMessageBox::warning(this, "State not found", "The selected state could not be found in memory.");
    return;
  }

  const moveit::core::RobotState& goal_state = *planning_display_->getQueryGoalState();
  moveit::core::RobotState stored_state(goal_state);
  moveit::core::robotStateMsgToRobotState(state_it->second, stored_state);

  const double* stored_positions = stored_state.getVariablePositions();
  const double* goal_positions = goal_state.getVariablePositions();
  const std::size_t variable_count = goal_state.getVariableCount();

  double max_difference = 0.0;
  for (std::size_t i = 0; i < variable_count; ++i)
  {
    const double difference = std::fabs(goal_positions[i] - stored_positions[i]);
    if (difference > max_difference)
      max_difference = difference;
  }

  constexpr double SIGNIFICANT_STATE_DIFFERENCE = 0.1;  // radians or meters, depending on joint type
  if (max_difference > SIGNIFICANT_STATE_DIFFERENCE)
  {
    QMessageBox::StandardButton response = QMessageBox::warning(
        this, tr("Confirm State Update"),
        tr("The goal state differs from stored state '%1' by up to %2.\n\nDo you want to overwrite this stored state "
           "with the current goal state?")
            .arg(item->text())
            .arg(max_difference, 0, 'f', 3),
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
    if (response != QMessageBox::Yes)
    {
      return;
    }
  }

  moveit_msgs::msg::RobotState updated_state_msg;
  moveit::core::robotStateToRobotStateMsg(goal_state, updated_state_msg);

  if (robot_state_storage_)
  {
    try
    {
      robot_state_storage_->removeRobotState(state_name);
      robot_state_storage_->addRobotState(updated_state_msg, state_name, planning_display_->getRobotModel()->getName());
    }
    catch (const std::exception& ex)
    {
      QMessageBox::warning(this, "Failed to update state in database",
                           QString("Unable to update the state in the warehouse: %1").arg(ex.what()));
      return;
    }
  }

  state_it->second = updated_state_msg;
  populateRobotStatesList();
  setItemSelectionInList(state_name, true, ui_->list_states);
}

void MotionPlanningFrame::clearStatesButtonClicked()
{
  QMessageBox msg_box;
  msg_box.setText("Clear all stored robot states (from memory, not from the database)?");
  msg_box.setStandardButtons(QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
  msg_box.setDefaultButton(QMessageBox::Yes);
  int ret = msg_box.exec();
  switch (ret)
  {
    case QMessageBox::Yes:
    {
      robot_states_.clear();
      populateRobotStatesList();
    }
    break;
  }
  return;
}

void MotionPlanningFrame::storedStateItemSelectionChanged()
{
  QListWidgetItem* item = ui_->list_states->currentItem();

  if (item)
  {
    const std::string& state_name = item->text().toStdString();
    auto it = robot_states_.find(state_name);
    if (it != robot_states_.end())
    {
      // Set the selected stored state as the goal state
      moveit::core::RobotState robot_state(*planning_display_->getQueryGoalState());
      moveit::core::robotStateMsgToRobotState(it->second, robot_state);
      planning_display_->setQueryGoalState(robot_state);
    }
  }
}

}  // namespace moveit_rviz_plugin
