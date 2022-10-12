/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Kentaro Wada.
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

/* Author: Kentaro Wada */

#include "execute_trajectory_action_capability.h"

#include <moveit/plan_execution/plan_execution.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/move_group/capability_names.h>

// Name of this class for logging
static const std::string LOGNAME = "execute_trajectory_action_capability";

namespace move_group
{
MoveGroupExecuteTrajectoryAction::MoveGroupExecuteTrajectoryAction() : MoveGroupCapability("ExecuteTrajectoryAction")
{
}

MoveGroupExecuteTrajectoryAction::~MoveGroupExecuteTrajectoryAction()
{
  if (execute_thread_)
  {
    terminate_mutex_.lock();
    need_to_terminate_ = true;
    terminate_mutex_.unlock();
    execute_condition_.notify_all();
    execute_thread_->join();
  }
}

void MoveGroupExecuteTrajectoryAction::initialize()
{
  // start the move action server
  execute_action_server_ = std::make_unique<ExecuteTrajectoryActionServer>(
      root_node_handle_, EXECUTE_ACTION_NAME, [this](const auto& goal) { goalCallback(goal); }, false);
  execute_action_server_->start();

  need_to_terminate_ = false;
  execute_thread_ = std::make_unique<std::thread>(&MoveGroupExecuteTrajectoryAction::executeLoop, this);
}

void MoveGroupExecuteTrajectoryAction::executeLoop()
{
  while (root_node_handle_.ok())
  {
    {
      std::unique_lock<std::mutex> ulock(execute_mutex_, std::try_to_lock);
      while (new_goals_.empty() && canceled_goals_.empty())
        execute_condition_.wait(ulock);
    }

    clearInactiveGoals();

    {
      std::lock_guard<std::mutex> terminate_lock(terminate_mutex_);
      if (need_to_terminate_)
        break;
    }

    {
      std::unique_lock<std::mutex> ulock(goal_mutex_);

      ROS_DEBUG_NAMED(getName(), "new_goals: %zu, canceled_goals: %zu", new_goals_.size(), canceled_goals_.size());

      // Process requests for canceling goals
      for (auto& canceled_goal : canceled_goals_)
      {
        context_->trajectory_execution_manager_->stopExecution(canceled_goal->getGoal()->trajectory);
        ;
        cancelGoal(*canceled_goal);
      }
      canceled_goals_.clear();

      // Process requests for new goals
      if (new_goals_.empty())
        return;

      if (!context_->trajectory_execution_manager_)
      {
        for (auto ng : new_goals_)
        {
          moveit_msgs::ExecuteTrajectoryResult action_res;
          const std::string response = "Cannot execute trajectory since ~allow_trajectory_execution was set to false";
          action_res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;
          ng->setCanceled(action_res, response);
        }
        std::unique_lock<std::mutex> ulock(goal_mutex_);
        new_goals_.clear();
        canceled_goals_.clear();
        continue;
      }

      if (context_->trajectory_execution_manager_->getEnableSimultaneousExecution())
      {
        for (auto ng : new_goals_)
        {
          active_goals_.emplace(std::make_pair(ng, std::make_unique<std::thread>([this, &ng]() { executePath(ng); })));
        }
        new_goals_.clear();
      }
      else
      {
        // preempt all but the most recent new goal
        auto new_goal = std::move(new_goals_.back());
        new_goals_.pop_back();

        for (auto& ng : new_goals_)
          cancelGoal(*ng);
        new_goals_.clear();

        goal_mutex_.unlock();

        // check if we need to send a preempted message for the goal that we're currently pursuing
        if (isActive(current_goal_))
        {
          // Stop current execution and then cancel current goal
          context_->trajectory_execution_manager_->stopExecution(true);
          ;
          cancelGoal(current_goal_);
        }

        current_goal_ = *new_goal;

        active_goals_mutex_.lock();
        active_goals_.emplace(
            std::make_pair(new_goal, std::make_unique<std::thread>([this, &new_goal]() {
          executePath(new_goal); })));
        active_goals_mutex_.unlock();
      }
    }
  }
}

void MoveGroupExecuteTrajectoryAction::clearInactiveGoals()
{
  // clear inactive goals
  std::lock_guard<std::mutex> slock(active_goals_mutex_);
  auto it = active_goals_.begin();
  while (it != active_goals_.end())
  {
    auto& goal_handle = it->first;
    auto& goal_thread = it->second;
    if (!isActive(*goal_handle))
    {
      if (goal_thread->joinable())
        goal_thread->join();
      it = active_goals_.erase(it);
    }
    else
      it++;
  }
}

void MoveGroupExecuteTrajectoryAction::cancelGoal(ExecuteTrajectoryActionServer::GoalHandle& goal)
{
  moveit_msgs::ExecuteTrajectoryResult action_res;
  const std::string response = "This goal was canceled because another goal was recieved by the MoveIt action server";
  action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
  goal.setCanceled(action_res, response);
}

bool MoveGroupExecuteTrajectoryAction::isActive(ExecuteTrajectoryActionServer::GoalHandle& goal)
{
  if (!goal.getGoal())
    return false;
  unsigned int status = goal.getGoalStatus().status;
  return status == actionlib_msgs::GoalStatus::ACTIVE || status == actionlib_msgs::GoalStatus::PREEMPTING;
}

void MoveGroupExecuteTrajectoryAction::goalCallback(ExecuteTrajectoryActionServer::GoalHandle goal_handle)
{
  std::lock_guard<std::mutex> slock(goal_mutex_);
  new_goals_.push_back(std::make_unique<ExecuteTrajectoryActionServer::GoalHandle>(goal_handle));
  execute_condition_.notify_all();
}

void MoveGroupExecuteTrajectoryAction::cancelCallback(ExecuteTrajectoryActionServer::GoalHandle goal_handle)
{
  std::lock_guard<std::mutex> slock(goal_mutex_);
  canceled_goals_.push_back(std::make_unique<ExecuteTrajectoryActionServer::GoalHandle>(goal_handle));
  execute_condition_.notify_all();
}

void MoveGroupExecuteTrajectoryAction::executePath(
    std::shared_ptr<ExecuteTrajectoryActionServer::GoalHandle> goal_handle_ptr)
{
  ROS_INFO_NAMED(LOGNAME, "Goal received (ExecuteTrajectoryActionServer)");
  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Goal ID" << goal_handle_ptr->getGoalID());

  goal_handle_ptr->setAccepted("This goal has been accepted by the action server");

  if (!context_->trajectory_execution_manager_->getEnableSimultaneousExecution())
    context_->trajectory_execution_manager_->clear();

  auto executionCallback = [this, goal_handle_ptr](const moveit_controller_manager::ExecutionStatus& status) {
    setExecuteTrajectoryState(IDLE, *goal_handle_ptr);
    sendGoalResponse(*goal_handle_ptr, status);
    ROS_INFO_STREAM_NAMED(LOGNAME, "Execution completed: " << status.asString());
    execute_condition_.notify_all();
  };

  if (context_->trajectory_execution_manager_->push(goal_handle_ptr->getGoal()->trajectory, "", executionCallback))
  {
    setExecuteTrajectoryState(MONITOR, *goal_handle_ptr);
    if (!context_->trajectory_execution_manager_->getEnableSimultaneousExecution())
      context_->trajectory_execution_manager_->execute(executionCallback, true);
  }
  else
  {
    ROS_ERROR_NAMED(LOGNAME, "Failed to pushed trajectory");
  }
}

void MoveGroupExecuteTrajectoryAction::sendGoalResponse(
    ExecuteTrajectoryActionServer::GoalHandle& goal, const moveit_controller_manager::ExecutionStatus& execution_status)
{
  setExecuteTrajectoryState(IDLE, goal);

  moveit_msgs::ExecuteTrajectoryResult action_res;

  if (execution_status == moveit_controller_manager::ExecutionStatus::SUCCEEDED)
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::SUCCESS;
  else if (execution_status == moveit_controller_manager::ExecutionStatus::PREEMPTED)
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
  else if (execution_status == moveit_controller_manager::ExecutionStatus::TIMED_OUT)
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::TIMED_OUT;
  else  // ABORTED, FAILED, UNKNOWN
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::CONTROL_FAILED;

  ROS_DEBUG_STREAM_NAMED(LOGNAME, "Execution completed: " << execution_status.asString());

  const std::string response = this->getActionResultString(action_res.error_code, false, false);

  if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    goal.setSucceeded(action_res, response);
  else
    goal.setAborted(action_res, response);
}

void MoveGroupExecuteTrajectoryAction::setExecuteTrajectoryState(const MoveGroupState& state,
                                                                 ExecuteTrajectoryActionServer::GoalHandle& goal)
{
  moveit_msgs::ExecuteTrajectoryFeedback execute_feedback;
  execute_feedback.state = stateToStr(state);
  goal.publishFeedback(execute_feedback);
}

}  // namespace move_group

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupExecuteTrajectoryAction, move_group::MoveGroupCapability)
