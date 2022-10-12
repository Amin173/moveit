/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
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

/* Author: Ioan Sucan */

#include "move_action_capability.h"

#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/plan_execution/plan_execution.h>
#include <moveit/plan_execution/plan_with_sensing.h>
#include <moveit/trajectory_processing/trajectory_tools.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/utils/message_checks.h>
#include <moveit/move_group/capability_names.h>

namespace move_group
{
MoveGroupMoveAction::MoveGroupMoveAction()
  : MoveGroupCapability("MoveAction"), move_state_(IDLE), preempt_requested_{ false }
{
}

MoveGroupMoveAction::~MoveGroupMoveAction()
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

void MoveGroupMoveAction::initialize()
{
  // start the move action server
  move_action_server_ = std::make_unique<MoveGroupActionServer>(
      root_node_handle_, MOVE_ACTION, [this](const auto& goal) { goalCallback(goal); }, false);
  move_action_server_->start();

  need_to_terminate_ = false;
  execute_thread_ = std::make_unique<std::thread>(&MoveGroupMoveAction::executeLoop, this);
}

void MoveGroupMoveAction::executeLoop()
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
      {
        break;
      }
    }

    {
      std::unique_lock<std::mutex> ulock(goal_mutex_);

      ROS_WARN_STREAM_NAMED(getName(),
                            "new_goals: " << new_goals_.size() << " canceled_goals: " << canceled_goals_.size());

      // Process requests for canceling goals
      for (auto& canceled_goal : canceled_goals_)
      {
        // TODO (cambel): this stops every execution, not just the one related to the goals
        if (*canceled_goal == current_goal_)
          context_->plan_execution_->stop();
        cancelGoal(*canceled_goal);
      }
      canceled_goals_.clear();

      // Process requests for new goals
      if (new_goals_.empty())
        return;

      if (context_->trajectory_execution_manager_->getEnableSimultaneousExecution())
      {
        for (auto ng : new_goals_)
        {
          active_goals_.emplace(
              std::make_pair(ng, std::make_unique<std::thread>([this, &ng]() { executeMoveCallback(ng); })));
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
          preempt_requested_ = true;
          context_->plan_execution_->stop();
          cancelGoal(current_goal_);
        }

        current_goal_ = *new_goal;

        active_goals_mutex_.lock();
        active_goals_.emplace(std::make_pair(
            new_goal, std::make_unique<std::thread>([this, &new_goal]() { executeMoveCallback(new_goal); })));
        active_goals_mutex_.unlock();
      }
    }
  }
}

void MoveGroupMoveAction::clearInactiveGoals()
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

void MoveGroupMoveAction::cancelGoal(MoveGroupActionServer::GoalHandle& goal_handle)
{
  moveit_msgs::MoveGroupResult action_res;
  const std::string response = "This goal was canceled because another goal was recieved by the MoveIt action server";
  action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
  goal_handle.setCanceled(action_res, response);
}

bool MoveGroupMoveAction::isActive(MoveGroupActionServer::GoalHandle& goal_handle)
{
  if (!goal_handle.getGoal())
    return false;
  unsigned int status = goal_handle.getGoalStatus().status;
  return status == actionlib_msgs::GoalStatus::ACTIVE || status == actionlib_msgs::GoalStatus::PREEMPTING;
}

void MoveGroupMoveAction::goalCallback(MoveGroupActionServer::GoalHandle goal_handle)
{
  std::lock_guard<std::mutex> slock(goal_mutex_);
  new_goals_.push_back(std::make_unique<MoveGroupActionServer::GoalHandle>(goal_handle));
  execute_condition_.notify_all();
}

void MoveGroupMoveAction::cancelCallback(MoveGroupActionServer::GoalHandle goal_handle)
{
  std::lock_guard<std::mutex> slock(goal_mutex_);
  canceled_goals_.push_back(std::make_unique<MoveGroupActionServer::GoalHandle>(goal_handle));
  execute_condition_.notify_all();
}

void MoveGroupMoveAction::executeMoveCallback(std::shared_ptr<MoveGroupActionServer::GoalHandle>& goal_handle_ptr)
{
  goal_handle.setAccepted("This goal has been accepted by the action server");
  goal_handle_ptr->setAccepted("This goal has been accepted by the action server");
  const moveit_msgs::MoveGroupGoalConstPtr& goal = goal_handle_ptr->getGoal();

  setMoveState(PLANNING, *goal_handle_ptr);
  // before we start planning, ensure that we have the latest robot state received...
  context_->planning_scene_monitor_->waitForCurrentRobotState(ros::Time::now());
  context_->planning_scene_monitor_->updateFrameTransforms();

  moveit_msgs::MoveGroupResult action_res;
  if (goal->planning_options.plan_only || !context_->allow_trajectory_execution_)
  {
    if (!goal->planning_options.plan_only)
      ROS_WARN_NAMED(getName(), "This instance of MoveGroup is not allowed to execute trajectories "
                                "but the goal request has plan_only set to false. "
                                "Only a motion plan will be computed anyway.");
    executeMoveCallbackPlanOnly(goal, action_res);
  }
  else
  {
    executeMoveCallbackPlanAndExecute(*goal_handle_ptr, action_res);
  }

  bool planned_trajectory_empty = trajectory_processing::isTrajectoryEmpty(action_res.planned_trajectory);
  std::string response =
      getActionResultString(action_res.error_code, planned_trajectory_empty, goal->planning_options.plan_only);
  if (action_res.error_code.val == moveit_msgs::MoveItErrorCodes::SUCCESS)
    goal_handle_ptr->setSucceeded(action_res, response);
  else
    goal_handle_ptr->setAborted(action_res, response);

  setMoveState(IDLE, *goal_handle_ptr);

  preempt_requested_ = false;
  execute_condition_.notify_all();
}

void MoveGroupMoveAction::executeMoveCallbackPlanAndExecute(MoveGroupActionServer::GoalHandle& goal_handle,
                                                            moveit_msgs::MoveGroupResult& action_res)
{
  ROS_INFO_NAMED(getName(), "Combined planning and execution request received for MoveGroup action. "
                            "Forwarding to planning and execution pipeline.");
  const moveit_msgs::MoveGroupGoalConstPtr& goal = goal_handle.getGoal();

  plan_execution::PlanExecution::Options opt;

  const moveit_msgs::MotionPlanRequest& motion_plan_request =
      moveit::core::isEmpty(goal->request.start_state) ? goal->request : clearRequestStartState(goal->request);
  const moveit_msgs::PlanningScene& planning_scene_diff =
      moveit::core::isEmpty(goal->planning_options.planning_scene_diff.robot_state) ?
          goal->planning_options.planning_scene_diff :
          clearSceneRobotState(goal->planning_options.planning_scene_diff);

  opt.replan_ = goal->planning_options.replan;
  opt.replan_attempts_ = goal->planning_options.replan_attempts;
  opt.replan_delay_ = goal->planning_options.replan_delay;
  opt.before_execution_callback_ = [this, &goal_handle] { startMoveExecutionCallback(goal_handle); };

  opt.plan_callback_ = [this, &motion_plan_request, &goal_handle](plan_execution::ExecutableMotionPlan& plan) {
    return planUsingPlanningPipeline(motion_plan_request, plan, goal_handle);
  };
  if (goal->planning_options.look_around && context_->plan_with_sensing_)
  {
    opt.plan_callback_ = [plan_with_sensing = context_->plan_with_sensing_.get(), planner = opt.plan_callback_,
                          attempts = goal->planning_options.look_around_attempts,
                          safe_execution_cost = goal->planning_options.max_safe_execution_cost](
                             plan_execution::ExecutableMotionPlan& plan) {
      return plan_with_sensing->computePlan(plan, planner, attempts, safe_execution_cost);
    };
    context_->plan_with_sensing_->setBeforeLookCallback(
        [this, &goal_handle] { return startMoveLookCallback(goal_handle); });
  }

  plan_execution::ExecutableMotionPlan plan;
  if (preempt_requested_)
  {
    ROS_INFO_NAMED(getName(), "Preempt requested before the goal is planned and executed.");
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
    return;
  }

  context_->plan_execution_->planAndExecute(plan, planning_scene_diff, opt);

  convertToMsg(plan.plan_components_, action_res.trajectory_start, action_res.planned_trajectory);
  if (plan.executed_trajectory_)
    plan.executed_trajectory_->getRobotTrajectoryMsg(action_res.executed_trajectory);
  action_res.error_code = plan.error_code_;
}

void MoveGroupMoveAction::executeMoveCallbackPlanOnly(const moveit_msgs::MoveGroupGoalConstPtr& goal,
                                                      moveit_msgs::MoveGroupResult& action_res)
{
  ROS_INFO_NAMED(getName(), "Planning request received for MoveGroup action. Forwarding to planning pipeline.");

  // lock the scene so that it does not modify the world representation while diff() is called
  planning_scene_monitor::LockedPlanningSceneRO lscene(context_->planning_scene_monitor_);
  const planning_scene::PlanningSceneConstPtr& the_scene =
      (moveit::core::isEmpty(goal->planning_options.planning_scene_diff)) ?
          static_cast<const planning_scene::PlanningSceneConstPtr&>(lscene) :
          lscene->diff(goal->planning_options.planning_scene_diff);
  planning_interface::MotionPlanResponse res;

  if (preempt_requested_)
  {
    ROS_INFO_NAMED(getName(), "Preempt requested before the goal is planned.");
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::PREEMPTED;
    return;
  }

  // Select planning_pipeline to handle request
  const planning_pipeline::PlanningPipelinePtr planning_pipeline = resolvePlanningPipeline(goal->request.pipeline_id);
  if (!planning_pipeline)
  {
    action_res.error_code.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return;
  }

  try
  {
    planning_pipeline->generatePlan(the_scene, goal->request, res);
  }
  catch (std::exception& ex)
  {
    ROS_ERROR_NAMED(getName(), "Planning pipeline threw an exception: %s", ex.what());
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }

  convertToMsg(res.trajectory_, action_res.trajectory_start, action_res.planned_trajectory);
  action_res.error_code = res.error_code_;
  action_res.planning_time = res.planning_time_;
}

bool MoveGroupMoveAction::planUsingPlanningPipeline(const planning_interface::MotionPlanRequest& req,
                                                    plan_execution::ExecutableMotionPlan& plan,
                                                    MoveGroupActionServer::GoalHandle& goal_handle)
{
  setMoveState(PLANNING, goal_handle);

  bool solved = false;
  planning_interface::MotionPlanResponse res;

  // Select planning_pipeline to handle request
  const planning_pipeline::PlanningPipelinePtr planning_pipeline = resolvePlanningPipeline(req.pipeline_id);
  if (!planning_pipeline)
  {
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
    return solved;
  }

  planning_scene_monitor::LockedPlanningSceneRO lscene(plan.planning_scene_monitor_);
  try
  {
    solved = planning_pipeline->generatePlan(plan.planning_scene_, req, res);
  }
  catch (std::exception& ex)
  {
    ROS_ERROR_NAMED(getName(), "Planning pipeline threw an exception: %s", ex.what());
    res.error_code_.val = moveit_msgs::MoveItErrorCodes::FAILURE;
  }
  if (res.trajectory_)
  {
    plan.plan_components_.resize(1);
    plan.plan_components_[0].trajectory_ = res.trajectory_;
    plan.plan_components_[0].description_ = "plan";
  }
  plan.error_code_ = res.error_code_;
  return solved;
}

void MoveGroupMoveAction::startMoveExecutionCallback(MoveGroupActionServer::GoalHandle& goal_handle)
{
  if (isActive(goal_handle))
    setMoveState(MONITOR, goal_handle);
}

void MoveGroupMoveAction::startMoveLookCallback(MoveGroupActionServer::GoalHandle& goal_handle)
{
  if (isActive(goal_handle))
    setMoveState(LOOK, goal_handle);
}

void MoveGroupMoveAction::setMoveState(MoveGroupState state, MoveGroupActionServer::GoalHandle& goal_handle)
{
  move_state_ = state;
  move_feedback_.state = stateToStr(state);
  goal_handle.publishFeedback(move_feedback_);
}
}  // namespace move_group

#include <class_loader/class_loader.hpp>
CLASS_LOADER_REGISTER_CLASS(move_group::MoveGroupMoveAction, move_group::MoveGroupCapability)
