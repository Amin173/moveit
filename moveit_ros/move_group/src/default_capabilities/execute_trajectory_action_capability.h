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

/*
 * Capability of execute trajectory with a ROS action.
 *
 * Author: Kentaro Wada
 * */

#pragma once

#include <moveit/move_group/move_group_capability.h>
#include <actionlib/server/action_server.h>
#include <moveit_msgs/ExecuteTrajectoryAction.h>
#include <moveit/controller_manager/controller_manager.h>

#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <thread>

typedef actionlib::ActionServer<moveit_msgs::ExecuteTrajectoryAction> ExecuteTrajectoryActionServer;
namespace move_group
{
class MoveGroupExecuteTrajectoryAction : public MoveGroupCapability
{
public:
  MoveGroupExecuteTrajectoryAction();
  ~MoveGroupExecuteTrajectoryAction();

  void initialize() override;

private:
  void executeLoop();
  bool isActive(ExecuteTrajectoryActionServer::GoalHandle& goal_handle);
  void cancelGoal(ExecuteTrajectoryActionServer::GoalHandle& goal);
  void goalCallback(ExecuteTrajectoryActionServer::GoalHandle goal_handle);
  void cancelCallback(ExecuteTrajectoryActionServer::GoalHandle goal_handle);
  void clearInactiveGoals();

  void executePath(std::shared_ptr<ExecuteTrajectoryActionServer::GoalHandle> goal_handle_ptr);
  void setExecuteTrajectoryState(const MoveGroupState& state, ExecuteTrajectoryActionServer::GoalHandle& goal);
  void sendGoalResponse(ExecuteTrajectoryActionServer::GoalHandle& goal,
                        const moveit_controller_manager::ExecutionStatus& execution_status);

  std::mutex goal_mutex_;
  ExecuteTrajectoryActionServer::GoalHandle current_goal_;
  std::deque<std::shared_ptr<ExecuteTrajectoryActionServer::GoalHandle>> new_goals_;
  std::deque<std::unique_ptr<ExecuteTrajectoryActionServer::GoalHandle>> canceled_goals_;
  std::map<std::shared_ptr<ExecuteTrajectoryActionServer::GoalHandle>, std::unique_ptr<std::thread>> active_goals_;
  std::mutex active_goals_mutex_;

  std::unique_ptr<std::thread> execute_thread_;
  std::condition_variable execute_condition_;
  std::mutex execute_mutex_;

  std::mutex terminate_mutex_;
  bool need_to_terminate_;

  std::unique_ptr<ExecuteTrajectoryActionServer> execute_action_server_;
};

}  // namespace move_group
