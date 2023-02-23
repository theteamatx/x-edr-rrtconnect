// Copyright 2023 Google LLC

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     https://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef RRTCONNECT_RRT_CONNECT_H_
#define RRTCONNECT_RRT_CONNECT_H_

#include <utility>

#include "rrtconnect/rrt.h"

namespace rrtconnect {

// RRT-Connect planner.
// Plans a configuration space path between an initial and a goal configuration.
// This implements the RRT connect planner described in the paper:
//
//  James J. Kuffner, and Steven M. LaValle. "RRT-connect: An efficient approach
//  to single-query path planning". IEEE Conference on Robotics and Automation
//  (ICRA), 2000.
//
// The node validity function (e.g. collision check, etc.) is provided by the
// user.
// Calling Plan() will compute a motion plan from the given initial
// configuration to the goal, in a way that all states in the path satisfy the
// user-given validity function.
//
// Incorporates search time bounds and predictibility improvements based on the
// paper:
//
//  Deterministic Sampling-Based Motion Planning: Optimality, Complexity,
//  and Performance (2016) Lucas Janson et. al. https://arxiv.org/abs/1505.00023
//
// A static method is also provided for stateless planning.
class RRTConnectPlanner {
 public:
  // Default maximum configuration-space step in radians.
  static constexpr double kDefaultConfigurationSpaceStep = 0.1;

  // Constructor.
  // max_iters: the maximum number of iterations to use when sampling new
  //   positions.
  // timeout_s: the maximum time (in seconds) to spend planning, set to
  //   zero to disable.
  // configuration_space_step: no configuration space step will be larger in
  //   magnitude than this value.
  // iters_between_goal_checks: the number of samples between occasions
  //   when a direct path to the goal is tried. If <2 it is never tried.
  // start_tree: If not null, the RRT tree for the start configuration is
  //   initialized with this value, and after completion this value is set to
  //   the final start configuration tree.
  // goal_tree: If not null, the RRT tree for the goal configuration is
  //   initialized with this value, and after completion this value is set to
  //   the final goal configuration tree.
  explicit RRTConnectPlanner(
      std::size_t max_iters = 10000, double timeout_s = 0.0,
      double configuration_space_step = kDefaultConfigurationSpaceStep,
      int iters_between_goal_checks = 3, RRT::Graph *start_tree = nullptr,
      RRT::Graph *goal_tree = nullptr)
      : max_iters_(max_iters),
        timeout_s_(timeout_s),
        configuration_space_step_(configuration_space_step),
        start_tree_(start_tree),
        goal_tree_(goal_tree) {}

  // Default destructor.
  ~RRTConnectPlanner() = default;

  // Sets the maximum number of iterations for the planner.
  // max_iters: the maximum number of iterations to use when sampling new
  // positions.
  void SetMaxIterations(std::size_t max_iters) { max_iters_ = max_iters; }

  // Sets a timeout.
  // The planner will return with an empty path if not plan found within the
  // given timeout.
  // timeout_s: the maximum time (in seconds) to spend planning, or zero
  //   to disable the timeout.
  void SetTimeout(double timeout_s) { timeout_s_ = timeout_s; }

  // Sets the maximum step between configuration waypoints.
  // No configuration space step will be larger in magnitude than the given
  // value.
  void SetJointSpaceStep(double configuration_space_step) {
    configuration_space_step_ = configuration_space_step;
  }

  // Sets how often to sample the goal configuration directly.
  // iters: sample the goal configuration every 'iters' iterations.
  void SetIterationsBetweenGoalChecks(int iters) {
    iters_between_goal_checks_ = iters;
  }

  // The starting tree to use.
  // If non-null this value is used to initialize the start tree of the search,
  // and written to after the search is complete.
  void SetStartTree(RRT::Graph *start_tree) { start_tree_ = start_tree; }

  // The goal tree to use.
  // If non-null this value is used to initialize the start tree of the search,
  // and written to after the search is complete.
  void SetEndTree(RRT::Graph *goal_tree) { goal_tree_ = goal_tree; }

  // Plans from an initial configuration to the given goal.
  // q_initial: an initial configuration.
  // q_goal: a goal configuration.
  // lower_imits: the lower limits in configuration space.
  // upper_limits: the upper limits in configuration space.
  // user_sample_validator: edge/node validation function.
  // Returns a PlanResult, with the path satisfying the lower/upper limits and
  // validation function.
  PlanResult Plan(const ConfigWaypoint &q_initial, const ConfigWaypoint &q_goal,
                  const ConfigWaypoint &lower_limits,
                  const ConfigWaypoint &upper_limits,
                  const RRT::ValidationFunction &user_sample_validator) {
    return Plan(q_initial, q_goal, lower_limits, upper_limits,
                user_sample_validator, max_iters_, timeout_s_,
                configuration_space_step_, iters_between_goal_checks_,
                start_tree_, goal_tree_);
  }

  // Plans a path between two points in the configuration space.
  // q_initial: the initial configuration.
  // q_goal: the goal configuration.
  // lower_limits: the lower limits in configuration space.
  // upper_limits: the upper limits in configuration space.
  // user_sample_validator:  the edge/node validation function.
  // iters: The number of iterations to use, when sampling new positions.
  // timeout_s: The maximum time (in seconds) to spend planning, or zero to
  //   not use any timeout.
  // configuration_space_step: No configuration space step will be larger in
  //   magnitude than this value.
  // iters_between_goal_checks: the number of samples between occasions
  //   when a direct path to the goal is tried. If <2 it is never tried.
  // start_tree: If not null, the RRT tree for the start configuration is
  //   initialized with this value, and after completion this value is set to
  //   the final start configuration tree.
  // goal_tree: If not null, the RRT tree for the goal configuration is
  //   initialized with this value, and after completion this value is set to
  //   the final goal configuration tree.
  // Returns a PlanResult, with the path satisfying the lower/upper limits and
  // validation function.
  static PlanResult Plan(
      const ConfigWaypoint &q_initial, const ConfigWaypoint &q_goal,
      const ConfigWaypoint &lower_limits, const ConfigWaypoint &upper_limits,
      const RRT::ValidationFunction &user_sample_validator, int iters,
      double timeout_s, double configuration_space_step,
      int iters_between_goal_checks, RRT::Graph *start_tree = nullptr,
      RRT::Graph *goal_tree = nullptr);

 protected:
  std::size_t max_iters_ = 10000;
  double timeout_s_ = 0.0;
  double configuration_space_step_ = kDefaultConfigurationSpaceStep;
  int iters_between_goal_checks_ = 3;
  RRT::Graph *start_tree_ = nullptr;
  RRT::Graph *goal_tree_ = nullptr;
};

}  // namespace rrtconnect

#endif  // RRTCONNECT_RRT_CONNECT_H_
