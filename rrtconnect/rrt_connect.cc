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

#include "rrtconnect/rrt_connect.h"

#include <functional>
#include <memory>
#include <optional>

#include "absl/log/log.h"
#include "absl/time/clock.h"
#include "absl/time/time.h"
#include "eigenmath/quasi_random_vector.h"

namespace rrtconnect {

PlanResult RRTConnectPlanner::Plan(
    const ConfigWaypoint &q_initial, const ConfigWaypoint &q_goal,
    const ConfigWaypoint &lower_limits, const ConfigWaypoint &upper_limits,
    const RRT::ValidationFunction &user_sample_validator, int iters,
    double timeout_s, double configuration_space_step,
    int iters_between_goal_checks, RRT::Graph *start_tree,
    RRT::Graph *goal_tree) {
  auto sample_validator = [&user_sample_validator](
                              const absl::optional<ConfigWaypoint> &q_origin,
                              const ConfigWaypoint &q_dest) -> bool {
    return !user_sample_validator || user_sample_validator(q_origin, q_dest);
  };

  // Early return if initial or goal state are not valid.
  if (!sample_validator(std::optional<ConfigWaypoint>{}, q_initial)) {
    LOG(INFO) << "The initial state is not valid. Cannot plan.";
    return PlanResult();
  }

  if (!sample_validator(std::optional<ConfigWaypoint>{}, q_goal)) {
    LOG(INFO) << "The goal state is not valid. Cannot plan.";
    return PlanResult();
  }

  // Create RRTs and plan.
  RRT t_start(q_initial, q_goal, configuration_space_step,
              start_tree != nullptr ? *start_tree : RRT::Graph{});
  RRT t_goal(q_goal, q_initial, configuration_space_step,
             goal_tree != nullptr ? *goal_tree : RRT::Graph{});

  ConfigWaypoint tmp_start = q_initial;
  ConfigWaypoint tmp_goal = q_goal;

  RRT *t_a = &t_start, *t_b = &t_goal;
  int not_trapped_count = 0;
  absl::Time start_time = absl::Now();
  eigenmath::QuasiRandomVectorGenerator generator(lower_limits, upper_limits);
  for (int i = 0; i < iters; ++i) {
    double elapsed_time_s = (absl::Now() - start_time) / absl::Seconds(1);
    if (timeout_s > 0.0 && elapsed_time_s > timeout_s) {
      LOG(INFO) << "Motion planning timeout (" << timeout_s << " seconds).";
      return {.path = ConfigPath(), .planning_iterations = i};
    }
    RRT::Vertex q_rand;

    // Always select random points if iters is <1
    // otherwise select the goal point first and then on every nth
    // occasion, where n is stored in iters_between_goal_checks.
    if (iters_between_goal_checks < 1 ||
        ((i > 0) && (i % iters_between_goal_checks != 0))) {
      // trying to extend RRT 'a' towards a quasirandomly selected
      // low dispersion point, which provides better repeatability
      // and time bounds than a truly random point.
      q_rand = generator();
    } else {
      // try going towards the goal directly on occasion.
      q_rand = tmp_goal;
    }

    RRT::ExtendResult result_a =
        t_a->Extend(q_rand, t_a->NearestNeighbor(q_rand),
                    std::bind(sample_validator, std::placeholders::_1,
                              std::placeholders::_2));
    if (result_a.extend_code != RRT::ExtendCode::kTrapped) {
      // extension successful, new vertex 'result_a.vertex_desc' was added to
      // RRT 'a'.
      ++not_trapped_count;
      RRT::Vertex q_new = t_a->GetVertex(result_a.vertex_desc);
      // trying to connect RRT 'b' to new vertex 'q_new'.
      RRT::ExtendResult result_b =
          t_b->Connect(q_new, std::bind(sample_validator, std::placeholders::_1,
                                        std::placeholders::_2));
      if (result_b.extend_code == RRT::ExtendCode::kReached) {
        LOG(INFO) << "Found path in " << i << " iterations.";
        if (start_tree != nullptr) {
          *start_tree = t_start.GetGraph();
        }

        if (goal_tree != nullptr) {
          *goal_tree = t_goal.GetGraph();
        }

        if (t_a == &t_start)
          return {.path = RRT::FindPath(*t_a, *t_b, result_a.vertex_desc,
                                        result_b.vertex_desc),
                  .planning_iterations = i};
        else
          return {.path = RRT::FindPath(*t_b, *t_a, result_b.vertex_desc,
                                        result_a.vertex_desc),
                  .planning_iterations = i};
      }
    }
    // Swap pointers, thus growing RRTs from both sides.
    std::swap(t_a, t_b);
    tmp_start.swap(tmp_goal);
  }
  LOG(INFO) << "No path found between start and goal in " << iters
            << " iterations.";
  LOG(INFO) << "Not trapped count " << not_trapped_count << "/" << iters;
  return {.path = ConfigPath(), .planning_iterations = iters};
}

}  // namespace rrtconnect
