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

#include "eigenmath/pose3.h"
#include "eigenmath/pose3_utils.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "rrtconnect/path_metric.h"

namespace rrtconnect {
namespace {

// Parameters of a planar 2-links robot.
constexpr int kNumberOfJoints = 2;
constexpr double kFirstLinkLength = 0.5;
constexpr double kSecondLinkLength = 0.5;

// Forward kinematics on the planar 2-links robot.
eigenmath::Pose3d ForwardKinematics(double j0, double j1) {
  return eigenmath::RotationZ(j0) *
         eigenmath::Translation(0.0, kFirstLinkLength, 0.0) *
         eigenmath::RotationZ(j1) *
         eigenmath::Translation(0.0, kSecondLinkLength, 0.0);
}

// Should find a direct path to the goal.
TEST(RRTConnectTest, PlanDirect) {
  ConfigWaypoint init_q = eigenmath::VectorXd::Zero(kNumberOfJoints);
  ConfigWaypoint goal_q = eigenmath::MakeVector({-M_PI_2, 0.0});

  ConfigWaypoint lower_joint_limits = eigenmath::MakeVector({-M_PI_2, -M_PI_2});
  ConfigWaypoint upper_joint_limits = eigenmath::MakeVector({M_PI_2, M_PI_2});

  RRTConnectPlanner planner;
  planner.SetMaxIterations(1000);
  planner.SetJointSpaceStep(0.1);

  auto empty_validator = [](const absl::optional<eigenmath::VectorXd> &q_origin,
                            const eigenmath::VectorXd &q_dest) { return true; };

  PlanResult plan_result = planner.Plan(init_q, goal_q, lower_joint_limits,
                                        upper_joint_limits, empty_validator);
  EXPECT_GT(plan_result.path.size(), 0);
  EXPECT_EQ(plan_result.planning_iterations, 0);
}

// Should find a path around a fake obstacle.
TEST(RRTConnectTest, PlanAroundObstacle) {
  ConfigWaypoint init_q = eigenmath::VectorXd::Zero(kNumberOfJoints);
  ConfigWaypoint goal_q = eigenmath::MakeVector({-M_PI_2, 0.0});

  ConfigWaypoint lower_joint_limits = eigenmath::MakeVector({-M_PI_2, -M_PI_2});
  ConfigWaypoint upper_joint_limits = eigenmath::MakeVector({M_PI_2, M_PI_2});

  RRTConnectPlanner planner;
  planner.SetMaxIterations(1000);
  planner.SetJointSpaceStep(0.1);

  // Fake an obstacle at the tip pose when J0=45 deg.
  auto obstacle_validator =
      [](const absl::optional<eigenmath::VectorXd> &q_origin,
         const eigenmath::VectorXd &q_dest) {
        constexpr double kSphereObstacleRadius = 0.2;
        eigenmath::Pose3d root_pose_tip =
            ForwardKinematics(q_dest[0], q_dest[1]);
        double distance_to_obstacle = (root_pose_tip.translation() -
                                       eigenmath::Vector3d(0.707, 0.707, 0.0))
                                          .norm();
        return distance_to_obstacle > kSphereObstacleRadius;
      };

  PlanResult plan_result = planner.Plan(init_q, goal_q, lower_joint_limits,
                                        upper_joint_limits, obstacle_validator);
  EXPECT_GT(plan_result.path.size(), 0);
  EXPECT_GT(plan_result.planning_iterations, 0);
}

// Should reach the maximum number of iterations before finding a path.
TEST(RRTConnectTest, MaxIterations) {
  ConfigWaypoint init_q = eigenmath::VectorXd::Zero(kNumberOfJoints);
  ConfigWaypoint goal_q = eigenmath::MakeVector({-M_PI_2, 0.0});

  ConfigWaypoint lower_joint_limits = eigenmath::MakeVector({-M_PI_2, -M_PI_2});
  ConfigWaypoint upper_joint_limits = eigenmath::MakeVector({M_PI_2, M_PI_2});

  RRTConnectPlanner planner;
  constexpr int kMaxNumberOfIterations = 10;
  planner.SetMaxIterations(kMaxNumberOfIterations);
  planner.SetJointSpaceStep(0.1);

  auto obstacle_validator =
      [](const absl::optional<eigenmath::VectorXd> &q_origin,
         const eigenmath::VectorXd &q_dest) {
        // Fake an obstacle at the tip pose when J0=45 deg.
        constexpr double kSphereObstacleRadius = 0.2;
        eigenmath::Pose3d root_pose_tip =
            ForwardKinematics(q_dest[0], q_dest[1]);
        double distance_to_obstacle = (root_pose_tip.translation() -
                                       eigenmath::Vector3d(0.707, 0.707, 0.0))
                                          .norm();
        return distance_to_obstacle > kSphereObstacleRadius;
      };

  PlanResult plan_result = planner.Plan(init_q, goal_q, lower_joint_limits,
                                        upper_joint_limits, obstacle_validator);
  EXPECT_EQ(plan_result.path.size(), 0);
  EXPECT_EQ(plan_result.planning_iterations, kMaxNumberOfIterations);
}

TEST(RRTConnectTest, PathShortcut) {
  ConfigWaypoint init_q = eigenmath::VectorXd::Zero(kNumberOfJoints);
  ConfigWaypoint goal_q = eigenmath::MakeVector({-M_PI_2, 0.0});

  ConfigWaypoint lower_joint_limits = eigenmath::MakeVector({-M_PI_2, -M_PI_2});
  ConfigWaypoint upper_joint_limits = eigenmath::MakeVector({M_PI_2, M_PI_2});

  RRTConnectPlanner planner;
  planner.SetMaxIterations(1000);
  constexpr double kJointSpaceStep = 0.1;
  planner.SetJointSpaceStep(kJointSpaceStep);

  // Fake an obstacle at the tip pose when J0=45 deg.
  auto obstacle_validator =
      [](const absl::optional<eigenmath::VectorXd> &q_origin,
         const eigenmath::VectorXd &q_dest) {
        constexpr double kSphereObstacleRadius = 0.2;
        eigenmath::Pose3d root_pose_tip =
            ForwardKinematics(q_dest[0], q_dest[1]);
        double distance_to_obstacle = (root_pose_tip.translation() -
                                       eigenmath::Vector3d(0.707, 0.707, 0.0))
                                          .norm();
        return distance_to_obstacle > kSphereObstacleRadius;
      };

  PlanResult plan_result = planner.Plan(init_q, goal_q, lower_joint_limits,
                                        upper_joint_limits, obstacle_validator);
  EXPECT_GT(plan_result.path.size(), 0);
  EXPECT_GT(plan_result.planning_iterations, 0);

  ConfigPath shorter_path = ShortcutPathWithUserValidator(
      plan_result.path, obstacle_validator, kJointSpaceStep);
  EXPECT_GT(shorter_path.size(), 0);
  EXPECT_LT(shorter_path.size(), plan_result.path.size());
}

}  // namespace
}  // namespace rrtconnect
