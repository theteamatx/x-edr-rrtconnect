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

#ifndef RRTCONNECT_PATH_METRIC_H_
#define RRTCONNECT_PATH_METRIC_H_

#include <utility>

#include "rrtconnect/rrt.h"
#include "rrtconnect/rrt_connect.h"

namespace rrtconnect {

// An abstract path metric class.
// A path metric computes a cost value for a configuration space path.
class PathMetric {
 public:
  virtual ~PathMetric() = default;

  // Computes the metric for a given path.
  // path: a path in the configuration space.
  // Returns the metric result over the path.
  virtual double Compute(const ConfigPath& path) {
    double cost = 0.0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
      cost += Compute(path[i], path[i + 1]);
    }
    return cost;
  }

  // Computes the metric between two configurations.
  // q1: a first point in the configuration space.
  // q2: a second point in the configuration space.
  // Returns the metric result between the two waypoints.
  virtual double Compute(const ConfigWaypoint& q1,
                         const ConfigWaypoint& q2) = 0;
};

// A simple binary path metric.
// Assigns either cost 0 if the path is valid, or infinity if the path is
// invalid. Path validity is determined by a user-provided validity function
// (e.g. a collision checker).
class BinaryValidationMetric : public PathMetric {
 public:
  // Constructor.
  // configuration_space_step: path sampling resolution. We assume that for any
  //   two valid points that are closer than this to each other, the entire
  //   segment connecting them is also valid.
  // validation_function: a user-given edge/node validation function.
  explicit BinaryValidationMetric(
      double configuration_space_step,
      const RRT::ValidationFunction& validation_function)
      : validation_function_(validation_function),
        configuration_space_step_(configuration_space_step) {}

  // Default destructor.
  ~BinaryValidationMetric() override = default;

  // Computes the binary validation metric between two configurations.
  // Samples the space between both configurations and runs the validator for
  // the intermediate samples.
  // q1: a first point in the configuration space.
  // q2: a second point in the configuration space.
  // Returns zero if the path from q1 to q2 is valid, infinity otherwise.
  double Compute(const ConfigWaypoint& q1, const ConfigWaypoint& q2) override;

 private:
  RRT::ValidationFunction validation_function_ = nullptr;
  const double configuration_space_step_ =
      RRTConnectPlanner::kDefaultConfigurationSpaceStep;
};

// Shortcut a path based on a given metric.
// input: an input path in configuration space.
// metric: a metric to use for shortcutting.
// Returns a shorter path according to the provided metric.
ConfigPath ShortcutPath(const ConfigPath& input, PathMetric* metric);

// Shortcut a path based on the BinaryValidationMetric.
// Allows to pass in a user-defined validation function to validate samples.
inline ConfigPath ShortcutPathWithUserValidator(
    const ConfigPath& input, const RRT::ValidationFunction& validation_function,
    double configuration_space_step =
        RRTConnectPlanner::kDefaultConfigurationSpaceStep) {
  BinaryValidationMetric metric(configuration_space_step, validation_function);
  return ShortcutPath(input, &metric);
}

}  // namespace rrtconnect

#endif  // RRTCONNECT_PATH_METRIC_H_
