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

#include "rrtconnect/path_metric.h"

#include <cmath>
#include <limits>
#include <optional>
#include <string>

#include "absl/log/check.h"
#include "absl/log/log.h"
#include "rrtconnect/binary_search.h"
#include "rrtconnect/rrt.h"

namespace rrtconnect {

double BinaryValidationMetric::Compute(const ConfigWaypoint& q1,
                                       const ConfigWaypoint& q2) {
  double cost = 0.0;

  // Validator that runs the validation function on a waypoint.
  auto validator = [this](const ConfigWaypoint& p_to_check) -> bool {
    return !validation_function_ ||
           validation_function_(std::optional<ConfigWaypoint>{}, p_to_check);
  };

  // Run the validator for waypoints in the path from q1 to q2.
  if (!EdgeBinarySearch(q1, q2, configuration_space_step_, validator)) {
    cost = std::numeric_limits<double>::infinity();
  }

  return cost;
}

ConfigPath ShortcutPath(const ConfigPath& input, PathMetric* metric) {
  CHECK(metric != nullptr);
  CHECK(input.size() >= 2) << "Need at least 2 waypoints";
  ConfigPath output;
  output.push_back(input[0]);

  // This is the point which we're always considering shortcutting.
  ConfigWaypoint middle_point = input[1];

  // Metric from the last output point to the middle point.
  double to_middle_metric = metric->Compute(input[0], input[1]);
  // Metric from the middle point to the next point under consideration.
  double from_middle_metric = 0.0;

  std::string debug_string = "[X";

  for (size_t i = 2; i < input.size(); ++i) {
    from_middle_metric = metric->Compute(middle_point, input[i]);

    // Metric from the last output point to the next point.
    double shortcut_metric = metric->Compute(output.back(), input[i]);

    // If the shortcut is worse, the middle point needs to be in our output
    // trajectory.
    if (std::isinf(shortcut_metric) ||
        shortcut_metric > to_middle_metric + from_middle_metric) {
      output.push_back(middle_point);
      debug_string += "X";
      to_middle_metric = from_middle_metric;
    } else {
      // We're skipping the middle point.
      to_middle_metric = shortcut_metric;
      debug_string += "_";
    }
    middle_point = input[i];
  }
  // Add the point that's left over in the end.
  output.push_back(middle_point);
  debug_string += "X]";

  LOG(INFO) << "Path shortcutter: " << debug_string;
  return output;
}

}  // namespace rrtconnect