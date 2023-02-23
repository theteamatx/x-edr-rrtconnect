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

#include "rrtconnect/binary_search.h"

#include <vector>

#include "eigenmath/types.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace rrtconnect {
namespace {

using ::testing::ElementsAreArray;
using ::testing::IsEmpty;

TEST(EdgeBinarySearch, DoesNotVisitPointForIdenticalEndpoints) {
  std::vector<eigenmath::Vector2d> visited;
  const eigenmath::Vector2d point(0.0, 2.0);
  EXPECT_TRUE(EdgeBinarySearch(point, point, 1.0,
                               [&](const eigenmath::Vector2d& point) {
                                 visited.push_back(point);
                                 return true;
                               }));
  EXPECT_THAT(visited, IsEmpty());
}

TEST(EdgeBinarySearch, DoesNotVisitPointsIfResolutionExceedsDistance) {
  std::vector<eigenmath::Vector2d> visited;
  const eigenmath::Vector2d from(0.0, 2.0);
  const eigenmath::Vector2d to(4.0, 6.0);
  EXPECT_TRUE(
      EdgeBinarySearch(from, to, 4.0, [&](const eigenmath::Vector2d& point) {
        visited.push_back(point);
        return true;
      }));
  EXPECT_THAT(visited, IsEmpty());
}

TEST(EdgeBinarySearch, VisitsPointsInExpectedOrderPerfectTree) {
  std::vector<eigenmath::Vector2d> visited;
  const eigenmath::Vector2d from(0.0, 2.0);
  const eigenmath::Vector2d to(4.0, 6.0);
  EXPECT_TRUE(
      EdgeBinarySearch(from, to, 1.0, [&](const eigenmath::Vector2d& point) {
        visited.push_back(point);
        return true;
      }));
  const eigenmath::Vector2d expected_order[] = {
      {2.0, 4.0},
      {1.0, 3.0},
      {3.0, 5.0},
  };
  EXPECT_THAT(visited, ElementsAreArray(expected_order));
}

TEST(EdgeBinarySearch, VisitsPointsInExpectedOrderUnbalancedTree) {
  std::vector<eigenmath::Vector2d> visited;
  const eigenmath::Vector2d from(0.0, 2.0);
  const eigenmath::Vector2d to(6.0, 8.0);
  EXPECT_TRUE(
      EdgeBinarySearch(from, to, 1.0, [&](const eigenmath::Vector2d& point) {
        visited.push_back(point);
        return true;
      }));
  const eigenmath::Vector2d expected_order[] = {
      {4.0, 6.0}, {2.0, 4.0}, {1.0, 3.0}, {3.0, 5.0}, {5.0, 7.0},
  };
  EXPECT_THAT(visited, ElementsAreArray(expected_order));
}

TEST(EdgeBinarySearch, InterruptsWhenPredicateReturnsFalse) {
  const eigenmath::Vector2d from(0.0, 2.0);
  const eigenmath::Vector2d to(4.0, 6.0);
  int count = 0;
  EXPECT_FALSE(
      EdgeBinarySearch(from, to, 0.1, [&](const eigenmath::Vector2d& point) {
        if (count == 5) {
          return false;
        }
        ++count;
        return true;
      }));
  EXPECT_EQ(count, 5);
}

}  // namespace
}  // namespace rrtconnect
