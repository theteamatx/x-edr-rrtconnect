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

#ifndef RRTCONNECT_BINARY_SEARCH_H_
#define RRTCONNECT_BINARY_SEARCH_H_

#include <cmath>

#include "eigenmath/types.h"

namespace rrtconnect {

// Ensures that `validation_function` holds along the line segment between
// `qstart` and `qfinal` by checking linearly interpolated points.  Does not
// check endpoints.
// The interpolation uses at least the provided `resolution` per dimension
// between samples.  The points are not visited in sequence, but in a
// binary-tree traversal order.
//
// Imagine you have a number line from 0 to 1, at each step it will
// be divided as follows:
// start  0                                          1 end
// line   |------------------------------------------|
// split1                       |
// split2           |                      |
// split3      |          |          |            |
// ...
// For example, this algorithm is useful as a free space checker.
// This is the eigenmath::VectorXd specific implementation, though it could be
// adapted to support more eigen vector types fairly easily.
//
// RETURNS true if predicate evaluates true on all sampled points, false
// otherwise
template <typename Scalar, int Dimension, typename ValidationFunction>
bool EdgeBinarySearch(const eigenmath::Vector<Scalar, Dimension> &qstart,
                      const eigenmath::Vector<Scalar, Dimension> &qfinal,
                      const Scalar resolution,
                      ValidationFunction validation_function) {
  using Vector = eigenmath::Vector<Scalar, Dimension>;

  const Scalar distance = (qfinal - qstart).template lpNorm<Eigen::Infinity>();
  if (distance <= resolution) {
    // At this resolution, there are no interior points on this line segment.
    return true;
  }

  // Perform a binary-search like traversal in [1, ..., n_steps].
  const int n_steps = std::lrint(std::ceil(distance / resolution));
  // Use evenly spread samples instead of scaling to provided resolution.
  const Vector step = (qfinal - qstart) / n_steps;
  // For a given binary search level, `offset` points to the first binary search
  // bucket and `width` is the size of the bucket.  Start with the next power
  // of 2.
  int width = (1 << std::lrint(std::ceil(std::log2(n_steps + 1))));
  int offset = width / 2;
  while (offset > 0) {
    for (int i = offset; i < n_steps; i += width) {
      const Vector current = qstart + i * step;
      if (!validation_function(current)) {
        return false;
      }
    }
    width /= 2;
    offset /= 2;
  }

  return true;
}

}  // namespace rrtconnect

#endif  // RRTCONNECT_BINARY_SEARCH_H_
