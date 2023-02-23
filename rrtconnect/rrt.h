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

#ifndef RRTCONNECT_RRT_H_
#define RRTCONNECT_RRT_H_

#include <functional>
#include <memory>
#include <utility>
#include <vector>

#include "absl/types/optional.h"
#include "boost/graph/adjacency_list.hpp"
#include "eigenmath/types.h"

namespace rrtconnect {

using ConfigWaypoint = eigenmath::VectorXd;
using ConfigPath = std::vector<eigenmath::VectorXd>;

// Result of a plan.
struct PlanResult {
  // A path in the configuration space that reaches the goal.
  ConfigPath path;
  // The number of iterations used to compute path.
  int planning_iterations = 0;
};

// A Rapidly-exploring Random Tree (RRT) implementation.
// A basic RRT with a generic configuration space (sequence of doubles) and
// validation function (e.g. collision checker, etc.).
class RRT {
 public:
  enum class ExtendCode { kReached, kAdvanced, kTrapped };

  using Vertex = ConfigWaypoint;
  using VertexPath = ConfigPath;
  using VertexDesc = std::size_t;

  // This type represents the internal tree used by RRT. It is exposed here to
  // allow for extracting/importing graphs to/from other compatible RRTs.
  using Graph =
      boost::adjacency_list<boost::vecS,            // OutEdgeList.
                            boost::vecS,            // VertexList.
                            boost::bidirectionalS,  // for walking in edges.
                            Vertex,                 // vertex property.
                            boost::no_property>;

  // Signature of a function to be used as a Vertex validator.
  // Caller should validate q_dest, and optionally validate that q_origin
  // (if present) has a valid transition to q_dest.
  using ValidationFunction = std::function<bool(
      const absl::optional<Vertex>& q_origin, const Vertex& q_dest)>;

  struct ExtendResult {
    ExtendCode extend_code;
    VertexDesc vertex_desc;
  };

  // Constructor.
  // q_start: start vertex.
  // q_goal: goal vertex.
  // eps: the maximum step between two vertex.
  RRT(const Vertex& q_start, const Vertex& q_goal, double eps,
      const Graph& graph = {});

  // Destructor.
  ~RRT();

  // No copy constructor.
  RRT(const RRT& other) = delete;

  // No assignment operator.
  RRT& operator=(const RRT& other) = delete;

  // Returns the built out tree as a graph.
  const Graph& GetGraph() const;

  // Returns the start vertex.
  const Vertex& GetStart() const;

  // Returns the goal vertex.
  const Vertex& GetGoal() const;

  // Implements the Extend algorithm.
  // Tries to extend the graph towards q starting from q_near_id (the nearest
  // vertex in the graph to q), validating new states with a user-defined
  // function.
  // q: the configuration to reach
  // q_near_id: the initial vertex, already in the RRT.
  // validity_func: a used-provided function to validate intermediate vertex.
  // Returns a Extend result (kReached, kAdvanced, kTrapped) and the new vertex
  // id.
  ExtendResult Extend(const Vertex& q, VertexDesc q_near_id,
                      const ValidationFunction& validity_func);

  // Implements the Connect algorithm.
  // Calls extend() repeteadly as long as the extend result is kAdvanced.
  // Internally finds the nearest neighbor to the desired configuration.
  // q: the configuration to reach.
  // validity_func: a used-provided function to validate intermediate vertex.
  // Returns a Extend result (kReached, kAdvanced, kTrapped) and the new vertex
  // id.
  ExtendResult Connect(const Vertex& q,
                       const ValidationFunction& validity_func);

  // Finds the nearest neighbor to a given configuration.
  // q: the configuration for which to find the nearest neighbor.
  // Returns a vertex id, already in the RRT, that is closest to q.
  VertexDesc NearestNeighbor(const Vertex& q) const;

  // Gets a vertex from its id.
  // vd: the vertex description.
  // Returns the vertex corresponding to the given description.
  const Vertex& GetVertex(VertexDesc vd) const;

  // Finds a path between two RRTs passing through the given vertices.
  // t_a: the first RRT.
  // t_b: the second RRT.
  // q_a: a vertex in the first RRT.
  // q_b: a vertex in the second RRT.
  // Returns a vector of vertices that join both RRTs.
  static VertexPath FindPath(const RRT& t_a, const RRT& t_b, VertexDesc q_a,
                             VertexDesc q_b);

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rrtconnect

#endif  // RRTCONNECT_RRT_H_
