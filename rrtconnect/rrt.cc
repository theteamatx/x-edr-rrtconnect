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

#include "rrtconnect/rrt.h"

#include <algorithm>
#include <iterator>
#include <limits>
#include <queue>
#include <tuple>
#include <unordered_set>
#include <utility>

#include "absl/container/flat_hash_set.h"
#include "absl/container/node_hash_set.h"

namespace rrtconnect {
namespace {
using RRTGraphTraits = boost::graph_traits<RRT::Graph>;
}

class RRT::Impl {
 public:
  Impl(const Vertex &q_start, const Vertex &q_goal, double eps,
       const Graph &graph);
  ~Impl() = default;

  const RRT::Graph &GetGraph() const { return graph_; }
  ExtendResult Extend(const Vertex &q, const VertexDesc q_near_id,
                      const ValidationFunction &validity_func);
  ExtendResult Connect(const Vertex &q,
                       const ValidationFunction &validity_func);
  VertexDesc NearestNeighbor(const Vertex &q) const;
  const Vertex &GetVertex(VertexDesc vd) const { return graph_[vd]; }
  static VertexPath FindPath(const Impl &t_a, const Impl &t_b, VertexDesc q_a,
                             VertexDesc q_b);

  VertexDesc AddVertex(const Vertex &q);
  void AddEdge(VertexDesc q_0, VertexDesc q_1) {
    boost::add_edge(q_0, q_1, graph_);
  }
  Vertex NewConfig(const Vertex &q, const Vertex &q_near) const;
  VertexPath Walk(VertexDesc vd) const;

  Graph graph_;
  double eps_;
  Vertex q_goal_;
  VertexDesc start_;
};

RRT::Impl::Impl(const Vertex &q_start, const Vertex &q_goal, double eps,
                const Graph &graph)
    : graph_(graph), eps_(eps), q_goal_(q_goal) {
  RRTGraphTraits::vertex_iterator v_i, v_end;
  bool found = false;
  for (std::tie(v_i, v_end) = vertices(graph_); v_i != v_end; ++v_i) {
    found = (GetVertex(*v_i) == q_start);
    if (found) {
      break;
    }
  }

  start_ = found ? *v_i : AddVertex(q_start);
}

// Iterate from the current position to the nearby one, extending
// the graph with each successful validation check.
//
// This approach ensures there aren't any duplicate validation checks
// and the graph grows as much as possible for every validation check
// that identifies the pose valid.
RRT::ExtendResult RRT::Impl::Extend(const Vertex &q, const VertexDesc q_near_id,
                                    const ValidationFunction &validity_func) {
  // This is a variant on the original RRT extend function.
  // Here, the tree continues to take more than one step towards the random
  // configuration as long it is valid and closer to the goal than the previous
  // one.
  VertexDesc temp_q_near_id = q_near_id;
  bool go_forward = true;
  while (go_forward) {
    const Vertex &q_near = GetVertex(temp_q_near_id);
    Vertex q_new = NewConfig(q, q_near);
    if ((!validity_func || validity_func(q_near, q_new))) {
      double near_dist2goal = (q_goal_ - q_near).norm();
      double new_dist2goal = (q_goal_ - q_new).norm();
      VertexDesc q_new_id = AddVertex(q_new);
      AddEdge(temp_q_near_id, q_new_id);
      // If q_new approx. q, we reached the vertex q.
      if ((q_new - q).norm() < eps_) {
        return {ExtendCode::kReached, q_new_id};
      } else if (new_dist2goal < near_dist2goal) {
        temp_q_near_id = q_new_id;
      } else {
        return {ExtendCode::kAdvanced, q_new_id};
      }
    } else {
      go_forward = false;
    }
  }
  if (temp_q_near_id == q_near_id) {
    return {ExtendCode::kTrapped, temp_q_near_id};
  } else {
    return {ExtendCode::kAdvanced, temp_q_near_id};
  }
}

RRT::ExtendResult RRT::Impl::Connect(const Vertex &q,
                                     const ValidationFunction &validity_func) {
  ExtendResult r;
  r.vertex_desc = NearestNeighbor(q);
  int extend_count = 0;
  do {
    r = Extend(q, r.vertex_desc, validity_func);
    ++extend_count;
  } while (r.extend_code == ExtendCode::kAdvanced);
  return r;
}

// We could have a more efficient nearest-neighbor implementation.
RRT::VertexDesc RRT::Impl::NearestNeighbor(const Vertex &q) const {
  VertexDesc best_desc = 0;
  double best_dist = std::numeric_limits<double>::max();
  RRTGraphTraits::vertex_iterator v_i, v_end;

  for (std::tie(v_i, v_end) = vertices(graph_); v_i != v_end; ++v_i) {
    double dist_i = (GetVertex(*v_i) - q).squaredNorm();
    if (dist_i < best_dist) {
      best_dist = dist_i;
      best_desc = *v_i;
    }
  }
  return best_desc;
}

RRT::VertexPath RRT::Impl::FindPath(const RRT::Impl &t_a, const RRT::Impl &t_b,
                                    VertexDesc q_a, VertexDesc q_b) {
  VertexPath path_a_mid = t_a.Walk(q_a);
  VertexPath path_b_mid = t_b.Walk(q_b);
  path_b_mid.pop_back();
  path_a_mid.insert(path_a_mid.end(), path_b_mid.rbegin(), path_b_mid.rend());
  return path_a_mid;
}

RRT::VertexDesc RRT::Impl::AddVertex(const Vertex &q) {
  VertexDesc vd = boost::add_vertex(q, graph_);
  return vd;
}

// Move q towards q_near with a step of magnitude eps_.
RRT::Vertex RRT::Impl::NewConfig(const Vertex &q, const Vertex &q_near) const {
  Vertex q_new;

  double dist = (q - q_near).norm();
  if (dist <= eps_) {
    q_new = q;
  } else {
    Vertex dir = (q - q_near) / dist;
    q_new = q_near + dir * eps_;
  }
  return q_new;
}

RRT::VertexPath RRT::Impl::Walk(VertexDesc vd_start) const {
  std::queue<std::pair<VertexDesc, RRT::VertexPath>> unexpanded;
  absl::flat_hash_set<VertexDesc> expanded = {};

  unexpanded.push({vd_start, RRT::VertexPath{}});

  // BFS
  while (!unexpanded.empty()) {
    auto [vertex_desc, path] = unexpanded.front();
    unexpanded.pop();
    expanded.insert(vertex_desc);
    path.push_back(GetVertex(vertex_desc));

    if (start_ == vertex_desc) {
      std::reverse(std::begin(path), std::end(path));
      return path;
    } else {
      for (auto [iter, end] = in_edges(vertex_desc, graph_); iter != end;
           ++iter) {
        VertexDesc vd = boost::source(*iter, graph_);
        if (expanded.find(vd) == expanded.end()) {
          unexpanded.push({vd, path});
        }
      }
    }
  }

  return {GetVertex(vd_start)};
}

/******************* Public interface impl **********************************/

RRT::RRT(const Vertex &q_start, const Vertex &q_goal, double eps,
         const Graph &graph)
    : impl_(new Impl(q_start, q_goal, eps, graph)) {}
RRT::~RRT() = default;

const RRT::Graph &RRT::GetGraph() const { return impl_->GetGraph(); }
const RRT::Vertex &RRT::GetStart() const { return GetVertex(impl_->start_); }
const RRT::Vertex &RRT::GetGoal() const { return impl_->q_goal_; }

RRT::ExtendResult RRT::Extend(const Vertex &q, const VertexDesc q_near_id,
                              const ValidationFunction &validity_func) {
  return impl_->Extend(q, q_near_id, validity_func);
}

RRT::ExtendResult RRT::Connect(const Vertex &q,
                               const ValidationFunction &validity_func) {
  return impl_->Connect(q, validity_func);
}

RRT::VertexDesc RRT::NearestNeighbor(const Vertex &q) const {
  return impl_->NearestNeighbor(q);
}

const RRT::Vertex &RRT::GetVertex(VertexDesc vd) const {
  return impl_->GetVertex(vd);
}

RRT::VertexPath RRT::FindPath(const RRT &t_a, const RRT &t_b, VertexDesc q_a,
                              VertexDesc q_b) {
  return RRT::Impl::FindPath(*t_a.impl_, *t_b.impl_, q_a, q_b);
}
}  // namespace rrtconnect
