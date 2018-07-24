/*
  Cycle-routing does multi-criteria route planning for bicycles.
  Copyright (C) 2017  Florian Barth

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
#ifndef CONTRACTOR_H
#define CONTRACTOR_H

#include "ndijkstra.hpp"
#include <future>
#include <set>

template <class T> class MultiQueue;

class ContractionLp;

struct EdgePair {
  HalfEdge in;
  HalfEdge out;
};

class Contractor {

  public:
  Contractor() = delete;
  Contractor(bool printStatistics);
  Contractor(const Contractor& other) = default;
  Contractor(Contractor&& other) = default;
  virtual ~Contractor() noexcept;
  Contractor& operator=(const Contractor& other) = delete;
  Contractor& operator=(Contractor&& other) = delete;

  static Edge createShortcut(const Edge& e1, const Edge& e2);

  std::pair<bool, std::optional<RouteWithCount>> isShortestPath(
      NormalDijkstra& d, const HalfEdge& startEdge, const HalfEdge& destEdge, const Config& conf);

  std::future<std::vector<Edge>> contract(
      MultiQueue<EdgePair>& queue, Graph& g, ContractionLp* lp, const std::set<NodePos>& set);
  Graph contract(Graph& g);
  Graph mergeWithContracted(Graph& g);
  Graph contractCompletely(Graph& g, double rest = 2);

  std::set<NodePos> independentSet(const Graph& g);
  std::set<NodePos> reduce(std::set<NodePos>& set, const Graph& g);
  std::set<NodePos> reduce(std::set<NodePos>&& set, const Graph& g) { return reduce(set, g); };

  protected:
  private:
  size_t level = 0;
  std::vector<Node> contractedNodes;
  std::vector<EdgeId> contractedEdges;
  bool printStatistics = false;
  const size_t THREAD_COUNT
      = std::thread::hardware_concurrency() > 0 ? std::thread::hardware_concurrency() : 1;
  std::vector<std::unique_ptr<ContractionLp>> lps;
};

#endif /* CONTRACTOR_H */
