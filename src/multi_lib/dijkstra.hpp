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
#ifndef DIJKSTRA_H
#define DIJKSTRA_H

#include "graph.hpp"
#include <cmath>
#include <iostream>
#include <queue>
#include <random>

using LengthConfig = NamedType<double, struct LengthConfigParameter>;
using HeightConfig = NamedType<double, struct HeightConfigParameter>;
using UnsuitabilityConfig = NamedType<double, struct UnsuitabilityConfigParameter>;

struct Config {
  double values[Cost::dim];

  template <class configType> void asureNonNegativity(configType& c)
  {
    if (c < 0) {
      c = configType{ 0 };
    } else if (!std::isfinite(c)) {
      std::cout << "Masking infinity" << '\n';
      c = configType{ 1 };
    } else if (c > 1) {
      c = configType{ 1 };
    }
  }

  Config(LengthConfig l, HeightConfig h, UnsuitabilityConfig u)
  {
    values[0] = l;
    values[1] = h;
    values[2] = u;
    asureNonNegativity(values[0]);
    asureNonNegativity(values[1]);
    asureNonNegativity(values[2]);
  }
  Config(const std::vector<double>& values)
  {
    for (size_t i = 0; i < Cost::dim; ++i) {
      this->values[i] = values[i];
      asureNonNegativity(this->values[i]);
    }
  }

  bool operator==(Config& other)
  {
    for (size_t i = 0; i < Cost::dim; ++i) {
      if (values[i] != other.values[i]) {
        return false;
      }
    }
    return true;
  }

  Config integerValues() const
  {
    std::vector<double> intValues;
    for (size_t i = 0; i < Cost::dim; ++i) {
      intValues.push_back(std::round(values[i] * 100));
    }
    return intValues;
  }
};

Config generateRandomConfig();

std::ostream& operator<<(std::ostream& stream, const Config& c);

struct Route {
  Cost costs;
  std::deque<Edge> edges;
};

class Dijkstra {
  public:
  Dijkstra(Graph* g, size_t nodeCount);
  Dijkstra(const Dijkstra& other) = default;
  Dijkstra(Dijkstra&& other) noexcept = default;
  virtual ~Dijkstra() noexcept = default;
  Dijkstra& operator=(const Dijkstra& other) = default;
  Dijkstra& operator=(Dijkstra&& other) noexcept = default;

  std::optional<Route> findBestRoute(NodePos from, NodePos to, Config config);

  size_t pqPops = 0;

  private:
  using QueueElem = std::pair<NodePos, double>;
  struct QueueComparator {
    bool operator()(QueueElem left, QueueElem right);
  };
  using Queue = std::priority_queue<QueueElem, std::vector<QueueElem>, QueueComparator>;

  void clearState();

  using NodeToEdgeMap = std::unordered_map<NodePos, HalfEdge>;
  Route buildRoute(NodePos node, NodeToEdgeMap previousEdgeS, NodeToEdgeMap previousEdgeT,
      NodePos from, NodePos to);

  enum class Direction { S, T };

  void relaxEdges(
      const NodePos& node, double cost, Direction dir, Queue& heap, NodeToEdgeMap& previousEdge);

  bool stallOnDemand(const NodePos& node, double cost, Direction dir);

  std::vector<double> costS;
  std::vector<double> costT;
  std::vector<NodePos> touchedS;
  std::vector<NodePos> touchedT;
  Config config = Config(LengthConfig(0), HeightConfig(0), UnsuitabilityConfig(0));
  Graph* graph;
};

#endif /* DIJKSTRA_H */
