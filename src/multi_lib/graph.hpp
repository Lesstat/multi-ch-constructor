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
#ifndef GRAPH_H
#define GRAPH_H

#include "namedType.hpp"

#include <atomic>
#include <boost/property_map/dynamic_property_map.hpp>
#include <fstream>
#include <iostream>
#include <optional>
#include <set>
#include <unordered_set>
#include <vector>

boost::dynamic_properties& get_graph_properties();

using OsmId = NamedType<size_t, struct OsmIdParameter>;
using NodeId = NamedType<size_t, struct NodeIdParameter>;
using NodePos = NamedType<size_t, struct NodePosParameter>;
using EdgeId = NamedType<size_t, struct EdgeParameter>;
using Lat = NamedType<double, struct LatParameter>;
using Lng = NamedType<double, struct LngParameter>;
using Height = NamedType<double, struct HeightParameter>;
using Length = NamedType<double, struct LengthParameter>;
using Unsuitability = NamedType<double, struct UnsuitabilityParameter>;

class Dijkstra;
class NormalDijkstra;
struct Config;

struct Cost {
  static const size_t dim = 3;
  std::array<double, dim> values;
  Cost(const std::vector<double>& values)
  {
    for (size_t i = 0; i < dim; ++i) {
      this->values[i] = values[i];
      if (std::abs(this->values[i]) < 0.0001) {
        this->values[i] = 0;
      }
    }
  }
  Cost()
  {
    for (size_t i = 0; i < Cost::dim; ++i) {
      values[i] = 0;
    }
  }
  Cost(const Cost& other) { values = other.values; }
  Cost operator=(const Cost& other)
  {
    values = other.values;
    return *this;
  }

  Cost(Cost&& other) noexcept { values = other.values; }
  Cost operator=(Cost&& other) noexcept
  {
    values = other.values;
    return *this;
  }

  Cost(const std::array<double, Cost::dim> values)
  {
    for (size_t i = 0; i < dim; ++i) {
      this->values[i] = values[i];
      if (std::abs(this->values[i]) < 0.0001) {
        this->values[i] = 0;
      }
    }
  }
  double operator*(const Config& conf) const;

  Cost operator+(const Cost& c) const
  {
    std::array<double, Cost::dim> newValues;
    for (size_t i = 0; i < dim; ++i) {
      newValues[i] = values[i] + c.values[i];
    }
    return newValues;
  };
  Cost operator-(const Cost& c) const
  {
    std::array<double, Cost::dim> newValues;
    for (size_t i = 0; i < dim; ++i) {
      newValues[i] = values[i] - c.values[i];
    }
    return newValues;
  };

  bool operator==(const Cost& c) const
  {
    for (size_t i = 0; i < Cost::dim; ++i) {
      if (std::abs(values[i] - c.values[i]) >= 0.0001) {
        return false;
      }
    }
    return true;
  };
  bool operator!=(const Cost& c) const { return !(*this == c); }
};

struct HalfEdge {
  EdgeId id;
  NodePos end;
  NodePos begin;
  Cost cost;

  double costByConfiguration(const Config& conf) const;
};

struct NodeOffset {
  size_t in { 0 };
  size_t out { 0 };
  NodeOffset() = default;
  NodeOffset(size_t in, size_t out)
      : in(in)
      , out(out)
  {
  }
};

using ReplacedEdge = std::optional<EdgeId>;

class Edge {
  public:
  Edge() = default;
  Edge(NodeId source, NodeId dest);
  Edge(NodeId source, NodeId dest, ReplacedEdge edgeA, ReplacedEdge edgeB);
  Edge(const Edge& other) = default;
  Edge(Edge&& other) noexcept = default;
  virtual ~Edge() noexcept = default;
  Edge& operator=(const Edge& other) = default;
  Edge& operator=(Edge&& other) noexcept = default;

  NodeId getSourceId() const;
  NodeId getDestId() const;
  const ReplacedEdge& getEdgeA() const;
  const ReplacedEdge& getEdgeB() const;

  NodePos sourcePos() const;
  NodePos destPos() const;
  void sourcePos(NodePos source);
  void destPos(NodePos dest);

  EdgeId getId() const;
  void setId(EdgeId id);
  const Cost& getCost() const;
  double costByConfiguration(const Config& conf) const;
  void setCost(Cost c);

  HalfEdge makeHalfEdge(NodePos begin, NodePos end) const;

  static Edge createFromText(const std::string& text);
  void writeToStream(std::ostream& out) const;
  static std::vector<EdgeId> administerEdges(std::vector<Edge>&& edges);
  static const Edge& getEdge(EdgeId id);
  static Edge& getMutEdge(EdgeId id);

  friend void testEdgeInternals(const Edge& e, NodeId source, NodeId destination, Length length,
      Height height, Unsuitability unsuitability, const ReplacedEdge& edgeA,
      const ReplacedEdge& edgeB);
  friend Edge createEdge(std::ifstream&, std::ifstream&);

  private:
  friend class boost::serialization::access;

  std::string external_id_;
  EdgeId internalId;
  NodeId source;
  NodeId destination;
  Cost cost;
  ReplacedEdge edgeA;
  ReplacedEdge edgeB;
  NodePos sourcePos_;
  NodePos destPos_;

  static std::vector<Edge> edges;
};

class Node {
  public:
  Node() = default;
  Node(const std::string& external_node_id,
      NodeId id); //, size_t osmId, Lat lat, Lng lng, double height);
  Node(const Node& other) = default;
  Node(Node&& other) noexcept = default;
  virtual ~Node() noexcept = default;
  Node& operator=(const Node& other) = default;
  Node& operator=(Node&& other) noexcept = default;

  void assignLevel(size_t level);
  size_t getLevel() const;
  NodeId id() const;
  friend std::ostream& operator<<(std::ostream& os, const Node& n);

  static Node createFromText(const std::string& text);
  void writeToStream(std::ostream& out) const;
  friend void testNodeInternals(const Node& n, NodeId id, Lat lat, Lng lng, size_t level);

  // Lat lat() const;
  // Lng lng() const;
  // short height() const;

  private:
  friend class boost::serialization::access;

  std::string external_node_id_;
  NodeId id_;
  // size_t osmId;
  // Lat lat_;
  // Lng lng_;
  size_t level = 0;
  // double height_;
};

class Grid;
class EdgeRange;
class Graph {
  public:
  Graph(std::vector<Node>&& nodes, std::vector<Edge>&& edges);
  Graph(std::vector<Node>&& nodes, std::vector<EdgeId>&& edges);
  Graph(const Graph& other) = delete;
  Graph(Graph&& other) noexcept = default;
  virtual ~Graph() noexcept = default;
  Graph& operator=(const Graph& other) = delete;
  Graph& operator=(Graph&& other) noexcept = default;

  friend std::ostream& operator<<(std::ostream& /*s*/, const Graph& /*g*/);

  std::vector<NodeOffset> const& getOffsets() const;
  Dijkstra createDijkstra();
  NormalDijkstra createNormalDijkstra(bool unpack = false);
  Grid createGrid(long sideLength = 100) const;

  EdgeRange getOutgoingEdgesOf(NodePos pos) const;
  EdgeRange getIngoingEdgesOf(NodePos pos) const;

  size_t getLevelOf(NodePos pos) const;

  static Graph createFromStream(std::istream& file);
  void writeToStream(std::ostream& out) const;

  const Node& getNode(NodePos pos) const;
  std::optional<NodePos> nodePosById(NodeId id) const;

  size_t getNodeCount() const;
  size_t getEdgeCount() const;

  size_t getInTimesOutDegree(NodePos node) const;

  std::unordered_map<NodeId, const Node*> getNodePosByIds(
      const std::unordered_set<NodeId>& ids) const;

  NodePos getNodePos(const Node* n) const;

  private:
  void init(std::vector<Node>&& nodes, std::vector<EdgeId>&& edges);
  void connectEdgesToNodes(const std::vector<Node>& nodes, const std::vector<EdgeId>& edges);

  std::vector<Node> nodes;
  std::vector<NodeOffset> offsets;
  std::vector<HalfEdge> inEdges;
  std::vector<HalfEdge> outEdges;
  std::vector<size_t> level;
  size_t edgeCount;
};

class EdgeRange {
  public:
  using iterator = std::vector<HalfEdge>::const_iterator;

  EdgeRange(iterator begin, iterator end)
      : begin_(begin)
      , end_(end)
  {
  }
  EdgeRange(const EdgeRange& other) = default;
  EdgeRange(EdgeRange&& other) noexcept = default;
  virtual ~EdgeRange() noexcept = default;
  EdgeRange& operator=(const EdgeRange& other) = default;
  EdgeRange& operator=(EdgeRange&& other) noexcept = default;

  iterator begin() const { return begin_; }
  iterator end() const { return end_; }

  protected:
  private:
  iterator begin_;
  iterator end_;
};

struct RouteWithCount;
struct Route;

void printRoutes(std::ofstream& dotFile, const Graph& graph, const RouteWithCount& route1,
    const Route& route2, const Config& config, const std::set<NodePos>& set = {});
#endif /* GRAPH_H */
