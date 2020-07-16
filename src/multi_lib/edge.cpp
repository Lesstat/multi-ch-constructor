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
#include "dijkstra.hpp"
#include "graph.hpp"
#include <algorithm>
#include <cassert>
#include <cstdlib>
#include <sstream>

std::vector<Edge> Edge::edges {};

Edge::Edge(NodeId source, NodeId dest)
    : Edge(source, dest, {}, {})
{
}

Edge::Edge(NodeId source, NodeId dest, ReplacedEdge edgeA, ReplacedEdge edgeB)
    : source(source)
    , destination(dest)
    , edgeA(std::move(edgeA))
    , edgeB(std::move(edgeB))
{
  if (edgeA && source == dest) {
    std::cout << "same source and target for shortcuts not allowed: " << source << " , " << dest
              << '\n';
    std::exit(1);
  }
  if (edges.size() > 0 && (edgeA || edgeB)) {
    auto& e1 = Edge::getEdge(*edgeA);
    auto& e2 = Edge::getEdge(*edgeB);
    if (e1.getSourceId() == e2.getSourceId() && e1.getSourceId() != e1.getDestId()) {
      std::cerr << "Same starting point" << '\n';
      std::terminate();
    }
  }
}

NodeId Edge::getSourceId() const { return source; }
NodeId Edge::getDestId() const { return destination; }

Edge Edge::createFromText(const std::string& text)
{

  std::string external_id = "";
  size_t source, dest;
  std::array<double, Cost::dim> cost;
  long edgeA, edgeB;

  std::stringstream ss(text);

  if (use_external_edge_ids_) {
    ss >> external_id;
  }
  ss >> source >> dest;
  for (auto& c : cost) {
    ss >> c;
  }
  ss >> edgeA >> edgeB;

  Edge e { NodeId(source), NodeId(dest) };
  e.set_external_id(external_id);

  if (edgeA > 0) {
    e.edgeA = EdgeId { static_cast<size_t>(edgeA) };
    e.edgeB = EdgeId { static_cast<size_t>(edgeB) };
  }
  e.cost = Cost(cost);
  for (double c : e.cost.values) {
    if (0 > c) {
      throw std::invalid_argument("Cost below zero: " + std::to_string(c));
    }
  }
  return e;
}

void Edge::writeToStream(std::ostream& out) const
{
  if (use_external_edge_ids_) {
    out << external_id_ << ' ';
  }
  if (use_node_osm_ids_) {
    const auto& graph_properties = get_graph_properties();
    std::string src_osm_id = get<std::string>("osmId", graph_properties, getSourceId().get());
    std::string dst_osm_id = get<std::string>("osmId", graph_properties, getDestId().get());
    out << src_osm_id << ' ' << dst_osm_id;
  } else {
    out << source << ' ' << destination;
  }
  for (const auto& c : cost.values) {
    out << ' ' << c;
  }
  out << ' ';
  if (edgeA && edgeB) {
    out << edgeA.value() << ' ' << edgeB.value();
  } else {
    out << "-1 -1";
  }
  out << '\n';
}

double Edge::costByConfiguration(const Config& conf) const { return cost * conf; }

EdgeId Edge::getId() const { return internalId; }

const Cost& Edge::getCost() const { return cost; }

void Edge::setCost(Cost c) { this->cost = c; }

const ReplacedEdge& Edge::getEdgeA() const { return edgeA; }
const ReplacedEdge& Edge::getEdgeB() const { return edgeB; }

HalfEdge Edge::makeHalfEdge(NodePos begin, NodePos end) const
{
  HalfEdge e;
  e.id = internalId;
  e.begin = begin;
  e.end = end;
  e.cost = cost;
  return e;
}

void Edge::setId(EdgeId id)
{
  this->internalId = id;
  if (this->external_id_.empty()) {
    this->external_id_ = std::to_string(id);
    if (edgeA && edgeB) {
      this->external_id_.insert(0, "ch");
    } else {
      this->external_id_.insert(0, "e");
    }
  }
}

void Edge::set_external_id(const std::string& external_id) { external_id_ = external_id; }

const std::string& Edge::external_id() const { return external_id_; }

double HalfEdge::costByConfiguration(const Config& conf) const { return cost * conf; }

std::vector<EdgeId> Edge::administerEdges(std::vector<Edge>&& edges)
{
  std::vector<EdgeId> ids;
  ids.reserve(edges.size());
  for (size_t i = 0; i < edges.size(); ++i) {
    size_t new_id = Edge::edges.size() + i;
    auto& edge = edges[i];
    if (edge.getId() == 0) {
      edge.setId(EdgeId { new_id });
    } else if (edge.getId() != new_id) {
      std::cerr << "Edge ids dont align: " << '\n';
      std::terminate();
    }
    if (edge.external_id().empty()) {
      edge.set_external_id(std::to_string(new_id));
    }
    ids.emplace_back(new_id);
  }
  if (edges.size() > Edge::edges.capacity() - Edge::edges.size()) {
    Edge::edges.reserve(Edge::edges.size() + 3 * edges.size());
  }
  std::move(edges.begin(), edges.end(), std::back_inserter(Edge::edges));
  edges = std::vector<Edge>();
  return ids;
}

const Edge& Edge::getEdge(EdgeId id) { return edges.at(id); }
Edge& Edge::getMutEdge(EdgeId id) { return edges.at(id); }

double Cost::operator*(const Config& conf) const
{
  double combinedCost = 0;

  for (size_t i = 0; i < dim; ++i) {
    combinedCost += values[i] * conf.values[i];
  }

  if (!(combinedCost >= 0)) {
    std::cout << "Cost < 0 detected" << '\n';
    for (size_t i = 0; i < dim; ++i) {
      std::cout << "metric " << i << ": " << values[i] << " * " << conf.values[i] << '\n';
    }
    throw std::invalid_argument("cost < 0");
  }
  return combinedCost;
}

NodePos Edge::sourcePos() const { return sourcePos_; }
NodePos Edge::destPos() const { return destPos_; }
void Edge::sourcePos(NodePos source) { sourcePos_ = source; }
void Edge::destPos(NodePos dest) { destPos_ = dest; }

bool Edge::valid() const
{
  if (!edgeA && !edgeB) {
    return true;
  } else if (edgeA && edgeB) {
    auto e1 = Edge::getEdge(*edgeA);
    auto e2 = Edge::getEdge(*edgeB);
    if (source != e1.source) {
      throw std::invalid_argument("Shortcut has different source than e1");
    }
    if (destination != e2.destination) {
      throw std::invalid_argument("Shortcut has different target than e2");
    }

    if (sourcePos_ != e1.sourcePos_) {
      throw std::invalid_argument("Shortcut has different source pos than e1");
    }
    if (destPos_ != e2.destPos_) {
      throw std::invalid_argument("Shortcut has different target pos than e2");
    }
    if (e1.destination != e2.source) {
      throw std::invalid_argument("Replaced edges are not connected");
    }

    if (e1.destPos_ != e2.sourcePos_) {
      throw std::invalid_argument("Replaced edges are not connected via position");
    }

  } else {
    throw std::invalid_argument("created half shortcut");
  }
  return true;
}

bool Edge::use_node_osm_ids_ = false;
bool Edge::use_external_edge_ids_ = false;

void Edge::write_osm_id_of_nodes(bool value) { use_node_osm_ids_ = value; }
void Edge::use_external_edge_ids(bool value) { use_external_edge_ids_ = value; }
