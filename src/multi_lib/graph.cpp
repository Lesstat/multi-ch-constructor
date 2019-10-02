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

#include "ndijkstra.hpp"
#include <future>
#include <iomanip>

#include <boost/property_map/property_map.hpp>

void Graph::connectEdgesToNodes(const std::vector<Node>& nodes, const std::vector<EdgeId>& edges)
{
  if (nodes.empty()) {
    return;
  }
  inEdges.reserve(edges.size());
  outEdges.reserve(edges.size());

  const auto max_id = std::max_element(
      nodes.begin(), nodes.end(), [](const auto& a, const auto& b) { return a.id() < b.id(); });

  std::vector<NodePos> map(max_id->id() + 1, NodePos { 0 });
  for (size_t i = 0; i < nodes.size(); ++i) {
    map[nodes[i].id()] = NodePos { i };
  }

  std::for_each(begin(edges), end(edges), [&map, this](const auto& id) {
    auto& e = Edge::getMutEdge(id);
    auto sourcePos = map[e.getSourceId()];
    auto destPos = map[e.getDestId()];
    inEdges.push_back(e.makeHalfEdge(destPos, sourcePos));
    outEdges.push_back(e.makeHalfEdge(sourcePos, destPos));
    e.sourcePos(sourcePos);
    e.destPos(destPos);
  });
}

enum class Pos { source, dest };
void sortEdgesByNodePos(std::vector<HalfEdge>& edges, const Graph& g)
{
  auto comparator = [&g](const HalfEdge& a, const HalfEdge& b) {
    if (a.begin == b.begin) {
      auto aLevel = g.getLevelOf(a.end);
      auto bLevel = g.getLevelOf(b.end);
      if (aLevel == bLevel) {
        return a.end < b.end;
      }
      return aLevel > bLevel;
    }
    return a.begin < b.begin;
  };
  std::sort(edges.begin(), edges.end(), comparator);
}

void calculateOffsets(
    std::vector<HalfEdge>& edges, std::vector<NodeOffset>& offsets, Pos p, const Graph& g)
{
  auto sourcePos = [&edges](size_t j) { return edges[j].begin; };
  auto destPos = [&edges](size_t j) { return edges[j].begin; };
  auto setOut = [&offsets](size_t i, size_t j) { offsets[i].out = j; };
  auto setIn = [&offsets](size_t i, size_t j) { offsets[i].in = j; };

  auto getPos
      = [p, sourcePos, destPos](size_t j) { return p == Pos::source ? sourcePos(j) : destPos(j); };

  auto setOffset
      = [p, setOut, setIn](size_t i, size_t j) { p == Pos::source ? setOut(i, j) : setIn(i, j); };

  sortEdgesByNodePos(edges, g);

  size_t lastNode = 0;

  for (size_t i = 0; i < edges.size(); ++i) {
    NodePos curNode = getPos(i);
    for (size_t j = lastNode + 1; j < curNode + 1; ++j) {
      setOffset(j, i);
    }
    lastNode = curNode;
  }

  for (size_t i = lastNode + 1; i < offsets.size(); ++i) {
    setOffset(i, edges.size());
  }
}

Graph::Graph(std::vector<Node>&& nodes, std::vector<Edge>&& edges)
    : edgeCount(edges.size())
{
  auto ids = Edge::administerEdges(std::move(edges));
  init(std::move(nodes), std::move(ids));
}
Graph::Graph(std::vector<Node>&& nodes, std::vector<EdgeId>&& edges)
    : edgeCount(edges.size())
{
  init(std::move(nodes), std::move(edges));
}
void Graph::init(std::vector<Node>&& nodes, std::vector<EdgeId>&& edges)
{
  std::stable_sort(nodes.begin(), nodes.end(),
      [](const Node& a, const Node& b) { return a.getLevel() < b.getLevel(); });

  connectEdgesToNodes(nodes, edges);
  level.reserve(nodes.size());
  for (const auto& node : nodes) {
    level.push_back(node.getLevel());
  }
  this->nodes = std::move(nodes);
  offsets.reserve(this->nodes.size() + 1);
  for (size_t i = 0; i < this->nodes.size() + 1; ++i) {
    offsets.emplace_back(NodeOffset {});
  }

  auto fut = std::async(
      std::launch::async, [&]() { calculateOffsets(outEdges, offsets, Pos::source, *this); });
  calculateOffsets(inEdges, offsets, Pos::dest, *this);
  fut.wait();
}

std::ostream& operator<<(std::ostream& s, const Graph& g)
{

  s << "Node positions" << '\n';
  for (auto const& node : g.nodes) {
    s << node << ", ";
  }
  s << '\n';
  s << "offsets for out" << '\n';
  for (auto off : g.offsets) {
    s << off.out << ", ";
  }
  s << '\n';
  s << "offsets for in" << '\n';
  for (auto off : g.offsets) {
    s << off.in << ", ";
  }
  return s;
}

std::vector<NodeOffset> const& Graph::getOffsets() const { return offsets; }

Dijkstra Graph::createDijkstra() { return Dijkstra { this, nodes.size() }; }

NormalDijkstra Graph::createNormalDijkstra(bool unpack)
{
  return NormalDijkstra { this, nodes.size(), unpack };
}

size_t readCount(std::istream& file)
{
  std::string line;
  std::getline(file, line);
  return std::stoi(line);
}

template <class Obj> void parseLines(std::vector<Obj>& v, std::istream& file, size_t count)
{
  std::string line {};
  for (size_t i = 0; i < count; ++i) {
    std::getline(file, line);
    v.push_back(Obj::createFromText(line));
  }
}

Graph Graph::createFromStream(std::istream& file)
{
  file >> std::setprecision(7);
  std::vector<Node> nodes {};
  std::vector<Edge> edges {};
  std::string line {};

  std::getline(file, line);
  while (line.front() == '#') {
    std::getline(file, line);
  }

  size_t dim = readCount(file);
  if (dim != Cost::dim) {
    std::cerr << "Graph file is of dimension " << dim << " code is compiled for dimension "
              << Cost::dim << '\n';
    throw std::runtime_error("Graph has wrong dimension");
  }
  size_t nodeCount = readCount(file);
  nodes.reserve(nodeCount);

  size_t edgeCount = readCount(file);
  edges.reserve(edgeCount);

  parseLines(nodes, file, nodeCount);
  parseLines(edges, file, edgeCount);

  return Graph { std::move(nodes), std::move(edges) };
}

const Node& Graph::getNode(NodePos pos) const { return nodes[pos]; }

EdgeRange Graph::getOutgoingEdgesOf(NodePos pos) const
{
  auto start = outEdges.begin();
  std::advance(start, offsets[pos].out);
  auto end = outEdges.begin();
  std::advance(end, offsets[pos + 1].out);
  return EdgeRange { start, end };
}

EdgeRange Graph::getIngoingEdgesOf(NodePos pos) const
{
  auto start = inEdges.begin();
  std::advance(start, offsets[pos].in);
  auto end = inEdges.begin();
  std::advance(end, offsets[pos + 1].in);
  return EdgeRange { start, end };
}

size_t Graph::getLevelOf(NodePos pos) const { return level[pos]; }

std::optional<NodePos> Graph::nodePosById(NodeId id) const
{
  for (size_t i = 0; i < nodes.size(); ++i) {
    if (nodes[i].id() == id) {
      return NodePos { i };
    }
  }
  return {};
}

size_t Graph::getNodeCount() const { return nodes.size(); }

size_t Graph::getEdgeCount() const { return edgeCount; }

size_t Graph::getInTimesOutDegree(NodePos node) const
{
  auto& offset = offsets[node];
  return offset.in * offset.out;
}

std::unordered_map<NodeId, const Node*> Graph::getNodePosByIds(
    const std::unordered_set<NodeId>& ids) const
{
  std::unordered_map<NodeId, const Node*> result;
  result.reserve(ids.size());
  for (const auto& node : nodes) {
    if (ids.find(node.id()) != ids.end()) {
      result[node.id()] = &node;
    }
  }
  return result;
}

NodePos Graph::getNodePos(const Node* n) const
{
  return NodePos { static_cast<size_t>(n - nodes.data()) };
}

void Graph::writeToStream(std::ostream& out) const
{
  out << std::setprecision(7);
  out << Cost::dim << '\n';
  out << nodes.size() << '\n';
  out << inEdges.size() << '\n';
  for (const auto& node : nodes) {
    node.writeToStream(out);
  }

  std::vector<EdgeId> edges;
  edges.reserve(inEdges.size());

  std::transform(inEdges.begin(), inEdges.end(), std::back_inserter(edges),
      [](const auto& e) { return e.id; });

  std::sort(edges.begin(), edges.end(), [](auto& a, auto& b) { return a.get() < b.get(); });

  for (const auto& id : edges) {
    auto& edge = Edge::getEdge(id);
    edge.writeToStream(out);
  }
}

std::optional<boost::dynamic_properties> graph_properties;
std::map<std::string, size_t> node2osmid;
std::map<std::string, double> node2lat;
std::map<std::string, double> node2lng;
std::map<std::string, double> node2height;

boost::dynamic_properties& get_graph_properties()
{
  if (!graph_properties) {
    graph_properties = boost::dynamic_properties();

    boost::associative_property_map<decltype(node2osmid)> osmId_map(node2osmid);
    graph_properties->property("osmId", osmId_map);

    boost::associative_property_map<decltype(node2lat)> lat_map(node2lat);
    graph_properties->property("lat", lat_map);

    boost::associative_property_map<decltype(node2lng)> lng_map(node2lng);
    graph_properties->property("lng", lng_map);

    boost::associative_property_map<decltype(node2height)> height_map(node2height);
    graph_properties->property("height", height_map);
  }

  return *graph_properties;
};
