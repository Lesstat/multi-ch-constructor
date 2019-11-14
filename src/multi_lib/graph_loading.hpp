/*
  Cycle-routing does multi-criteria route planning for bicycles.
  Copyright (C) 2018  Florian Barth

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
#include "graph.hpp"
#include <boost/filesystem.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <chrono>
#include <fstream>
#include <iostream>

using ms = std::chrono::milliseconds;
namespace iostr = boost::iostreams;

Node createNode(std::ifstream& graph, std::ifstream& labels)
{
  size_t id, osmId, level;
  double lat, lng;
  double height;

  graph >> id >> osmId >> lat >> lng >> height >> level;
  labels >> level;

  // std::string external_id = std::to_string(id);
  // external_id.insert(0, "n");

  Node n { id, NodeId { id } }; //, osmId, Lat(lat), Lng(lng), height };

  auto& graph_properties = get_graph_properties();

  put("osmId", graph_properties, id, osmId);
  put("lat", graph_properties, id, lat);
  put("lng", graph_properties, id, lng);
  put("height", graph_properties, id, height);

  n.assignLevel(level);
  return n;
}

Edge createEdge(std::ifstream& ch, std::ifstream& skips)
{

  size_t source, dest;
  double length, height, unsuitability;
  long edgeA, edgeB;

  ch >> source >> dest >> length >> height >> unsuitability;
  skips >> edgeA >> edgeB;

  Edge e { NodeId(source), NodeId(dest) };
  if (edgeA > 0) {
    e.edgeA = EdgeId { static_cast<size_t>(edgeA) };
    e.edgeB = EdgeId { static_cast<size_t>(edgeB) };
  }

  Cost cost(std::array<double, Cost::dim> { length, height, unsuitability });

  e.setCost(cost);
  return e;
}

Graph readMultiFileGraph(std::string graphPath)
{
  using namespace boost::filesystem;

  auto directory = canonical(graphPath).parent_path();

  path chGraph { directory };
  chGraph /= "ch_graph";

  path nodeLabels { directory };
  nodeLabels /= "node_labels";

  path skips { directory };
  skips /= "skips";

  std::ifstream graphFile {};
  graphFile.open(graphPath);

  std::ifstream chFile {};
  chFile.open(chGraph.string());
  std::ifstream nodeLabelsFile { nodeLabels.string() };
  std::ifstream skipsFile { skips.string() };

  std::cout << "Reading Graphdata" << '\n';
  auto start = std::chrono::high_resolution_clock::now();

  std::string line;
  std::getline(graphFile, line);
  while (line.front() == '#') {
    std::getline(graphFile, line);
  }
  size_t nodeCountGraph = 0;
  size_t nodeCountCh = 0;

  graphFile >> nodeCountGraph;
  chFile >> nodeCountCh;

  if (nodeCountGraph != nodeCountCh) {
    throw std::invalid_argument("node counts of ch and graph do not match");
  }

  size_t edgeCount = 0;
  graphFile >> edgeCount;
  chFile >> edgeCount;

  std::vector<Node> nodes;
  nodes.reserve(nodeCountGraph);
  std::vector<Edge> edges;
  edges.reserve(edgeCount);

  for (size_t i = 0; i < nodeCountGraph; ++i) {
    nodes.push_back(createNode(graphFile, nodeLabelsFile));
  }

  for (size_t i = 0; i < edgeCount; ++i) {
    edges.push_back(createEdge(chFile, skipsFile));
  }
  Graph g { std::move(nodes), std::move(edges) };
  auto end = std::chrono::high_resolution_clock::now();

  std::cout << "creating the graph took " << std::chrono::duration_cast<ms>(end - start).count()
            << "ms" << '\n';
  return g;
}

Graph loadGraphFromTextFile(std::string& graphPath, bool zipped)
{
  const size_t N = 256 * 1024;
  char buffer[N];
  std::ifstream graphFile {};
  graphFile.rdbuf()->pubsetbuf((char*)buffer, N);
  graphFile.open(graphPath);

  iostr::filtering_istream in;
  if (zipped)
    in.push(iostr::gzip_decompressor());
  in.push(graphFile);

  std::cout << "Reading Graphdata" << '\n';
  auto start = std::chrono::high_resolution_clock::now();
  Graph g = Graph::createFromStream(in);
  auto end = std::chrono::high_resolution_clock::now();

  std::cout << "creating the graph took " << std::chrono::duration_cast<ms>(end - start).count()
            << "ms" << '\n';
  return g;
}
