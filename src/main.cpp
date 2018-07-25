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
#include "contractor.hpp"
#include "dijkstra.hpp"
#include "graph_loading.hpp"
#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <random>

Graph contractGraph(Graph& g, double rest, bool printStats, size_t maxThreads)
{
  Contractor c{ printStats, maxThreads };
  auto start = std::chrono::high_resolution_clock::now();
  Graph ch = c.contractCompletely(g, rest);
  auto end = std::chrono::high_resolution_clock::now();
  using m = std::chrono::minutes;
  std::cout << "contracting the graph took " << std::chrono::duration_cast<m>(end - start).count()
            << " minutes" << '\n';
  return ch;
}

std::string return_current_time_and_date()
{
  auto now = std::chrono::system_clock::now();
  auto in_time_t = std::chrono::system_clock::to_time_t(now);

  std::stringstream ss;
  ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
  return ss.str();
}

using ms = std::chrono::milliseconds;

int testGraph(Graph& g)
{
  Dijkstra d = g.createDijkstra();
  NormalDijkstra n = g.createNormalDijkstra(true);
  std::random_device rd{};
  std::uniform_int_distribution<size_t> dist(0, g.getNodeCount() - 1);
  Config c{ LengthConfig{ 1.0 / 3.0 }, HeightConfig{ 1.0 / 3.0 },
    UnsuitabilityConfig{ 1.0 / 3.0 } };

  size_t route = 0;
  size_t noRoute = 0;

  size_t dTime = 0;
  size_t nTime = 0;

  for (int i = 0; i < 200; ++i) {
    NodePos from{ dist(rd) };
    NodePos to{ dist(rd) };

    auto dStart = std::chrono::high_resolution_clock::now();
    auto dRoute = d.findBestRoute(from, to, c);
    auto dEnd = std::chrono::high_resolution_clock::now();
    auto nRoute = n.findBestRoute(from, to, c);
    auto nEnd = std::chrono::high_resolution_clock::now();

    if (dRoute && nRoute) {
      auto normalTime = std::chrono::duration_cast<ms>(nEnd - dEnd).count();
      auto chTime = std::chrono::duration_cast<ms>(dEnd - dStart).count();
      std::cout << "ND/CH: " << normalTime << "/" << chTime << " = "
                << (chTime > 0 ? normalTime / chTime : 999999999999999) << '\n';
      dTime += chTime;
      nTime += normalTime;
      ++route;
      if (std::abs(dRoute->costs * c - nRoute->costs * c) > 0.1) {
        std::cout << '\n'
                  << "cost differ in route from " << from << " (" << g.getNode(from).id() << ") to "
                  << to << " (" << g.getNode(to).id() << ")" << '\n';
        std::cout << "Edge count: " << nRoute->edges.size() << '\n';
        std::cout << "dLength: " << dRoute->costs.values[0]
                  << ", nLength: " << nRoute->costs.values[0] << '\n';
        std::cout << "dHeight: " << dRoute->costs.values[1]
                  << ", nHeight: " << nRoute->costs.values[1] << '\n';
        std::cout << "dUnsuitability: " << dRoute->costs.values[2]
                  << ", nUnsuitability: " << nRoute->costs.values[2] << '\n';
        std::cout << "total cost d: " << dRoute->costs * c
                  << ", total cost n: " << nRoute->costs * c << '\n';
        std::unordered_set<NodeId> nodeIds;
        std::transform(nRoute->edges.begin(), nRoute->edges.end(),
            std::inserter(nodeIds, begin(nodeIds)), [](const auto& e) {
              const auto& edge = Edge::getEdge(e);
              return edge.getSourceId();
            });

        auto idToPos = g.getNodePosByIds(nodeIds);
        std::vector<const Node*> nodeLevels;
        std::transform(nRoute->edges.begin(), nRoute->edges.end(), std::back_inserter(nodeLevels),
            [&idToPos](const auto& e) {
              const auto edge = Edge::getEdge(e);
              return idToPos[edge.getSourceId()];
            });

        size_t pos = 1;
        while (pos < nodeLevels.size()) {
          std::cout << "Trying pos " << pos << "\n";
          const Node* start = nullptr;
          start = nodeLevels[pos];

          if (start) {
            auto nextToLastPos = g.getNodePos(start);
            auto currentPos = to;
            const auto dTest = d.findBestRoute(nextToLastPos, currentPos, c);
            const auto nTest = n.findBestRoute(nextToLastPos, currentPos, c);
            if (!(dTest->costs * c <= nTest->costs * c)) {
              std::cout << "start id: " << start->id() << "\n";
              std::cout << "did not find correct subpath between " << nextToLastPos << " and "
                        << currentPos << " at index " << i << '\n';

              std::cout << '\n'
                        << "Normal dijkstra needs length " << nTest->costs.values[0] << " height "
                        << nTest->costs.values[1] << " road " << nTest->costs.values[2] << '\n';

              std::cout << "CH dijkstra needs     length " << dTest->costs.values[0] << " height "
                        << dTest->costs.values[1] << " road " << dTest->costs.values[2] << '\n';

              pos++;
            } else {
              std::cout << "equal routes"
                        << "\n";
              return 1;
            }
          }
        }
      }
    } else if (nRoute && !dRoute) {
      std::cout << "Only Normal dijkstra found route form " << from << " to " << to << "!" << '\n';
      return 1;
    } else {
      ++noRoute;
    }
    std::cout << '+';
    std::cout.flush();
  }

  std::cout << '\n';
  std::cout << "Compared " << route << " routes" << '\n';
  std::cout << "Did not find a route in " << noRoute << " cases" << '\n';
  std::cout << "average speed up is " << static_cast<double>(nTime) / dTime << '\n';
  return 0;
}

namespace po = boost::program_options;
int main(int argc, char* argv[])
{
  std::cout.imbue(std::locale(""));

  std::string loadFileName{};
  std::string saveFileName{};
  double contractionPercent;
  size_t maxThreads = std::thread::hardware_concurrency();

  po::options_description loading{ "loading options" };
  loading.add_options()(
      "text,t", po::value<std::string>(&loadFileName), "Load graph from text file")(
      "multi,m", po::value<std::string>(&loadFileName), "Load graph from multiple files");

  po::options_description contraction{ "contraction options" };
  contraction.add_options()("percent,p", po::value<double>(&contractionPercent)->default_value(98),
      "How far the graph should be contracted");
  contraction.add_options()("stats", "Print statistics while contracting");
  contraction.add_options()("threads", po::value(&maxThreads), "Maximal number of threads used");

  po::options_description saving{ "saving" };
  saving.add_options()("write,w", po::value<std::string>(&saveFileName), "File to save graph to");

  po::options_description all;
  all.add_options()("help,h", "Prints help message");
  all.add(loading).add(contraction).add(saving);

  po::variables_map vm{};
  po::store(po::parse_command_line(argc, argv, all), vm);
  po::notify(vm);

  if (vm.count("help") > 0) {
    std::cout << all << '\n';
    return 0;
  }
  Graph g{ std::vector<Node>(), std::vector<Edge>() };
  if (vm.count("text") > 0) {
    g = loadGraphFromTextFile(loadFileName);
  } else if (vm.count("multi") > 0) {
    g = readMultiFileGraph(loadFileName);
  } else {
    std::cout << "No input file given" << '\n';
    std::cout << all << '\n';
    return 0;
  }

  bool printStats = vm.count("stats") > 0;
  std::cout << "Start contracting" << '\n';
  g = contractGraph(g, 100 - contractionPercent, printStats, maxThreads);

  if (vm.count("write") > 0) {
    std::cout << "saving" << '\n';
    std::ofstream outFile{ saveFileName };
    outFile << "# Graph created at: " << return_current_time_and_date() << '\n';
    outFile << "# Contracted to: " << contractionPercent << "%" << '\n';
    outFile << "# Input Graphfile: " << loadFileName << '\n';
    outFile << '\n';

    g.writeToStream(outFile);
  }
  testGraph(g);
}
