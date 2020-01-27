
#include "contractionLP.hpp"
#include "contractor.hpp"
#include "graph.hpp"
#include "graphml.hpp"
#include "multiqueue.hpp"

#include "catch.hpp"

#include <set>
#include <string>

std::vector<Edge> buildGraphAndContractNode(std::stringstream& graph_file, size_t node_id)
{
  auto g = Graph::createFromStream(graph_file);
  Contractor c(false);

  MultiQueue<EdgePair> q;
  ContractionLp lp;
  const std::set<NodePos> set;

  auto future = c.contract(q, g, &lp, set);

  for (auto& e_in : g.getIngoingEdgesOf(NodePos { node_id })) {
    for (auto& e_out : g.getOutgoingEdgesOf(NodePos { node_id })) {
      q.send({ e_in, e_out });
    }
  }

  q.close();

  return future.get();
}

TEST_CASE("Three node Line graph")
{
  std::string graph_txt = R"file(# Some comment 

2
3
2
0 1 48.1 9.2 0 0
1 2 48.1 9.2 0 0
2 3 48.1 9.2 0 0
0 1 1.5 2 -1 -1
1 2 1.5 2 -1 -1)file";

  std::stringstream graph_file(graph_txt);

  auto result = buildGraphAndContractNode(graph_file, 1);

  REQUIRE(result.size() == 1);

  auto& edge = result[0];

  REQUIRE(edge.getSourceId() == NodePos { 0 });
  REQUIRE(edge.getDestId() == NodePos { 2 });

  const Cost& cost = edge.getCost();
  REQUIRE(cost.values[0] == 3.0);
  REQUIRE(cost.values[1] == 4.0);
}

TEST_CASE("Three node Line graph with additional paths")
{
  std::string graph_txt = R"file(# Some comment 

2
3
4
0 1 48.1 9.2 0 0
1 2 48.1 9.2 0 0
2 3 48.1 9.2 0 0
0 1 1.5 2 -1 -1
1 2 1.5 2 -1 -1
0 2 0.1 100 -1 -1
0 2 100 0.1 -1 -1
)file";

  std::stringstream graph_file(graph_txt);

  auto result = buildGraphAndContractNode(graph_file, 1);

  REQUIRE(result.size() == 1);

  auto& edge = result[0];

  REQUIRE(edge.getSourceId() == NodePos { 0 });
  REQUIRE(edge.getDestId() == NodePos { 2 });

  const Cost& cost = edge.getCost();
  REQUIRE(cost.values[0] == 3.0);
  REQUIRE(cost.values[1] == 4.0);
}

TEST_CASE("Three node Line graph with same cost path")
{
  std::string graph_txt = R"file(# Some comment 

2
3
5
0 1 48.1 9.2 0 0
1 2 48.1 9.2 0 0
2 3 48.1 9.2 0 0
0 1 1.5 2 -1 -1
1 2 1.5 2 -1 -1
0 2 0.1 100 -1 -1
0 2 100 0.1 -1 -1
0 2 3 4 -1 -1
)file";

  std::stringstream graph_file(graph_txt);

  auto result = buildGraphAndContractNode(graph_file, 1);

  REQUIRE(result.empty());
}

TEST_CASE("Three node Line graph where LP is needed")
{
  std::string graph_txt = R"file(# Some comment 

2
3
5
0 1 48.1 9.2 0 0
1 2 48.1 9.2 0 0
2 3 48.1 9.2 0 0
0 1 1.5 2 -1 -1
1 2 1.5 2 -1 -1
0 2 0.1 100 -1 -1
0 2 100 0.1 -1 -1
0 2 2 4.2 -1 -1
)file";

  std::stringstream graph_file(graph_txt);

  auto result = buildGraphAndContractNode(graph_file, 1);

  REQUIRE(result.size() == 1);

  auto& edge = result[0];

  REQUIRE(edge.getSourceId() == NodePos { 0 });
  REQUIRE(edge.getDestId() == NodePos { 2 });

  const Cost& cost = edge.getCost();
  REQUIRE(cost.values[0] == 3.0);
  REQUIRE(cost.values[1] == 4.0);
}

TEST_CASE("Three node Line graph where LP is needed with same cost paths")
{
  std::string graph_txt = R"file(# Some comment 

2
3
6
0 1 48.1 9.2 0 0
1 2 48.1 9.2 0 0
2 3 48.1 9.2 0 0
0 1 1.5 2 -1 -1
1 2 1.5 2 -1 -1
0 2 0.1 100 -1 -1
0 2 100 0.1 -1 -1
0 2 2 4.2 -1 -1
0 2 3 4 -1 -1
)file";

  std::stringstream graph_file(graph_txt);

  auto result = buildGraphAndContractNode(graph_file, 1);

  REQUIRE(result.empty());
}
