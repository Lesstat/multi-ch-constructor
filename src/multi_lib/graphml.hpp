

#ifndef GRAPHML_H
#define GRAPHML_H

#include "graph.hpp"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/graphml.hpp>
#include <boost/property_map/property_map.hpp>

#include <iostream>
#include <utility>

template <class Graph> class dynamic_property_store {
  public:
  using vert_id = typename boost::graph_traits<Graph>::vertex_descriptor;
  using edge_id = typename boost::graph_traits<Graph>::edge_descriptor;

  using vert_info_map = std::map<vert_id, std::string>;
  using vert_bool_map = std::map<vert_id, bool>;
  using vert_cost_map = std::map<vert_id, double>;
  using vert_level_map = std::map<vert_id, size_t>;

  using edge_info_map = std::map<edge_id, std::string>;
  using edge_bool_map = std::map<edge_id, bool>;
  using edge_cost_map = std::map<edge_id, double>;

  std::map<std::string, vert_cost_map> vert_cost_maps;
  std::map<std::string, vert_bool_map> vert_bool_maps;
  std::map<std::string, vert_info_map> vert_info_maps;
  std::map<std::string, vert_level_map> vert_level_maps;

  std::map<std::string, edge_cost_map> edge_cost_maps;
  std::map<std::string, edge_bool_map> edge_bool_maps;
  std::map<std::string, edge_info_map> edge_info_maps;

  boost::shared_ptr<boost::dynamic_property_map> operator()(
      const std::string& key, const boost::any& descriptor, const boost::any& value)
  {

    boost::shared_ptr<boost::dynamic_property_map> map;

    if (descriptor.type() == typeid(vert_id)) {

      if (value.type() == typeid(std::string)) {
        map = create_map(key, vert_info_maps);
      } else if (value.type() == typeid(double)) {
        map = create_map(key, vert_cost_maps);
      } else if (value.type() == typeid(bool)) {
        map = create_map(key, vert_bool_maps);
      } else if (value.type() == typeid(size_t)) {
        map = create_map(key, vert_level_maps);
      } else {
        std::cout << "Upps no map found for vert_index and " << value.type().name() << '\n';
        std::exit(3);
      }

    } else if (descriptor.type() == typeid(edge_id)) {
      if (value.type() == typeid(std::string)) {
        map = create_map(key, edge_info_maps);
      } else if (value.type() == typeid(double)) {
        map = create_map(key, edge_cost_maps);
      } else if (value.type() == typeid(bool)) {
        map = create_map(key, edge_bool_maps);
      } else {
        std::cout << "Upps no map found for edge_index and " << value.type().name() << '\n';
        std::exit(3);
      }
    } else {
      std::cout << "Key is neither vertex nor edge descriptor: " << descriptor.type().name()
                << '\n';
      std::exit(4);
    }

    return map;
  }

  template <class Map>
  boost::shared_ptr<boost::dynamic_property_map> create_map(const std::string& key, Map& m)
  {
    boost::associative_property_map<typename Map::mapped_type> associative_map(m[key]);
    return boost::shared_ptr<boost::dynamic_property_map>(
        new boost::detail::dynamic_property_map_adaptor(associative_map));
  }
};

std::optional<boost::dynamic_properties> graph_properties;
dynamic_property_store<boost::adjacency_list<>> store;

boost::adjacency_list<> boost_graph;

Graph read_graphml(const std::string& loadFileName)
{

  std::ifstream graph_stream(loadFileName);

  auto& graph_properties = get_graph_properties();

  boost::read_graphml(graph_stream, boost_graph, graph_properties);

  auto vertices = boost::vertices(boost_graph);

  std::vector<Node> nodes;
  std::vector<Edge> edges;
  auto self_loops = 0;
  for (auto vert = vertices.first; vert != vertices.second; ++vert) {
    std::string id = boost::get("name", graph_properties, *vert);
    nodes.emplace_back(id, NodeId { *vert });

    auto out_edges = boost::out_edges(*vert, boost_graph);
    for (auto edge = out_edges.first; edge != out_edges.second; ++edge) {
      if (edge->m_source != *vert) {
        std::cout << "edge does not source != vert id (" << edge->m_source << " != " << *vert << ")"
                  << '\n';
        std::exit(5);
      }
      if (edge->m_source == edge->m_target) {
        self_loops++;
        continue;
      }
      auto& my_edge = edges.emplace_back(NodeId { edge->m_source }, NodeId { edge->m_target });

      my_edge.set_external_id(store.edge_info_maps["name"][*edge]);

      Cost c;
      size_t idx = 0;
      for (auto it = store.edge_cost_maps.begin(); it != store.edge_cost_maps.end(); ++it) {
        if (idx >= Cost::dim) {
          std::cerr << "Graph file has more than " << Cost::dim << " metrics." << '\n';
          std::cerr << "Please recompile code for correct amount of metrics" << '\n';
          std::exit(2);
        }
        c.values[idx] = it->second[*edge];

        if (0 > c.values[idx]) {
          std::cout << " found cost below zero: " << c.values[idx] << "." << '\n';
          std::cout << "exiting because of invalid graph." << '\n';
          std::exit(7);
        }
        ++idx;
      }

      if (idx < Cost::dim) {
        std::cout << "Graph file has not enough metrics." << '\n';
        std::cout << "Found " << idx << " metrics but need " << Cost::dim << "." << '\n';
        std::exit(2);
      }
      my_edge.setCost(c);
    }
  }

  std::cout << "Ignored " << self_loops << " self loop edges" << '\n';

  Graph g { std::move(nodes), std::move(edges) };

  return g;
}

boost::dynamic_properties& get_graph_properties()
{
  if (!graph_properties) {
    graph_properties = boost::dynamic_properties(
        [](const std::string& key, const boost::any& descriptor, const boost::any& value) {
          return store(key, descriptor, value);
        });
  }

  return *graph_properties;
}

void write_graphml(std::ostream& outfile, const Graph& g)
{

  auto& graph_properties = get_graph_properties();

  bool has_edges = false;
  auto [begin, end] = edges(boost_graph);

  for (auto& e = begin; e != end; ++e) {
    has_edges = true;
    std::string no_replace = "-1";
    put("edgeA", graph_properties, *e, no_replace);
    put("edgeB", graph_properties, *e, no_replace);
  }

  for (size_t i = 0; i < g.getNodeCount(); ++i) {
    NodePos pos { i };
    const auto& n = g.getNode(pos);
    put("level", graph_properties, n.id().get(), n.getLevel());

    for (const auto& e : g.getOutgoingEdgesOf(pos)) {

      const auto& edge = Edge::getEdge(e.id);
      // properties speichern
      if (!has_edges || edge.getEdgeA()) {
        const auto& start_node = g.getNode(e.begin);
        const auto& dest_node = g.getNode(e.end);

        auto [descriptor, inserted]
            = boost::add_edge(start_node.id().get(), dest_node.id().get(), boost_graph);

        const auto& edgeA = Edge::getEdge(*edge.getEdgeA());
        const auto& edgeB = Edge::getEdge(*edge.getEdgeB());

        put("edgeA", graph_properties, descriptor, edgeA.external_id());
        put("edgeB", graph_properties, descriptor, edgeB.external_id());
        put("name", graph_properties, descriptor, edge.external_id());
        size_t idx = 0;
        for (auto it = store.edge_cost_maps.begin(); it != store.edge_cost_maps.end(); ++it) {
          it->second[descriptor] = e.cost.values[idx];
          ++idx;
        }
      }
    }
  }

  boost::write_graphml(outfile, boost_graph, graph_properties);
}

#endif /* GRAPHML_H */
