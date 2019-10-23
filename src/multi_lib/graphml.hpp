

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

  using edge_info_map = std::map<edge_id, std::string>;
  using edge_bool_map = std::map<edge_id, bool>;
  using edge_cost_map = std::map<edge_id, double>;

  std::map<std::string, vert_cost_map> vert_cost_maps;
  std::map<std::string, vert_bool_map> vert_bool_maps;
  std::map<std::string, vert_info_map> vert_info_maps;

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
      }

    } else if (descriptor.type() == typeid(edge_id)) {
      if (value.type() == typeid(std::string)) {
        map = create_map(key, edge_info_maps);
      } else if (value.type() == typeid(double)) {
        map = create_map(key, edge_cost_maps);
      } else if (value.type() == typeid(bool)) {
        map = create_map(key, edge_bool_maps);
      }
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

Graph read_graphml(const std::string& loadFileName)
{

  std::ifstream graph_stream(loadFileName);

  boost::adjacency_list<> boost_graph;

  dynamic_property_store<decltype(boost_graph)> store;
  boost::dynamic_properties graph_properties(
      [&store](const std::string& key, const boost::any& descriptor, const boost::any& value) {
        return store(key, descriptor, value);
      });

  boost::read_graphml(graph_stream, boost_graph, graph_properties);

  auto vertices = boost::vertices(boost_graph);

  std::vector<Node> nodes;
  std::vector<Edge> edges;
  for (auto vert = vertices.first; vert != vertices.second; ++vert) {
    std::string id = boost::get("id", graph_properties, *vert);
    nodes.emplace_back(id, NodeId { *vert });

    auto out_edges = boost::out_edges(*vert, boost_graph);
    for (auto edge = out_edges.first; edge != out_edges.second; ++edge) {
      auto& my_edge = edges.emplace_back(NodeId { edge->m_source }, NodeId { edge->m_target });

      my_edge.set_extrenal_id(store.edge_info_maps["name"][*edge]);

      Cost c;
      size_t idx = 0;
      for (auto it = store.edge_cost_maps.begin(); it != store.edge_cost_maps.end(); ++it) {
        c.values[idx] = it->second[*edge];
        ++idx;
      }
      my_edge.setCost(c);
    }
  }

  Graph g { std::move(nodes), std::move(edges) };

  return g;
};
#endif /* GRAPHML_H */
