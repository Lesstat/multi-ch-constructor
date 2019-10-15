

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

  public:
  boost::shared_ptr<boost::dynamic_property_map> operator()(
      const std::string& key, const boost::any& any1, const boost::any& any2)
  {
    boost::shared_ptr<boost::dynamic_property_map> map;

    if (any1.type() == typeid(vert_id)) {

      if (any2.type() == typeid(std::string)) {

        boost::associative_property_map<vert_info_map> ass_map(vert_info_maps[key]);
        map = boost::shared_ptr<boost::dynamic_property_map>(
            new boost::detail::dynamic_property_map_adaptor(ass_map));
      } else if (any2.type() == typeid(double)) {

        boost::associative_property_map<vert_cost_map> ass_map(vert_cost_maps[key]);
        map = boost::shared_ptr<boost::dynamic_property_map>(
            new boost::detail::dynamic_property_map_adaptor(ass_map));
      } else if (any2.type() == typeid(bool)) {

        boost::associative_property_map<vert_bool_map> ass_map(vert_bool_maps[key]);
        map = boost::shared_ptr<boost::dynamic_property_map>(
            new boost::detail::dynamic_property_map_adaptor(ass_map));
      }
    } else if (any1.type() == typeid(edge_id)) {

      if (any2.type() == typeid(std::string)) {

        boost::associative_property_map<edge_info_map> ass_map(edge_info_maps[key]);
        map = boost::shared_ptr<boost::dynamic_property_map>(
            new boost::detail::dynamic_property_map_adaptor(ass_map));
      } else if (any2.type() == typeid(double)) {

        boost::associative_property_map<edge_cost_map> ass_map(edge_cost_maps[key]);
        map = boost::shared_ptr<boost::dynamic_property_map>(
            new boost::detail::dynamic_property_map_adaptor(ass_map));
      } else if (any2.type() == typeid(bool)) {

        boost::associative_property_map<edge_bool_map> ass_map(edge_bool_maps[key]);
        map = boost::shared_ptr<boost::dynamic_property_map>(
            new boost::detail::dynamic_property_map_adaptor(ass_map));
      }
    }

    return map;
  }
};

Graph read_graphml(const std::string& loadFileName)
{

  std::ifstream graph_stream(loadFileName);

  boost::adjacency_list<> boost_graph;

  dynamic_property_store<decltype(boost_graph)> store;
  boost::dynamic_properties graph_properties(store);

  boost::read_graphml(graph_stream, boost_graph, graph_properties);

  Graph g { std::vector<Node>(), std::vector<Edge>() };

  return g;
};
#endif /* GRAPHML_H */
