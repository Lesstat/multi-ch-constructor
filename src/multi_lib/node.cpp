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
#include "graph.hpp"

Node::Node(const std::string external_id, NodeId id)
    : external_node_id_(external_id)
    , id_(id)
    , level(0)
{
}

size_t Node::getLevel() const { return level; }

void Node::assignLevel(size_t level) { this->level = level; }

NodeId Node::id() const { return id_; }

// Lat Node::lat() const { return lat_; }
// Lng Node::lng() const { return lng_; }
// short Node::height() const { return height_; }

std::ostream& operator<<(std::ostream& os, const Node& n)
{
  os << std::to_string(n.id_);
  return os;
}

Node Node::createFromText(const std::string& text)
{
  size_t id, osmId, level;
  double lat, lng;
  double height;

  std::sscanf(
      text.c_str(), "%lu%lu%lf%lf%lf%lu", &id, &osmId, &lat, &lng, &height, &level); // NOLINT

  Node n { std::to_string(id), NodeId { id } };

  auto& graph_properties = get_graph_properties();

  auto osm_id_string = std::to_string(osmId);
  put("osmId", graph_properties, id, osm_id_string);
  put("lat", graph_properties, id, lat);
  put("lng", graph_properties, id, lng);
  put("height", graph_properties, id, height);

  n.level = level;
  return n;
}

void Node::writeToStream(std::ostream& out) const
{
  const auto& graph_properties = get_graph_properties();
  size_t id = std::stoi(external_node_id_);

  std::string osm_id = "0";
  double lat = 0;
  double lng = 0;
  double height = 0;

  try {
    osm_id = get<std::string>("osmId", graph_properties, id);
    lat = get<double>("lat", graph_properties, id);
    lng = get<double>("lng", graph_properties, id);
    height = get<double>("height", graph_properties, id);

  } catch (boost::wrapexcept<boost::dynamic_get_failure>& e) {
    std::cout << "failed: " << e.what() << '\n';
  }
  out << id_ << ' ' << osm_id << ' ' << lat << ' ' << lng << ' ' << height << ' ' << level << '\n';
}

const std::string& Node::external_id() const { return external_node_id_; }
