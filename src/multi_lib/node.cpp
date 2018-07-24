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

Node::Node(NodeId id, size_t osmId, Lat lat, Lng lng, double height)
    : id_(id)
    , osmId(osmId)
    , lat_(lat)
    , lng_(lng)
    , level(0)
    , height_(height)
{
}

size_t Node::getLevel() const { return level; }

void Node::assignLevel(size_t level) { this->level = level; }

NodeId Node::id() const { return id_; }

Lat Node::lat() const { return lat_; }
Lng Node::lng() const { return lng_; }
short Node::height() const { return height_; }

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

  Node n{ NodeId{ id }, osmId, Lat(lat), Lng(lng), height };
  n.level = level;
  return n;
}

void Node::writeToStream(std::ostream& out) const
{
  out << id_ << ' ' << osmId << ' ' << lat_ << ' ' << lng_ << ' ' << height_ << ' ' << level
      << '\n';
}
