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
#ifndef CONTRACTIONLP_H
#define CONTRACTIONLP_H

#include "child.hpp"
#include "graph.hpp"
#include "io.hpp"
#include <boost/dll.hpp>
#include <iomanip>
#include <sstream>

const auto dir = boost::dll::program_location().parent_path();
const auto lp_executable = "multi_lp";

namespace bp = boost::process;
class ContractionLp {

  std::vector<double> variableValues_;
  double epsilon_ = -1;

  public:
  ContractionLp()
      : lp(dir / lp_executable, std::to_string(Cost::dim), bp::std_out > lpOutput,
          bp::std_in < lpInput)
  {
    variableValues_.reserve(Cost::dim);
  };
  ContractionLp(const ContractionLp& other) = delete;
  ContractionLp(ContractionLp&& other)
  {
    lpOutput = std::move(other.lpOutput);
    lpInput = std::move(other.lpInput);
    lp = std::move(other.lp);
    lpResult = std::move(other.lpResult);
  }
  virtual ~ContractionLp() noexcept = default;
  ContractionLp& operator=(const ContractionLp& other) = delete;
  ContractionLp& operator=(ContractionLp&& other) = default;

  void addConstraint(const std::array<double, Cost::dim>& coeff)
  {
    for (size_t i = 0; i < Cost::dim; ++i) {
      std::stringstream ss;
      ss << std::fixed << std::setprecision(std::numeric_limits<long double>::digits10 + 1)
         << coeff[i];
      lpInput << ss.str() << ' ';
    }
    lpInput << '\n';
  }
  bool solve()
  {
    variableValues_.clear();
    epsilon_ = -1;

    lpInput << '\n';
    lpInput.flush();
    lpOutput >> lpResult;
    if (lpResult == "Infeasible") {
      return false;
    }

    variableValues_.push_back(std::stod(lpResult));
    double coeff = 0;
    for (size_t i = 1; i < Cost::dim; ++i) {
      lpOutput >> coeff;
      variableValues_.push_back(coeff);
    }
    lpOutput >> epsilon_;

    return true;
  }
  std::vector<double> variableValues() const { return variableValues_; }

  double epsilon() const { return epsilon_; }

  protected:
  private:
  bp::ipstream lpOutput;
  bp::opstream lpInput;
  bp::child lp;
  std::string lpResult;
};

#endif /* CONTRACTIONLP_H */
