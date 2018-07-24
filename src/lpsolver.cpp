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
#include "glpk.h"
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

void addVariables(glp_prob* lp, const int varCount)
{
  glp_add_cols(lp, varCount);
  for (int i = 0; i < varCount; ++i) {
    glp_set_col_bnds(lp, i + 1, GLP_DB, 0, 1);
    glp_set_col_kind(lp, i + 1, GLP_CV);
    glp_set_obj_coef(lp, i + 1, 1);
  }
}

void addSumEqOneConstraint(glp_prob* lp, const int varCount)
{
  int row = glp_add_rows(lp, 1);

  std::vector<int> ind(varCount + 1, 0);
  std::vector<double> val(varCount + 1, 1);

  for (int i = 0; i < varCount + 1; ++i) {
    ind[i] = i;
  }
  glp_set_row_bnds(lp, row, GLP_FX, 1, 1);
  glp_set_mat_row(lp, row, varCount, &ind[0], &val[0]);
}

bool readConstraints(glp_prob* lp, const size_t varCount)
{
  while (true) {
    std::string line;
    std::getline(std::cin, line);
    std::stringstream test(line);

    std::string stringValue;
    std::vector<std::string> stringValues;

    while (std::getline(test, stringValue, ' ')) {
      stringValues.push_back(stringValue);
    }
    if (stringValues.size() == 0) {
      break;
    }
    if (stringValues.size() != varCount) {
      std::cerr << "Incorrect number of coefficients " << stringValues.size() << "\n";
      return false;
    }

    int row = glp_add_rows(lp, 1);
    std::vector<int> ind(1, 0);
    std::vector<double> val(1, 0);
    int i = 1;
    for (const auto& s : stringValues) {
      ind.push_back(i++);
      val.push_back(std::stod(s));
    }
    glp_set_row_bnds(lp, row, GLP_LO, 0, 0);
    glp_set_mat_row(lp, row, varCount, &ind[0], &val[0]);
  }
  return true;
}

bool active = true;

int main(int /*argc*/, char* argv[])
{
  const size_t varCount = std::stoull(argv[1]);

  while (active) {
    std::unique_ptr<glp_prob, void (*)(glp_prob*)> lp(glp_create_prob(), glp_delete_prob);
    glp_set_obj_dir(lp.get(), GLP_MIN);

    addVariables(lp.get(), varCount);

    addSumEqOneConstraint(lp.get(), varCount);

    if (!readConstraints(lp.get(), varCount)) {
      return 1;
    }

    glp_smcp params;
    glp_init_smcp(&params);
    params.presolve = GLP_ON;
    params.msg_lev = GLP_MSG_OFF;
    int status = glp_simplex(lp.get(), &params);
    if (status == 0) {
      status = glp_get_status(lp.get());
      if (!(status == GLP_OPT || status == GLP_FEAS)) {
        std::cout << "Infeasible" << '\n';
        continue;
      }
    } else {
      std::cout << "Infeasible" << '\n';
      continue;
    }

    for (size_t i = 1; i < varCount + 1; ++i) {
      std::cout << glp_get_col_prim(lp.get(), i) << "\n";
    }

    if (!std::cin) {
      active = false;
    }
  }

  return 0;
}
