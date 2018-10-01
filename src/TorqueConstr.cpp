// This file is part of PG.
//
// PG is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// PG is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with PG.  If not, see <http://www.gnu.org/licenses/>.

#include "TorqueConstr.h"
//#include "AutoDiffFunction.h"

// RBDyn
#include <RBDyn/ID.h>

std::string testCode()
{
    return "testCode";
}

namespace pg
{

TorqueConstr::TorqueConstr(PGData* pgdata)
    : roboptim::SparseFunction(pgdata->pbSize(),
                         pgdata->multibody().nrDof() - pgdata->multibody().joint(0).dof(),
                         "Torque")
    , pgdata_(pgdata)
{

}

TorqueConstr::~TorqueConstr()
{}

void TorqueConstr::impl_compute(result_ref res, const_argument_ref x) const
{
  pgdata_->x(x);
  rbd::MultiBody mb(pgdata_->mb());
  rbd::MultiBodyConfig mbc(pgdata_->mbc());
  rbd::InverseDynamics id;
  id.inverseDynamics(mb, mbc);
  Eigen::VectorXd torque_vec = rbd::dofToVector(mb, mbc.jointTorque);
  res = torque_vec.segment(mb.joint(0).dof(), torque_vec.size());

  //long unsigned int pos = 0;
  //for (unsigned int i = 1; i < mbc.jointTorque.size(); i++)
  //{
  //  auto torque_i = mbc.jointTorque[i];
  //  auto size = torque_i.size();
  //  result_ad_t res_i(size);
  //  for (unsigned int j = 0; j < size; j++) res_i[j] = torque_i[j];
  //  res.segment(pos, size) = res_i;
  //  pos+=size;
  //}
  // @todo Limits of torque of root is set as -/+inf?
}

} // namespace pg
