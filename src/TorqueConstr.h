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

#pragma once

// include
// roboptim
//#include <roboptim/core/differentiable-function.hh>
#include <roboptim/core/function.hh>

// Eigen
#include <unsupported/Eigen/Polynomials>

// PG
#include "AutoDiffFunction.h"
#include "PGData.h"


namespace pg
{

template<typename Type>
class TorqueConstr_autodiff : public AutoDiffFunction<Type, Eigen::Dynamic>
{
public:
  typedef AutoDiffFunction<Type, Eigen::Dynamic> parent_t;
  typedef typename parent_t::scalar_t scalar_t;
  typedef typename parent_t::result_ad_t result_ad_t;
  typedef typename parent_t::argument_t argument_t;

public:
  TorqueConstr_autodiff(PGData* pgdata)
    : parent_t(pgdata, pgdata->pbSize(),
               pgdata->multibody().nrDof() - pgdata->multibody().joint(0).dof(),
               "Torque_ad")
    , pgdata_(pgdata)
  {}
  ~TorqueConstr_autodiff()
  { }


  void impl_compute(result_ad_t& res, const argument_t& x) const
  {
  }

private:
  PGData* pgdata_;
};

class TorqueConstr : public roboptim::Function
{
public:
  //typedef typename parent_t::argument_t argument_t;

public:
  TorqueConstr(PGData* pgdata);
  ~TorqueConstr();


  void impl_compute(result_ref res, const_argument_ref x) const;
  //void impl_jacobian(jacobian_ref /* jac */, const_argument_ref /* x */) const;
  //void impl_jacobian(jacobian_ref jac, const_argument_ref x) const;
  //void impl_gradient(gradient_ref /* gradient */,
  //    const_argument_ref /* x */, size_type /* functionId */) const
  //{
  //  throw std::runtime_error("NEVER GO HERE");
  //}

private:
  PGData* pgdata_;

};


//template<typename Type>
//class TorquePolyBoundsConstr : public AutoDiffFunction<Type, Eigen::Dynamic>
//{
//public:
//  typedef AutoDiffFunction<Type, Eigen::Dynamic> parent_t;
//  typedef typename parent_t::scalar_t scalar_t;
//  typedef typename parent_t::result_ad_t result_ad_t;
//  typedef typename parent_t::argument_t argument_t;
//
//public:
//  //TorquePolyBoundsConstr(PGData<Type>* pgdata,
//  TorquePolyBoundsConstr(PGData* pgdata,
//                         std::vector<std::vector<Eigen::VectorXd>> tl,
//                         std::vector<std::vector<Eigen::VectorXd>> tu)
//    : parent_t(pgdata, pgdata->pbSize(),
//               (pgdata->multibody().nrDof() - pgdata->multibody().joint(0).dof())*2,
//               "TorquePolyBounds")
//    , pgdata_(pgdata)
//    , tl_(std::move(tl))
//    , tu_(std::move(tu))
//  {}
//  ~TorquePolyBoundsConstr()
//  { }
//
//
//  void impl_compute(result_ad_t& res, const argument_t& /* x */) const
//  {
//    const ID<scalar_t>& id = pgdata_->id();
//    const std::vector<std::vector<scalar_t>>&q = pgdata_->q();
//    int vecPos = 0;
//    int dof = (pgdata_->multibody().nrDof() - pgdata_->multibody().joint(0).dof());
//    for(std::size_t i = 1; i < q.size(); ++i)
//    {
//      for(std::size_t j = 0; j < q[i].size(); ++j)
//      {
//        res(vecPos) = id.torque()[i][j] - Eigen::poly_eval(tl_[i][j], q[i][j]);
//        res(vecPos + dof) = id.torque()[i][j] - Eigen::poly_eval(tu_[i][j], q[i][j]);
//        ++vecPos;
//      }
//    }
//  }
//
//private:
//  //PGData<Type>* pgdata_;
//  PGData* pgdata_;
//  std::vector<std::vector<Eigen::VectorXd>> tl_;
//  std::vector<std::vector<Eigen::VectorXd>> tu_;
//};

} // namespace pg
