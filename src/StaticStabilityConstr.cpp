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

// associated header
#include "StaticStabilityConstr.h"

// include
// PG
#include "PGData.h"
#include "FillSparse.h"


namespace pg
{


StaticStabilityConstr::StaticStabilityConstr(PGData* pgdata)
  : roboptim::DifferentiableSparseFunction(pgdata->pbSize(), 6, "StaticStability")
  , pgdata_(pgdata)
  , gravityForce_(-pgdata->robotMass()*pgdata->gravity())
  , com_()
  , comJac_(pgdata->mb())
  , xStamp_(0)
  , jacPoints_(pgdata->nrForcePoints())
  , jacFullMat_(3, pgdata->mb().nrParams())
  , T_com_fi_jac_(3, pgdata->mb().nrParams())
  , couple_jac_(3, pgdata->mb().nrParams())
{
  std::size_t index = 0;
  for(const PGData::ForceData& fd: pgdata_->forceDatas())
  {
    for(std::size_t i = 0; i < fd.forces.size(); ++i)
    {
      jacPoints_[index] = rbd::Jacobian(pgdata_->mb(), fd.bodyId, fd.points[i].translation());
      ++index;
    }
  }
}


StaticStabilityConstr::~StaticStabilityConstr()
{ }


void StaticStabilityConstr::impl_compute(result_ref res, const_argument_ref x) const
{
  if(xStamp_ != pgdata_->x(x))
  {
    computeCoM();
  }

  res.head<3>().setZero();
  res.tail<3>() = gravityForce_;
  for(const PGData::ForceData& fd: pgdata_->forceDatas())
  {
    const sva::PTransformd& X_0_b = pgdata_->mbc().bodyPosW[fd.bodyIndex];
    for(std::size_t i = 0; i < fd.forces.size(); ++i)
    {
      sva::PTransformd X_0_pi = fd.points[i]*X_0_b;
      Eigen::Vector3d T_com_fi(X_0_pi.translation() - com_);
      Eigen::Vector3d fi_world(fd.forces[i].force());
      res.head<3>() += T_com_fi.cross(fi_world);
      res.tail<3>() += fi_world;
    }
  }
}


void StaticStabilityConstr::impl_jacobian(jacobian_ref jac, const_argument_ref x) const
{
  if(xStamp_ != pgdata_->x(x))
  {
    computeCoM();
  }

  couple_jac_.setZero();
  const Eigen::MatrixXd& comJacMat = comJac_.jacobian(pgdata_->mb(), pgdata_->mbc());

  int index = 0;
  for(const PGData::ForceData& fd: pgdata_->forceDatas())
  {
    const sva::PTransformd& X_0_b = pgdata_->mbc().bodyPosW[fd.bodyIndex];
    for(std::size_t i = 0; i < fd.forces.size(); ++i)
    {
      sva::PTransformd X_0_pi = fd.points[i]*X_0_b;
      Eigen::Vector3d T_com_fi(X_0_pi.translation() - com_);

      // kinematic jacobian
      const Eigen::MatrixXd& jacP = jacPoints_[index].jacobian(pgdata_->mb(),
                                                                pgdata_->mbc());

      // couple
      jacPoints_[index].fullJacobian(pgdata_->mb(), jacP.block(3, 0, 3, jacP.cols()), jacFullMat_);
      T_com_fi_jac_.noalias() = jacFullMat_ - comJacMat;
      couple_jac_.noalias() += sva::vector3ToCrossMatrix((-fd.forces[i].force()).eval())*T_com_fi_jac_;

      // force
      // Zero


      // force jacobian
      // couple
      int indexCols = (pgdata_->forceParamsBegin() + index*3);
      Eigen::Matrix3d couple_force_jac = sva::vector3ToCrossMatrix(T_com_fi);
      fillSparse(couple_force_jac, jac, {0, indexCols});
      // force
      fillSparse(Eigen::Matrix3d::Identity(), jac, {3, indexCols});

      ++index;
    }
  }
  // fill couple
  fillSparse(couple_jac_, jac, {0, pgdata_->qParamsBegin()});
}


void StaticStabilityConstr::computeCoM() const
{
  xStamp_ = pgdata_->xStamp();
  com_ = rbd::computeCoM(pgdata_->mb(), pgdata_->mbc());
}


} // pg
