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


// include
// std
#include <fstream>
#include <iostream>
#include <tuple>

// boost
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE PG test
#include <boost/test/unit_test.hpp>
#include <boost/math/constants/constants.hpp>

// sch
#include <sch/S_Object/S_Sphere.h>

// RBDyn
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/ID.h>

// PG
#include "ConfigStruct.h"
#include "PostureGenerator.h"
#include "CollisionConstr.h" // tosch

// Arm
#include "Z12Arm.h"


const Eigen::Vector3d gravity(0., 9.81, 0.);

/// @return An simple ZXZ arm with Y as up axis.
std::tuple<rbd::MultiBody, rbd::MultiBodyConfig> makeZXZArm(bool isFixed=true)
{
  using namespace Eigen;
  using namespace sva;
  using namespace rbd;

  MultiBodyGraph mbg;

  double mass = 1.;
  Matrix3d I = Matrix3d::Identity();
  Vector3d h = Vector3d::Zero();

  RBInertiad rbi(mass, h, I);

  Body b0(rbi, "b0");
  Body b1(rbi, "b1");
  Body b2(rbi, "b2");
  Body b3(rbi, "b3");

  mbg.addBody(b0);
  mbg.addBody(b1);
  mbg.addBody(b2);
  mbg.addBody(b3);

  Joint j0(Joint::RevZ, true, "j0");
  Joint j1(Joint::RevX, true, "j1");
  Joint j2(Joint::RevZ, true, "j2");

  mbg.addJoint(j0);
  mbg.addJoint(j1);
  mbg.addJoint(j2);

  //  Root     j0       j1     j2
  //  ---- b0 ---- b1 ---- b2 ----b3
  //  Fixed    Z       X       Z


  PTransformd to(Vector3d(0., 0.5, 0.));
  PTransformd from(Vector3d(0., 0., 0.));


  mbg.linkBodies("b0", PTransformd::Identity(), "b1", from, "j0");
  mbg.linkBodies("b1", to, "b2", from, "j1");
  mbg.linkBodies("b2", to, "b3", from, "j2");

  MultiBody mb = mbg.makeMultiBody("b0", isFixed);

  MultiBodyConfig mbc(mb);
  mbc.zero(mb);

  return std::make_tuple(mb, mbc);
}


BOOST_AUTO_TEST_CASE(PGTest)
{
  using namespace Eigen;
  using namespace sva;
  using namespace rbd;
  namespace cst = boost::math::constants;

  MultiBody mb;
  MultiBodyConfig mbcInit, mbcWork;

  std::tie(mb, mbcInit) = makeZXZArm();
  mbcWork = mbcInit;

  {
    pg::PostureGenerator pgPb;
    pg::RobotConfig rc(mb);
    pgPb.param("ipopt.print_level", 0);

    Vector3d target(0., 0.5, 0.5);
    rc.fixedPosContacts = {{"b3", target, sva::PTransformd::Identity()}};

    pgPb.robotConfigs({rc}, gravity);
    BOOST_REQUIRE(pgPb.run({{mbcInit.q, {}, mbcInit.q}}));

    mbcWork.q = pgPb.q();
    forwardKinematics(mb, mbcWork);
    BOOST_CHECK_SMALL((mbcWork.bodyPosW[3].translation() - target).norm(), 1e-5);
  }

  {
    pg::PostureGenerator pgPb;
    pg::RobotConfig rc(mb);
    pgPb.param("ipopt.print_level", 0);

    Matrix3d target(Quaterniond(AngleAxisd(-cst::pi<double>()/2., Vector3d::UnitX())));
    rc.fixedOriContacts = {{"b3", target, sva::PTransformd::Identity()}};

    pgPb.robotConfigs({rc}, gravity);
    BOOST_REQUIRE(pgPb.run({{mbcInit.q, {}, mbcInit.q}}));

    mbcWork.q = pgPb.q();
    forwardKinematics(mb, mbcWork);
    BOOST_CHECK_SMALL((mbcWork.bodyPosW[3].rotation() - target).norm(), 1e-3);
  }
}


void toPython(const rbd::MultiBody& mb,
              const rbd::MultiBodyConfig& mbc,
              const std::vector<pg::ForceContact>& fc,
              const std::vector<sva::ForceVecd>& forces,
              const std::string& filename)
{
  std::ofstream out(filename);

  out << "pos = [";
  for(std::size_t i = 0; i < mbc.bodyPosW.size(); ++i)
  {
    out << "[" << mbc.bodyPosW[i].translation()[0] << ", "
               << mbc.bodyPosW[i].translation()[1] << ", "
               << mbc.bodyPosW[i].translation()[2] << "], ";
  }
  out << "]" << std::endl;

  out << "forces = [";
  std::size_t findex = 0;
  for(std::size_t i = 0; i < fc.size(); ++i)
  {
    int bodyIndex = mb.bodyIndexByName(fc[i].bodyName);
    for(std::size_t j = 0; j < fc[i].points.size(); ++j)
    {
      Eigen::Vector3d start = (fc[i].points[j]*mbc.bodyPosW[bodyIndex]).translation();
      out << "[";
      out << "(" << start[0] << ", "
                 << start[1] << ", "
                 << start[2] << "), ";
      Eigen::Vector3d end = start + forces[findex].force();
      out << "(" << end[0] << ", "
                 << end[1] << ", "
                 << end[2] << "), ";
      out << "]," << std::endl;
      ++findex;
    }
  }
  out << "]" << std::endl;
}


BOOST_AUTO_TEST_CASE(PGTestZ12)
{
  using namespace Eigen;
  using namespace sva;
  using namespace rbd;
  namespace cst = boost::math::constants;

  MultiBody mb;
  MultiBodyConfig mbcInit, mbcWork;

  std::tie(mb, mbcInit) = makeZ12Arm();
  // to avoid to start in singularity
  mbcInit.q[3][0] = -0.1;
  mbcWork = mbcInit;

  {
    pg::PostureGenerator pgPb;
    pg::RobotConfig rc(mb);
    pgPb.param("ipopt.print_level", 0);
    pgPb.param("ipopt.linear_solver", "mumps");

    Vector3d target(2., 0., 0.);
    Matrix3d oriTarget(sva::RotZ(-cst::pi<double>()));
    std::string id = "b12";
    int index = mb.bodyIndexByName(id);
    rc.fixedPosContacts = {{id, target, sva::PTransformd::Identity()}};
    rc.fixedOriContacts = {{id, oriTarget, sva::PTransformd::Identity()}};

    pgPb.robotConfigs({rc}, gravity);
    BOOST_REQUIRE(pgPb.run({{mbcInit.q, {}, mbcInit.q}}));

    mbcWork.q = pgPb.q();
    forwardKinematics(mb, mbcWork);
    BOOST_CHECK_SMALL((mbcWork.bodyPosW[index].translation() - target).norm(), 1e-5);
    BOOST_CHECK_SMALL((mbcWork.bodyPosW[index].rotation() - oriTarget).norm(), 1e-3);
    toPython(mb, mbcWork, rc.forceContacts, pgPb.forces(),"Z12.py");
  }

  {
    pg::PostureGenerator pgPb;
    pg::RobotConfig rc(mb);
    pgPb.param("ipopt.print_level", 0);
    pgPb.param("ipopt.linear_solver", "mumps");

    Vector3d target(2., 0., 0.);
    Matrix3d oriTarget(sva::RotZ(-cst::pi<double>()));
    std::string id = "b12";
    int index = mb.bodyIndexByName(id);
    rc.fixedPosContacts = {{id, target, sva::PTransformd::Identity()}};
    rc.fixedOriContacts = {{id, oriTarget, sva::PTransformd::Identity()}};
    Matrix3d frame(RotX(-cst::pi<double>()/2.));
    rc.forceContacts = {{"b0", {sva::PTransformd(frame, Vector3d(0.01, 0., 0.)),
                             sva::PTransformd(frame, Vector3d(-0.01, 0., 0.))}, 1.}};

    pgPb.robotConfigs({rc}, gravity);
    BOOST_REQUIRE(pgPb.run({{mbcInit.q, {}, mbcInit.q}}));

    mbcWork.q = pgPb.q();
    forwardKinematics(mb, mbcWork);
    BOOST_CHECK_SMALL((mbcWork.bodyPosW[index].translation() - target).norm(), 1e-5);
    BOOST_CHECK_SMALL((mbcWork.bodyPosW[index].rotation() - oriTarget).norm(), 1e-3);
    toPython(mb, mbcWork, rc.forceContacts, pgPb.forces(),"Z12Stab.py");
  }

  {
    pg::PostureGenerator pgPb;
    pg::RobotConfig rc(mb);
    pgPb.param("ipopt.print_level", 0);
    pgPb.param("ipopt.linear_solver", "mumps");

    Vector3d target(1.5, 0., 0.);
    Matrix3d oriTarget(sva::RotZ(-cst::pi<double>()));
    std::string id = "b12";
    int index = mb.bodyIndexByName(id);
    rc.fixedPosContacts = {{id, target, sva::PTransformd::Identity()}};
    rc.fixedOriContacts = {{id, oriTarget, sva::PTransformd::Identity()}};
    Matrix3d frame(RotX(-cst::pi<double>()/2.));
    Matrix3d frameEnd(RotX(cst::pi<double>()/2.));
    rc.forceContacts = {{"b0", {sva::PTransformd(frame, Vector3d(0.01, 0., 0.)),
                             sva::PTransformd(frame, Vector3d(-0.01, 0., 0.))}, 1.},
                       {id, {sva::PTransformd(frameEnd, Vector3d(0.01, 0., 0.)),
                             sva::PTransformd(frameEnd, Vector3d(-0.01, 0., 0.))}, 1.}};

    rc.postureScale = 1.;
    pgPb.robotConfigs({rc}, gravity);
    BOOST_REQUIRE(pgPb.run({{mbcInit.q, {}, mbcInit.q}}));

    mbcWork.q = pgPb.q();
    forwardKinematics(mb, mbcWork);
    BOOST_CHECK_SMALL((mbcWork.bodyPosW[index].translation() - target).norm(), 1e-5);
    BOOST_CHECK_SMALL((mbcWork.bodyPosW[index].rotation() - oriTarget).norm(), 1e-3);
    toPython(mb, mbcWork, rc.forceContacts, pgPb.forces(),"Z12Stab2.py");
  }

  {
    pg::PostureGenerator pgPb;
    pg::RobotConfig rc(mb);
    pgPb.param("ipopt.print_level", 0);
    pgPb.param("ipopt.linear_solver", "mumps");

    std::string id = "b12";
    int index = mb.bodyIndexByName(id);
    Matrix3d frame(RotX(-cst::pi<double>()/2.));
    sva::PTransformd targetSurface(frame, Vector3d(0., 1., 0.));
    sva::PTransformd bodySurface(frame);
    std::vector<Eigen::Vector2d> targetPoints = {{1., 1.}, {-0., 1.}, {-0., -1.}, {1., -1.}};
    std::vector<Eigen::Vector2d> surfPoints = {{0.1, 0.1}, {-0.1, 0.1}, {-0.1, -0.1}, {0.1, -0.1}};
    rc.planarContacts = {{id, targetSurface, targetPoints, bodySurface, surfPoints}};
    rc.bodyPosTargets = {{id, Vector3d(2., 1., 0.), 10.}};

    pgPb.robotConfigs({rc}, gravity);
    /// @todo couldn't solve under this condition?
    /*
    BOOST_REQUIRE(pgPb.run({{mbcInit.q, {}, mbcInit.q}}));

    mbcWork.q = pgPb.q();
    forwardKinematics(mb, mbcWork);
    sva::PTransformd surfPos = bodySurface*mbcWork.bodyPosW[index];
    double posErr = (surfPos.translation() -
        targetSurface.translation()).dot(targetSurface.rotation().row(2));
    double oriErr = surfPos.rotation().row(2).dot(targetSurface.rotation().row(2)) - 1.;
    BOOST_CHECK_SMALL(posErr, 1e-5);
    BOOST_CHECK_SMALL(oriErr, 1e-5);
    toPython(mb, mbcWork, rc.forceContacts, pgPb.forces(),"Z12Planar.py");
    */
  }

  /*
  //Test for Ellipse Constraints
  {
    pg::PostureGenerator pgPb(mb, gravity);
    pgPb.param("ipopt.print_level", 5);
    pgPb.param("ipopt.linear_solver", "mumps");

    int id = 12;
    int index = mb.bodyIndexByName(id);
    Matrix3d frame(RotX(-cst::pi<double>()/2.));
    sva::PTransformd targetSurface(frame, Vector3d(0., 3., 0.));
    sva::PTransformd bodySurface(frame);
    std::vector<Eigen::Vector2d> targetPoints = {{0.0, 0.0}, {4.0, 0.0}, {4.0, 1.0}, {0.0, 1.0}};
    std::vector<Eigen::Vector2d> surfPoints = {{0.2, -0.3}, {0.4, -0.5}, {1.0, 0.5}, {0.8, 1.7}};
    pgPb.ellipseContacts({{id, double(0.1), targetSurface, targetPoints, bodySurface, surfPoints}});
    pgPb.bodyPositionTargets({{id, Vector3d(2., 1., 0.), 10.}});

    BOOST_REQUIRE(pgPb.run(mbcInit.q, {}, mbcInit.q, 0., 0., 0.));

    mbcWork.q = pgPb.q();
    forwardKinematics(mb, mbcWork);
    sva::PTransformd surfPos = bodySurface*mbcWork.bodyPosW[index];
    double posErr = (surfPos.translation() -
        targetSurface.translation()).dot(targetSurface.rotation().row(2));
    double oriErr = surfPos.rotation().row(2).dot(targetSurface.rotation().row(2)) - 1.;
    BOOST_CHECK_SMALL(posErr, 1e-5);
    BOOST_CHECK_SMALL(oriErr, 1e-5);
    toPython(mb, mbcWork, pgPb.forceContacts(), pgPb.forces(),"Z12Ellipse.py");
  }
  */

  // Test for TorqueConstr
  {
    pg::PostureGenerator pgPb;
    pg::RobotConfig rc(mb);
    pgPb.param("ipopt.print_level", 0);
    pgPb.param("ipopt.linear_solver", "mumps");

    Vector3d target(1.5, 0., 0.);
    Matrix3d oriTarget(sva::RotZ(-cst::pi<double>()));
    std::string id = "b12";
    rc.fixedPosContacts = {{id, target, sva::PTransformd::Identity()}};
    rc.fixedOriContacts = {{id, oriTarget, sva::PTransformd::Identity()}};
    Matrix3d frame(RotX(-cst::pi<double>()/2.));
    Matrix3d frameEnd(RotX(cst::pi<double>()/2.));
    rc.forceContacts = {{"b0", {sva::PTransformd(frame, Vector3d(0.01, 0., 0.)),
                             sva::PTransformd(frame, Vector3d(-0.01, 0., 0.))}, 1.},
                       {id, {sva::PTransformd(frameEnd, Vector3d(0.01, 0., 0.)),
                             sva::PTransformd(frameEnd, Vector3d(-0.01, 0., 0.))}, 1.}};

    std::vector<std::vector<double>> tl(mb.nrJoints());
    std::vector<std::vector<double>> tu(mb.nrJoints());
    for(std::size_t i = 0; i < tl.size(); ++i)
    {
      tl[i].resize(mb.joint(int(i)).dof());
      tu[i].resize(mb.joint(int(i)).dof());
      for(std::size_t j = 0; j < tl[i].size(); ++j)
      {
        tl[i][j] = -100.;
        tu[i][j] = 100.;
      }
    }
    rc.tl = tl;
    rc.tu = tu;

    pgPb.robotConfigs({rc}, gravity);
    BOOST_REQUIRE(pgPb.run({{mbcInit.q, {}, mbcInit.q}}));

    std::vector<sva::ForceVecd> forces = pgPb.forces();
    std::vector<std::vector<double>> torque = pgPb.torque();

    mbcWork.zero(mb);
    mbcWork.q = pgPb.q();
    forwardKinematics(mb, mbcWork);
    forwardVelocity(mb, mbcWork);

    // Input force computed by the pg
    int forceIndex = 0;
    for(const pg::ForceContact& f: rc.forceContacts)
    {
      int index = mb.bodyIndexByName(f.bodyName);
      for(const sva::PTransformd& p: f.points)
      {
        mbcWork.force[index] = mbcWork.force[index] +
            mbcWork.bodyPosW[index].transMul(p.transMul(forces[forceIndex]));
        ++forceIndex;
      }
    }

    // Compute the inverse dynamics
    rbd::InverseDynamics invDyn(mb);
    invDyn.inverseDynamics(mb, mbcWork);

    // check if torque are equals
    for(int i = 0; i < mb.nrJoints(); ++i)
    {
      for(int j = 0; j < mb.joint(i).dof(); ++j)
      {
        /// @todo implement pgPb.torque()
        //BOOST_CHECK_SMALL(mbcWork.jointTorque[i][j] -torque[i][j], 1e-5);
      }
    }

    toPython(mb, mbcWork, rc.forceContacts, pgPb.forces(),"Z12Torque.py");
  }


  /*
   *                      Environment collision avoidance
   */
  {
    pg::PostureGenerator pgPb;
    pg::RobotConfig rc(mb);
    pgPb.param("ipopt.print_level", 0);
    pgPb.param("ipopt.linear_solver", "mumps");

    Vector3d target(0., 0., 0.);
    std::string id = "b12";
    int index = mb.bodyIndexByName(id);
    rc.bodyPosTargets = {{id, target, 0.1}};

    // first we try to go to origin
    pgPb.robotConfigs({rc}, gravity);
    BOOST_REQUIRE(pgPb.run({{mbcInit.q, {}, mbcInit.q}}));

    auto qOrigin = pgPb.q();
    mbcWork.q = qOrigin;
    forwardKinematics(mb, mbcWork);
    BOOST_CHECK_SMALL((mbcWork.bodyPosW[index].translation() - target).norm(), 1e-5);
    toPython(mb, mbcWork, rc.forceContacts, pgPb.forces(),"Z12EnvCol0.py");

    pgPb.param("ipopt.tol", 1e-1);
    pgPb.param("ipopt.dual_inf_tol", 1e-1);
    sch::S_Sphere hullBody(0.5);
    sch::S_Sphere hullEnv(0.5);
    hullEnv.setTransformation(pg::tosch(sva::PTransformd::Identity()));

    rc.envCollisions = {{id, &hullBody, sva::PTransformd::Identity(), &hullEnv, 0.1}};
    // we check that we couldn't go in collision
    pgPb.robotConfigs({rc}, gravity);
    BOOST_REQUIRE(pgPb.run({{mbcInit.q, {}, mbcInit.q}}));

    mbcWork.q = pgPb.q();
    forwardKinematics(mb, mbcWork);
    BOOST_CHECK_GT((mbcWork.bodyPosW[index].translation() - target).norm(), 1. + 0.1);
    toPython(mb, mbcWork, rc.forceContacts, pgPb.forces(),"Z12EnvCol1.py");

    rc.bodyPosTargets = {};
    // same check but we start in constraint violation
    pgPb.robotConfigs({rc}, gravity);
    BOOST_REQUIRE(pgPb.run({{qOrigin, {}, mbcInit.q}}));

    mbcWork.q = pgPb.q();
    forwardKinematics(mb, mbcWork);
    BOOST_CHECK_GT((mbcWork.bodyPosW[index].translation() - target).norm(), 1. + 0.1);
    toPython(mb, mbcWork, rc.forceContacts, pgPb.forces(),"Z12EnvCol2.py");
  }


  /*
   *                      Self collision avoidance
   */
  {
    pg::PostureGenerator pgPb;
    pg::RobotConfig rc(mb);
    pgPb.param("ipopt.print_level", 0);
    pgPb.param("ipopt.linear_solver", "mumps");

    Vector3d target(0., 0., 0.);
    std::string id1 = "b12";
    int index1 = mb.bodyIndexByName(id1);
    std::string id2 = "b6";
    int index2 = mb.bodyIndexByName(id2);
    rc.bodyPosTargets = {{id1, target, 0.1}, {id2, target, 0.1}};

    // first we try to go to origin
    pgPb.robotConfigs({rc}, gravity);
    BOOST_REQUIRE(pgPb.run({{mbcInit.q, {}, mbcInit.q}}));

    auto qOrigin = pgPb.q();
    mbcWork.q = qOrigin;
    forwardKinematics(mb, mbcWork);
    double bodyDist = (mbcWork.bodyPosW[index1].translation() -
                       mbcWork.bodyPosW[index2].translation()).norm();
    BOOST_CHECK_SMALL(bodyDist, 1e-5);
    toPython(mb, mbcWork, rc.forceContacts, pgPb.forces(),"Z12SelfCol0.py");

    pgPb.param("ipopt.tol", 1e-1);
    pgPb.param("ipopt.dual_inf_tol", 1e-1);
    sch::S_Sphere hullBody1(0.5);
    sch::S_Sphere hullBody2(0.5);

    rc.selfCollisions = {{id1, &hullBody1, sva::PTransformd::Identity(),
                          id2, &hullBody2, sva::PTransformd::Identity(),
                          0.1}};

    // we check that we couldn't go in collision
    pgPb.robotConfigs({rc}, gravity);
    BOOST_REQUIRE(pgPb.run({{mbcInit.q, {}, mbcInit.q}}));

    mbcWork.q = pgPb.q();
    forwardKinematics(mb, mbcWork);
    bodyDist = (mbcWork.bodyPosW[index1].translation() -
                mbcWork.bodyPosW[index2].translation()).norm();
    BOOST_CHECK_GT(bodyDist, 1. + 0.1);
    toPython(mb, mbcWork, rc.forceContacts, pgPb.forces(),"Z12SelfCol1.py");


    rc.bodyPosTargets = {};
    // same check but we start in constraint violation
    pgPb.robotConfigs({rc}, gravity);
    BOOST_REQUIRE(pgPb.run({{qOrigin, {}, mbcInit.q}}));

    mbcWork.q = pgPb.q();
    forwardKinematics(mb, mbcWork);
    bodyDist = (mbcWork.bodyPosW[index1].translation() -
                mbcWork.bodyPosW[index2].translation()).norm();
    BOOST_CHECK_GT(bodyDist, 1. + 0.1);
    toPython(mb, mbcWork, rc.forceContacts, pgPb.forces(),"Z12SelfCol2.py");
  }

  /*
   *                              MultiRobot
   */
  {
    pg::PostureGenerator pgPb;
    pg::RobotConfig rc1(mb), rc2(mb);
    pgPb.param("ipopt.print_level", 0);
    pgPb.param("ipopt.linear_solver", "mumps");

    Vector3d target(2., 0., 0.);
    Matrix3d oriTarget(sva::RotZ(-cst::pi<double>()));
    std::string id = "b12";
    int index = mb.bodyIndexByName(id);
    rc1.fixedPosContacts = {{id, target, sva::PTransformd::Identity()}};
    rc1.fixedOriContacts = {{id, oriTarget, sva::PTransformd::Identity()}};

    pg::RobotLink rl(0, 1, {{id, sva::PTransformd::Identity(),
                             sva::PTransformd::Identity()}});
    pg::RunConfig rc({mbcInit.q, {}, mbcInit.q});

    pgPb.robotConfigs({rc1, rc2}, gravity);
    pgPb.robotLinks({rl});
    BOOST_REQUIRE(pgPb.run({rc, rc}));

    rbd::MultiBodyConfig mbcWork2(mb);
    mbcWork.q = pgPb.q(0);
    mbcWork2.q = pgPb.q(1);

    forwardKinematics(mb, mbcWork);
    forwardKinematics(mb, mbcWork2);
    sva::PTransformd body1(mbcWork.bodyPosW[index]);
    sva::PTransformd body2(mbcWork2.bodyPosW[index]);
    BOOST_CHECK_SMALL((body1.translation() - body2.translation()).norm(), 1e-5);
    BOOST_CHECK_SMALL((body1.rotation() - body2.rotation()).norm(), 1e-3);
  }

  /*
   *                        CylindricalContact
   */
  {
    pg::PostureGenerator pgPb;
    pg::RobotConfig rc(mb);
    pgPb.param("ipopt.print_level", 0);
    pgPb.param("ipopt.linear_solver", "mumps");

    std::string id = "b12";
    int index = mb.bodyIndexByName(id);
    Matrix3d bodyFrame(RotX(cst::pi<double>()/2.)*RotY(cst::pi<double>()/2.));
    sva::PTransformd targetSurface(RotY(cst::pi<double>()/2.), Vector3d(0., 1., 0.));
    sva::PTransformd bodySurface(bodyFrame);
    double radius = 0.1;
    rc.cylindricalContacts = {{id, radius, 5., targetSurface, bodySurface}};

    pgPb.robotConfigs({rc}, gravity);
    BOOST_REQUIRE(pgPb.run({{mbcInit.q, {}, mbcInit.q}}));

    mbcWork.q = pgPb.q();
    forwardKinematics(mb, mbcWork);
    sva::PTransformd surfPos = bodySurface*mbcWork.bodyPosW[index];

    Vector3d vec = targetSurface.translation() - surfPos.translation();
    double posErr = vec.norm();
    double oriErr = surfPos.rotation().row(0).dot(targetSurface.rotation().row(0));
    double NErr = vec.normalized().dot(surfPos.rotation().row(2));
    BOOST_CHECK_SMALL(posErr - radius, 1e-5);
    BOOST_CHECK_SMALL(oriErr - 1., 1e-5);
    BOOST_CHECK_SMALL(NErr - 1., 1e-5);
    toPython(mb, mbcWork, rc.forceContacts, pgPb.forces(),"Z12FreeGripper.py");
  }

  {
    pg::PostureGenerator pgPb;
    pg::RobotConfig rc(mb);
    pgPb.param("ipopt.print_level", 0);
    pgPb.param("ipopt.linear_solver", "mumps");

    std::string id = "b12";
    Eigen::Vector3d target(100., 0., 0.);
    rc.bodyPosTargets = {{id, target, 1}};

    // add com plane constraint
    Eigen::Vector3d O(1.5,0.,0.);
    Eigen::Vector3d n(-1.,0.,0.);
    rc.comHalfSpaces = {{{O}, {n}}};

    pgPb.robotConfigs({rc}, gravity);
    BOOST_REQUIRE(pgPb.run({{mbcInit.q, {}, mbcInit.q}}));

    mbcWork.q = pgPb.q();
    forwardKinematics(mb, mbcWork);

    // compute CoM
    Eigen::Vector3d C = rbd::computeCoM(mb, mbcWork);
    Eigen::Vector3d OC = C - O;

    // check that the CoM is below the plane
    BOOST_CHECK_GE(n.dot(OC), -1e-8);

    toPython(mb, mbcWork, rc.forceContacts, pgPb.forces(),"Z12CoMPlane.py");
  }
}
