#ifndef NOMINAL_THRUST_SOLVER
#define NOMINAL_THRUST_SOLVER

#include "ceres/ceres.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "auv_navigation/auv_math_lib.hpp"

using namespace Eigen;

namespace AUV_GNC
{
typedef Matrix<double, 6, 1> Vector6d;
typedef Matrix<double, 6, 2> Matrix62d;
typedef Matrix<double, 6, 10> Matrix610d;
class NominalThrustSolver
{
private:
  double mass_, Fg_, Fb_, density_;
  Matrix3d inertia_;
  Vector3d CoB_;
  Matrix62d dragCoeffs_;
  Matrix610d thrustCoeffs_; // Thrust coefficients (effective contributions of each thruster for force and moments)
  double *state_;           // (Pointer) State vector (U, V, W, P, Q, R)
  double *stateDot_;        // (Pointer) Time derivative of state vector
  double *quaternion_;       // (Pointer) to quaternion for orientation

  static const double GRAVITY = 9.80665;

public:
  NominalThrustSolver(double Fg, double Fb, double density, const Ref<const Matrix3d> &inertia, const Ref<const Vector3d> &CoB,
                      const Ref<const Matrix62d> &dragCoeffs, const Ref<const Matrix610d> &thrustCoeffs, 
                      double *quaternion, double *state, double *stateDot)
  {
    Fg_ = Fg;
    Fb_ = Fb;
    mass_ = Fg_ / NominalThrustSolver::GRAVITY;
    density_ = density;
    inertia_ = inertia;
    CoB_ = CoB;
    dragCoeffs_ = dragCoeffs;
    thrustCoeffs_ = thrustCoeffs;
    quaternion_ = quaternion;
    state_ = state;
    stateDot_ = stateDot;
  }

  template <typename T>
  bool operator()(const T *const nominal_forces, T *residuals_ptr) const
  {
    // NOTE: MUST have extra space in nested <> to compile: "<<...> >(...)"
    //                                                             ^

    // Map private variables to Eigen objects with Jet type
    Eigen::Map<const Eigen::Matrix<T, 6, 2> > dragCoeffs(dragCoeffs_);
    Eigen::Map<const Eigen::Matrix<T, 6, 10> > thrustCoeffs(thrustCoeffs_);
    Eigen::Map<const Eigen::Matrix<T, 3, 3> > inertia(inertia_);
    Eigen::Map<const Eigen::Matrix<T, 3, 1> > CoB(CoB_);
    Eigen::Map<const Eigen::Matrix<T, 6, 1> > state(state_);
    Eigen::Map<const Eigen::Matrix<T, 6, 1> > stateDot(stateDot_);

    // Map ceres parameters and residuals to Eigen object with Jet type
    Eigen::Map<const Eigen::Matrix<T, 10, 1> > forces(nominal_forces);
    Eigen::Map<Eigen::Matrix<T, 10, 1> > residuals(residuals_ptr);

    // Compute quaternion
    // Using Eigen::Quaternion: quaternion * vector = rotates vector by the described axis-angle
    // So: B-frame vector = quaternion.conjugate() * I-frame vector
    // So: I-frame vector = quaternion * B-frame vector
    Eigen::Map<const Eigen::Matrix<T, 4, 1> > quat(quaternion_[0], quaternion_[1], quaternion_[2], quaternion_[3]);
    /*Eigen::Matrix<T, 3, 3> Rquat;
    Rquat(0, 0) = q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3);
    Rquat(1, 1) = q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3);
    Rquat(2, 2) = q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
    Rquat(0, 1) = 2 * q(1) * q(2) + 2 * q(0) * q(3);
    Rquat(1, 0) = 2 * q(1) * q(2) - 2 * q(0) * q(3);
    Rquat(0, 2) = 2 * q(1) * q(3) - 2 * q(0) * q(2);
    Rquat(2, 0) = 2 * q(1) * q(3) + 2 * q(0) * q(2);
    Rquat(1, 2) = 2 * q(2) * q(3) + 2 * q(0) * q(1);
    Rquat(2, 1) = 2 * q(2) * q(3) - 2 * q(0) * q(1);*/

    // Translational Equations
    Eigen::Matrix<T, 3, 1> weightAccel, transDrag;
    weightAccel.setZero();
    transDrag.setZero();
    weightAccel(2) = T((Fg_ - Fb_) / mass_);

    transDrag(0) = (dragCoeffs(0, 0) * state(0) + T(0.5 * AUVMathLib::sign(state(0)) * density_) * dragCoeffs(0, 1) * state(0) * state(0)) / T(mass_);
    transDrag(1) = (dragCoeffs(1, 0) * state(1) + T(0.5 * AUVMathLib::sign(state(1)) * density_) * dragCoeffs(1, 1) * state(1) * state(1)) / T(mass_);
    transDrag(2) = (dragCoeffs(2, 0) * state(2) + T(0.5 * AUVMathLib::sign(state(2)) * density_) * dragCoeffs(2, 1) * state(2) * state(2)) / T(mass_);
    residuals.template head<3>() = (stateDot.template head<3>()) - 
                                  ((quat.conjugate() * weightAccel) - transDrag - 
                                  (state.template tail<3>()).cross(state.template head<3>()) + 
                                  (thrustCoeffs.template block<3,10>(0,0)) * forces);

    // Rotational Equations
    Eigen::Matrix<T, 3, 1> forceBuoyancy, rotDrag;
    forceBuoyancy.setZero();
    rotDrag.setZero();
    forceBuoyancy(2) = T(-Fb_);

    rotDrag(0) = (dragCoeffs(3, 0) * state(3) + T(0.5 * AUVMathLib::sign(state(3)) * density_) * dragCoeffs(3, 1) * state(3) * state(3));
    rotDrag(1) = (dragCoeffs(4, 0) * state(4) + T(0.5 * AUVMathLib::sign(state(4)) * density_) * dragCoeffs(4, 1) * state(4) * state(4));
    rotDrag(2) = (dragCoeffs(5, 0) * state(5) + T(0.5 * AUVMathLib::sign(state(5)) * density_) * dragCoeffs(5, 1) * state(5) * state(5));
    residuals.template tail<3>() = (stateDot.template tail<3>()) - 
                                  (inertia.inverse() * (-rotDrag + CoB.cross(quat.conjugate() * forceBuoyancy) - 
                                  (state.template tail<3>()).cross(inertia * (state.template tail<3>()))) +
                                  (thrustCoeffs.template block<3,10>(3,0)) * forces);
    return true;
  }
};

} // namespace AUV_GNC

#endif