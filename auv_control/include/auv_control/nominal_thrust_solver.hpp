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
  double *accelState_;           // (Pointer) Acceleration state vector (U, V, W, P, Q, R)
  double *accelStateDot_;        // (Pointer) Time derivative of acceleration state vector
  double *quaternion_;       // (Pointer) to quaternion for orientation

  static const double GRAVITY = 9.80665;

public:
  NominalThrustSolver(double Fg, double Fb, double density, const Ref<const Matrix3d> &inertia, const Ref<const Vector3d> &CoB,
                      const Ref<const Matrix62d> &dragCoeffs, const Ref<const Matrix610d> &thrustCoeffs, 
                      double *quaternion, double *accelState, double *accelStateDot)
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
    accelState_ = accelState;
    accelStateDot_ = accelStateDot;
  }

  template <typename T>
  bool operator()(const T *const nominalForces, T *residuals) const
  {
    // NOTE: MUST have extra space in nested <> to compile: "<<...> >(...)"
    //                                                             ^

    // Cast Eigen objects to Jet type
    Eigen::Matrix<T, 6, 2> dragCoeffsT = dragCoeffs_.cast<T>();
    Eigen::Matrix<T, 6, 10> thrustCoeffsT = thrustCoeffs_.cast<T>();
    Eigen::Matrix<T, 3, 3> inertiaT = inertia_.cast<T>();
    Eigen::Matrix<T, 3, 1> CoBT = CoB_.cast<T>();

    // Cast accelState_ and accelStateDot_ to Jet type
    Eigen::Matrix<T, 6, 1> accelStateT, accelStateDotT;
    for (int i = 0; i < 6; i++)
    {
      accelStateT(i) = T(accelState_[i]);
      accelStateDotT(i) = T(accelStateDot_[i]);
    }

    // Map ceres parameters and residuals to Eigen object with Jet type
    Eigen::Map<const Eigen::Matrix<T, 10, 1> > nominalForcesT(nominalForces);
    Eigen::Map<Eigen::Matrix<T, 10, 1> > residualsT(residuals);

    // Compute quaternion
    // Using Eigen::Quaternion: quaternion * vector = rotates vector by the described axis-angle
    // So: B-frame vector = quaternion.conjugate() * I-frame vector
    // So: I-frame vector = quaternion * B-frame vector
    Eigen::Quaterniond quat(quaternion_[0], quaternion_[1], quaternion_[2], quaternion_[3]);
    Eigen::Quaternion<T> quatT = quat.cast<T>();

    // Translational Equations
    Eigen::Matrix<T, 3, 1> weightAccelT, transDragT;
    weightAccelT.setZero();
    transDragT.setZero();
    weightAccelT(2) = T((Fg_ - Fb_) / mass_);

    transDragT(0) = (dragCoeffsT(0, 0) * accelStateT(0) + T(0.5 * AUVMathLib::sign(accelState_[0]) * density_) * dragCoeffsT(0, 1) * accelStateT(0) * accelStateT(0)) / T(mass_);
    transDragT(1) = (dragCoeffsT(1, 0) * accelStateT(1) + T(0.5 * AUVMathLib::sign(accelState_[1]) * density_) * dragCoeffsT(1, 1) * accelStateT(1) * accelStateT(1)) / T(mass_);
    transDragT(2) = (dragCoeffsT(2, 0) * accelStateT(2) + T(0.5 * AUVMathLib::sign(accelState_[2]) * density_) * dragCoeffsT(2, 1) * accelStateT(2) * accelStateT(2)) / T(mass_);
    residualsT.template head<3>() = (accelStateDotT.template head<3>()) - 
                                  ((quatT.conjugate() * weightAccelT) - transDragT - 
                                  (accelStateT.template tail<3>()).cross(accelStateT.template head<3>()) + 
                                  (thrustCoeffsT.template block<3,10>(0,0)) / T(mass_) * nominalForcesT);

    // Rotational Equations
    Eigen::Matrix<T, 3, 1> forceBuoyancyT, rotDragT;
    forceBuoyancyT.setZero();
    rotDragT.setZero();
    forceBuoyancyT(2) = T(-Fb_);

    rotDragT(0) = (dragCoeffsT(3, 0) * accelStateT(3) + T(0.5 * AUVMathLib::sign(accelState_[3]) * density_) * dragCoeffsT(3, 1) * accelStateT(3) * accelStateT(3));
    rotDragT(1) = (dragCoeffsT(4, 0) * accelStateT(4) + T(0.5 * AUVMathLib::sign(accelState_[4]) * density_) * dragCoeffsT(4, 1) * accelStateT(4) * accelStateT(4));
    rotDragT(2) = (dragCoeffsT(5, 0) * accelStateT(5) + T(0.5 * AUVMathLib::sign(accelState_[5]) * density_) * dragCoeffsT(5, 1) * accelStateT(5) * accelStateT(5));
    residualsT.template tail<3>() = (accelStateDotT.template tail<3>()) - 
                                  (inertiaT.inverse() * (-rotDragT + CoBT.cross(quatT.conjugate() * forceBuoyancyT) - 
                                  (accelStateT.template tail<3>()).cross(inertiaT * (accelStateT.template tail<3>())) +
                                  (thrustCoeffsT.template block<3,10>(3,0)) * nominalForcesT));
    return true;
  }
};

} // namespace AUV_GNC

#endif