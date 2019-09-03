#ifndef MONOTONIC_TRAJECTORY_TIME_SOLVER
#define MONOTONIC_TRAJECTORY_TIME_SOLVER

#include "ceres/ceres.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"

namespace auv_guidance
{
class MonotonicTrajectoryTimeSolver
{
private:
   double x0_, v0_, a0_, j0_; // Initial state
   double xf_, vf_, af_, jf_; // Final state

public:
   MonotonicTrajectoryTimeSolver(const Eigen::Ref<const Eigen::Vector4d> &start, const Eigen::Ref<const Eigen::Vector4d> &end)
   {
      x0_ = start(0);
      v0_ = start(1);
      a0_ = start(2);
      j0_ = start(3);
      xf_ = end(0);
      vf_ = end(1);
      af_ = end(2);
      jf_ = end(3);

      if (v0_ == 0 && vf_ == 0)
         v0_ = 0.001; // Both v0 and vf cannot be 0 for algorithm to work
   }

   template <typename T>
   bool operator()(const T *const dt, T *residual) const
   {
      T c0 = T(v0_);
      T c1 = T(a0_) * dt[0];
      T c2 = T(0.5 * j0_) * dt[0] * dt[0];
      T c3 = T(10.0 * (vf_ - v0_)) - T(6.0 * a0_) * dt[0] - T(1.5 * j0_) * dt[0] * dt[0] - T(4.0 * af_) * dt[0] + T(0.5 * jf_) * dt[0] * dt[0];
      T c4 = T(15.0 * (v0_ - vf_)) + T(8.0 * a0_) * dt[0] + T(1.5 * j0_) * dt[0] * dt[0] + T(7.0 * af_) * dt[0] - T(jf_) * dt[0] * dt[0];
      T c5 = T(6.0 * (vf_ - v0_)) - T(3.0 * (a0_ + af_)) * dt[0] + T(0.5 * (jf_ - j0_)) * dt[0] * dt[0];

      residual[0] = (c0 + T(0.5) * c1 + T(1.0 / 3.0) * c2 + T(1.0 / 4.0) * c3 + T(1.0 / 5.0) * c4 + T(1.0 / 6.0) * c5) * dt[0] - T(xf_ - x0_);

      return true;
   }
};

} // namespace auv_guidance

#endif