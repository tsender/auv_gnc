#ifndef MIN_JERK_TIME_SOLVER
#define MIN_JERK_TIME_SOLVER

#include "ceres/ceres.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"

using namespace Eigen;

namespace AUVGuidance
{
class MinJerkTimeSolver
{
  private:
    double x0_, v0_, a0_;
    double xf_, vf_, af_;
    double deltaX_;

  public:
    MinJerkTimeSolver(const Ref<const Vector3d> &start, const Ref<const Vector3d> &end)
    {
        x0_ = start(1);
        v0_ = start(2);
        a0_ = 0;
        xf_ = end(1);
        vf_ = end(2);
        af_ = 0;
        deltaX_ = end(0) - start(0);
    }

    template <typename T>
    bool operator()(const T *const dt, T *residual) const
    {
        T c0 = T(x0_);
        T c1 = T(v0_) * dt[0];
        T c2 = T(0.5 * a0_) * dt[0] * dt[0];
        T c3 = T(10.0 * (xf_ - x0_)) - T(6.0 * v0_) * dt[0] - T(1.5 * a0_) * dt[0] * dt[0] - T(4.0 * vf_) * dt[0] + T(0.5 * af_) * dt[0] * dt[0];
        T c4 = T(15.0 * (x0_ - xf_)) + T(8.0 * v0_) * dt[0] + T(1.5 * a0_) * dt[0] * dt[0] + T(7.0 * vf_) * dt[0] - T(af_) * dt[0] * dt[0];
        T c5 = T(6.0 * (xf_ - x0_)) - T(3.0 * (v0_ + vf_)) * dt[0] + T(0.5 * (af_ - af_)) * dt[0] * dt[0];

        residual[0] = (c0 + T(0.5) * c1 + T(1.0/3.0) * c2 + T(1.0/4.0) * c3 + T(1.0/5.0) * c4 + T(1.0/6.0) * c5) * dt[0] - T(deltaX_);

        return true;
    }
};

} // namespace AUVGuidance

#endif