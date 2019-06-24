#ifndef ABSTRACT_TRAJECTORY
#define ABSTRACT_TRAJECTORY

#include "eigen3/Eigen/Dense"

namespace auv_guidance
{
typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

/**
 * \brief This is a pure virtual class. All methods MUST be declared in inheriting classes/
 */
class Trajectory
{
  public:
    virtual Vector12d computeState(double time) = 0;
    virtual Vector6d computeAccel(double time) = 0;
};
} // namespace auv_guidance

#endif