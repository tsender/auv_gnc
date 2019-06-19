#ifndef TRAJECTORY_GENERATOR_ABSTRACT
#define TRAJECTORY_GENERATOR_ABSTRACT

#include "eigen3/Eigen/Dense"

namespace auv_guidance
{
typedef Eigen::Matrix<double, 12, 1> Vector12d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;

class Trajectory
{
public:
    Trajectory();
    virtual Vector12d computeState(double time);
    virtual Vector6d computeAccel(double time);
};
}

#endif