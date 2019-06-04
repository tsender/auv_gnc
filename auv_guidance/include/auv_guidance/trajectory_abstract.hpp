#ifndef TRAJECTORY_GENERATOR_ABSTRACT
#define TRAJECTORY_GENERATOR_ABSTRACT

#include "eigen3/Eigen/Dense"
using namespace Eigen;

namespace AUVGuidance
{
typedef Matrix<double, 12, 1> Vector12d;
class Trajectory
{
public:
    Trajectory();
    virtual Vector12d computeState(double time);
};
}

#endif