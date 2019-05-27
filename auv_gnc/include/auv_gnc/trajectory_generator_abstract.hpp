#ifndef TRAJECTORY_GENERATOR_ABSTRACT
#define TRAJECTORY_GENERATOR_ABSTRACT

#include "eigen3/Eigen/Dense"

using namespace Eigen;
typedef Matrix<float, 12, 1> Vector12f;

namespace AUV_GNC
{
class TrajectoryGenerator
{
public:
    TrajectoryGenerator();
    virtual Vector12f computeState(float time);
};
}

#endif