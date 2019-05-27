#ifndef TRAJECTORY_GENERATOR_ABSTRACT
#define TRAJECTORY_GENERATOR_ABSTRACT

#include "eigen3/Eigen/Dense"

using namespace Eigen;

namespace AUV_GNC
{
class TrajectoryGenerator
{
public:
    TrajectoryGenerator();
    virtual VectorXf computeState(float time);
};
}

#endif