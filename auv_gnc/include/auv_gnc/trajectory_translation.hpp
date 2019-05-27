#ifndef TRAJECTORY_TRANSLATION
#define TRAJECTORY_TRANSLATION

#include "eigen3/Eigen/Dense"
#include "math.h"

using namespace Eigen;

namespace AUV_GNC
{
namespace Translation
{
class Line : public TrajectoryGenerator
{
public:
    TrajectoryGenerator();
    virtual VectorXf computeState(float time);
};
}
}

#endif