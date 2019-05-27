#ifndef TRAJECTORY_TRANSLATION
#define TRAJECTORY_TRANSLATION

#include "auv_gnc/trajectory_generator_abstract.hpp"
#include "eigen3/Eigen/Dense"
#include "math.h"

using namespace Eigen;

namespace AUV_GNC
{
namespace Translation
{
class Line : public TrajectoryGenerator
{
private:
public:
    static const float DEFAULT_SPEED = 0.5; // [m/s]
    static const float DEFAULT_ACCEL = 0.2; // [m/s^2]
    
    Line();
    Vector12f computeState(float time);
};
}
}

#endif