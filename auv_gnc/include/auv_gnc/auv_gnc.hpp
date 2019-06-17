#ifndef AUV_GNC
#define AUV_GNC

#include "auv_guidance/abstract_trajectory.hpp"
#include "auv_guidance/segment_planner.hpp"
#include "eigen3/Eigen/Dense"
#include "math.h"

namespace AUV_GNC
{

class AUVGNC
{
private:

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  AUVGNC();
};
} // namespace AUV_GNC

#endif