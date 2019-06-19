#ifndef DISTANCE_MOTION_PLANNER
#define DISTANCE_MOTION_PLANNER

#include "eigen3/Eigen/Dense"
#include "math.h"
#include <sstream>

using namespace Eigen;

namespace auv_guidance
{
// This class performs motion planning along a single axis using the distance to be traveled,
// with which is constrains velocity to a trapezoidal profile

class SegmentPlanner
{
  private:
    double distance_, cruiseSpeed_, acceleration_;
    double cruiseDuration_, initialSpeed_, maxSpeed_, finalSpeed_;
    int accelSeq_;
    double t1_, t2_, tMid_, tEnd_; // Key times. At cruiseSpeed for in the time interval [t1_, t2_]
    bool accelerate_;

  public:
    static const int SEQ_NONE = 0;
    static const int SEQ_START = 1;
    static const int SEQ_END = 2;
    static const int SEQ_BOTH = 3;
    static const double DEFAULT_SPEED = 1.0;

    SegmentPlanner(double distance, double nominalSpeed, double accel = 0.0, int seq = SegmentPlanner::SEQ_NONE);
    void initMotionPlanner();
    double getTravelTime();
    Vector2d computeState(double t);
};
} // namespace auv_guidance

#endif