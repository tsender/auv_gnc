#ifndef DISTANCE_MOTION_PLANNER
#define DISTANCE_MOTION_PLANNER

#include "eigen3/Eigen/Dense"
#include "math.h"
#include <sstream>

using namespace Eigen;

namespace AUV_GNC
{
// This class performs motion planning along a single axis using the distance to be traveled, 
// with which is constrains velocity to a trapezoidal profile

class SegmentPlanner
{
private:
    float distance_, cruiseSpeed_, acceleration_;
    float cruiseDuration_, initialSpeed_, maxSpeed_, finalSpeed_;
    int accelSeq_;
    float t1_, t2_, tMid_, tEnd_; // Key times. At cruiseSpeed for in the time interval [t1_, t2_]
    bool accelerate_;
public:
    static const int SEQ_NONE = 0;
    static const int SEQ_START = 1;
    static const int SEQ_END = 2;
    static const int SEQ_BOTH = 3;
    static const float DEFAULT_SPEED = 1.0;
    
    SegmentPlanner(float distance, float nominalSpeed, float accel = 0.0, int seq = SegmentPlanner::SEQ_NONE);
    void initMotionPlanner();
    float getTravelTime();
    Vector2f computeState(float t);
};
}

#endif