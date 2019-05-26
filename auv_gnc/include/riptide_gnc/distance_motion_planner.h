#ifndef DISTANCE_MOTION_PLANNER
#define DISTANCE_MOTION_PLANNER

#include "eigen3/Eigen/Dense"
#include "math.h"

using namespace Eigen;

// This class performs motion planning along a single axis using the distance to be traveled, 
// with which is constrains velocity to a trapezoidal profile

class DistanceMotionPlanner
{
private:
    float distance_, cruiseSpeed_, acceleration_;
    float cruiseDuration_, velocity0_, acceleration1_, acceleration2_;
    int accelSeq_;
    float t1_, t2_, tMid_, tEnd_; // Key times. At cruiseSpeed for in the time interval [t1_, t2_]
    bool accelerate_;
public:
    const int SEQ_NONE = 0;
    const int SEQ_START = 1;
    const int SEQ_END = 2;
    const int SEQ_BOTH = 3;
    const float DEFAULT_SPEED = 1.0;
    
    DistanceMotionPlanner(float startPos, float nominalSpeed);
    void SetAcceleration(float accel, int seq);
    void initMotionPlanner();
    float getTravelTime();
    Vector2f computeState(float t);
}

#endif