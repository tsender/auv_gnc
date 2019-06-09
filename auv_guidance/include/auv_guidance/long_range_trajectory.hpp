#ifndef LONG_RANGE_TRAJECTORY
#define LONG_RANGE_TRAJECTORY

#include "auv_guidance/abstract_trajectory.hpp"
#include "auv_guidance/short_range_trajectory.hpp"
#include "auv_guidance/min_jerk_trajectory.hpp"
#include "auv_guidance/waypoint.hpp"
#include "auv_control/auv_model.hpp"
#include "auv_navigation/auv_math_lib.hpp"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "math.h"

using namespace Eigen;

namespace AUVGuidance
{
typedef Matrix<double, 6, 1> Vector6d;
class LongRangeTrajectory : public Trajectory
{
  private:
    ShortRangeTrajectory *srtPreCruiseRot_, *srtPreCruiseTrans_, *srtPostCruiseTrans_, *srtPostCruiseRot_;
    Waypoint *start_, *end_, *preCruise_, *cruiseStart_, *cruiseEnd_, *postCruise_;
    Quaterniond qStart_, qEnd_, qTravel_;
    double travelDuration_, rotationDuration_, accelDuration_, cruiseRatio_, cruiseSpeed_, cruiseDuration_;
    bool newTravelHeading_;
    Vector3d travelVec_, preCruisePos_, postCruisePos_, cruiseVel_;
    double t0, t1_, t2_, t3_, t4_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static const float DEFAULT_MIN_Z_ANGLE = 10 * (180 / M_PI); // Min angle [rad] from z-axis to require new travel heading 
    static const float DEFAULT_CRUISE_RATIO = 0.8; // Default is to cruise for 80% of total distance

    LongRangeTrajectory(Waypoint *start, Waypoint *end, double rotationDuration, double accelDuration, 
                        double cruiseRatio, double cruiseSpeed);
    void initTrajectory();
    void initTravelVectors();
    void initCruiseWaypoints();
    Vector12d computeState(double time);
    Vector6d computeAccel(double time);
};
} // namespace AUVGuidance

#endif