#ifndef LONG_RANGE_TRAJECTORY
#define LONG_RANGE_TRAJECTORY

#include "auv_guidance/abstract_trajectory.hpp"
#include "auv_guidance/short_range_trajectory.hpp"
#include "auv_guidance/waypoint.hpp"
#include "auv_control/auv_model.hpp"
#include "auv_navigation/auv_math_lib.hpp"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "math.h"
#include <vector>

using namespace Eigen;

namespace AUVGuidance
{
class LongRangeTrajectory : public Trajectory
{
  private:
    ShortRangeTrajectory *srtPreRotation_, *srtSpeedUp_, *srtCruise_, *srtSlowDown_, *srtPostRotation_;
    std::vector<ShortRangeTrajectory*> srtList_;
    std::vector<double> srtTimes_;
    Waypoint *wStart_, *wEnd_, *wPreTranslate_, *wCruiseStart_, *wCruiseEnd_, *wPostTranslate_;
    Quaterniond qStart_, qEnd_, qTravel_;
    
    double travelDuration_, rotationDuration_, accelDuration_, cruiseRatio_, cruiseSpeed_, cruiseDuration_;
    bool newTravelHeading_;
    Vector3d travelVec_, cruiseStartPos_, cruiseEndPos_, cruiseVel_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    static const float MAX_TRAJECTORY_PITCH = 80 * (180 / M_PI); // Min angle [rad] from z-axis to require new travel heading 
    static const float DEFAULT_CRUISE_RATIO = 0.8; // Default is to cruise for 80% of total distance

    LongRangeTrajectory(Waypoint *start, Waypoint *end, double rotationDuration, double accelDuration, 
                        double cruiseRatio, double cruiseSpeed);
    void initTrajectory();
    void initTravelVectors();
    void initWaypoints();
    void initShortRangeTrajectories();
    Vector12d computeState(double time);
    Vector6d computeAccel(double time);
};
} // namespace AUVGuidance

#endif