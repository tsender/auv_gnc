#include "auv_guidance/long_trajectory.hpp"

namespace auv_guidance
{
/**
 * @param start Starting waypoint
 * @param end Ending waypoint
 * @param constraints Pointer to AUV constraints
 * @param cruiseRatio Indicates what fraction of the total distance is to be traveled while cruising
 * @param cruiseSpeed Vehicle speed while cruising
 */
LongTrajectory::LongTrajectory(Waypoint *wStart, Waypoint *wEnd, auv_core::auvConstraints *constraints, double cruiseRatio, double cruiseSpeed)
{
   wStart_ = wStart;
   wEnd_ = wEnd;
   auvConstraints_ = constraints;

   totalDuration_ = 0;
   rotationDuration1_ = 0;
   rotationDuration2_ = 0;
   accelDuration_ = 0;
   cruiseDuration_ = 0;
   cruiseSpeed_ = cruiseSpeed;
   newTravelHeading_ = false;

   if (cruiseRatio > 0 && cruiseRatio < 1)
      cruiseRatio_ = cruiseRatio;
   else
      cruiseRatio_ = 0.1; // Default cruise ratio

   stList_.clear();
   stTimes_.clear();

   unitVec_.setZero();
   deltaVec_.setZero();
   cruiseStartPos_.setZero();
   cruiseEndPos_.setZero();
   cruiseVel_.setZero();

   LongTrajectory::initTrajectory();
}

/**
 * \brief Initialize all trajectories
 */
void LongTrajectory::initTrajectory()
{
   qStart_ = wStart_->quaternion();
   qEnd_ = wEnd_->quaternion();
   qCruise_ = qStart_;

   // Determine travel attitude
   double dx = wEnd_->posI()(0) - wStart_->posI()(0);
   double dy = wEnd_->posI()(1) - wStart_->posI()(1);
   double dz = wEnd_->posI()(2) - wStart_->posI()(2);
   double xyDistance = sqrt(dx * dx + dy * dy);
   double travelHeading = 0;

   // If motion mostly along XY plane, then set a level cruise attitude (zero roll and pitch)
   if ((xyDistance > 0) && (atan(fabs(dz) / xyDistance) < auvConstraints_->maxAlignInclination))
   {
      travelHeading = atan2(dy, dx); // Radians
      qCruise_ = auv_core::rot3d::rpy2Quat(0, 0, travelHeading);
      newTravelHeading_ = true;
      std::cout << "LT: path inclination " << atan(fabs(dz) / xyDistance) * 180 / M_PI << " < " << auvConstraints_->maxAlignInclination * 180 / M_PI << std::endl; // Debug
      std::cout << "LT: new cruise heading [deg]: " << travelHeading * 180 / M_PI << std::endl;                                                                                        // Debug
   }

   LongTrajectory::initWaypoints();
   LongTrajectory::initSimultaneousTrajectories();
}

/**
 * \brief Initialize all waypoints
 */
void LongTrajectory::initWaypoints()
{
   // Init travel vectors
   deltaVec_ = wEnd_->posI() - wStart_->posI();
   unitVec_ = deltaVec_.normalized();
   double accelDistance = (1.0 - cruiseRatio_) * deltaVec_.norm() / 2.0;

   // Calculate accel duration
   Eigen::Vector4d transStart = Eigen::Vector4d::Zero();
   Eigen::Vector4d transEnd = Eigen::Vector4d::Zero();
   transStart << 0, 0, 0, auvConstraints_->transJerk;
   transEnd << accelDistance, 0, 0, auvConstraints_->transJerk;
   MinJerkTimeSolver *mjts;
   mjts = new MinJerkTimeSolver(transStart, transEnd);
   accelDuration_ = mjts->getDuration();

   // Init position vectors and cruise duration
   cruiseStartPos_ = wStart_->posI() + (accelDistance * unitVec_);
   cruiseEndPos_ = wEnd_->posI() - (accelDistance * unitVec_);
   cruiseVel_ = cruiseSpeed_ * unitVec_;
   cruiseDuration_ = deltaVec_.norm() * cruiseRatio_ / cruiseSpeed_;

   // Init waypoints: pre-translate -> cruise-start -> cruise-end -> post-translate
   // Pre/Post translate waypoints are at rest
   Eigen::Vector3d zero3d = Eigen::Vector3d::Zero();
   wPreTranslate_ = new Waypoint(wStart_->posI(), zero3d, zero3d, qCruise_, zero3d);
   wCruiseStart_ = new Waypoint(cruiseStartPos_, cruiseVel_, zero3d, qCruise_, zero3d);
   wCruiseEnd_ = new Waypoint(cruiseEndPos_, cruiseVel_, zero3d, qCruise_, zero3d);
   wPostTranslate_ = new Waypoint(wEnd_->posI(), zero3d, zero3d, qCruise_, zero3d);
}

/**
 * \brief Initialize all simultaneous trajectories
 */
void LongTrajectory::initSimultaneousTrajectories()
{
   totalDuration_ = 0;
   stList_.clear();
   stTimes_.clear();
   Eigen::Quaterniond qRel = Eigen::Quaterniond::Identity();

   if (newTravelHeading_)
   {
      qRel = qStart_.conjugate() * qCruise_; // auv_core::rot3d::relativeQuat(qStart_, qCruise_);
      rotationDuration1_ = LongTrajectory::computeRotationDuration(qRel);
      stPreRotation_ = new SimultaneousTrajectory(wStart_, wPreTranslate_, rotationDuration1_);
      stList_.push_back(stPreRotation_);
      totalDuration_ += rotationDuration1_;
      stTimes_.push_back(totalDuration_);
   }

   stSpeedUp_ = new SimultaneousTrajectory(wPreTranslate_, wCruiseStart_, accelDuration_);
   stList_.push_back(stSpeedUp_);
   totalDuration_ += accelDuration_;
   stTimes_.push_back(totalDuration_);

   stCruise_ = new SimultaneousTrajectory(wCruiseStart_, wCruiseEnd_, cruiseDuration_);
   stList_.push_back(stCruise_);
   totalDuration_ += cruiseDuration_;
   stTimes_.push_back(totalDuration_);

   stSlowDown_ = new SimultaneousTrajectory(wCruiseEnd_, wPostTranslate_, accelDuration_);
   stList_.push_back(stSlowDown_);
   totalDuration_ += accelDuration_;
   stTimes_.push_back(totalDuration_);

   qRel = qCruise_.conjugate() * qEnd_; // auv_core::rot3d::relativeQuat(qCruise_, qEnd_); //
   rotationDuration2_ = LongTrajectory::computeRotationDuration(qRel);
   stPostRotation_ = new SimultaneousTrajectory(wPostTranslate_, wEnd_, rotationDuration2_);
   stList_.push_back(stPostRotation_);
   totalDuration_ += rotationDuration2_;
   stTimes_.push_back(totalDuration_);
}

/**
 * @param qRel Difference quaternion wrt B-frame (qRel = q1.conjugate * q2)
 * \brief Compute rotation duration given difference quaternion within max speed allowed
 */
double LongTrajectory::computeRotationDuration(Eigen::Quaterniond qRel)
{
   double angularDistance = auv_core::rot3d::quat2AngleAxis(qRel)(0);
   angularDistance = fabs(auv_core::rot3d::mapRollYaw(angularDistance));

   Eigen::Vector4d rotStart = Eigen::Vector4d::Zero();
   Eigen::Vector4d rotEnd = Eigen::Vector4d::Zero();
   rotStart << 0, 0, 0, auvConstraints_->rotJerk;
   rotEnd << angularDistance, 0, 0, auvConstraints_->rotJerk;

   MinJerkTimeSolver *mjts;
   mjts = new MinJerkTimeSolver(rotStart, rotEnd);

   // Check max rotation speed
   double rotDuration = mjts->getDuration();
   MinJerkTrajectory *mjt;
   mjt = new MinJerkTrajectory(rotStart.head<3>(), rotEnd.head<3>(), rotDuration);
   double maxRotVel = mjt->getMiddleVelocity();

   // Iterate until max attained speed is within limits
   int maxIter = 10;
   int iter = 0;
   while (maxRotVel > auvConstraints_->maxRotVel && iter < maxIter)
   {
      rotDuration = rotDuration * (maxRotVel / auvConstraints_->maxRotVel);
      mjt = new MinJerkTrajectory(rotStart.head<3>(), rotEnd.head<3>(), rotDuration);
      maxRotVel = mjt->getMiddleVelocity();
      iter++;
   }

   return rotDuration;
}

/**
 * \brief Return total duration
 */
double LongTrajectory::getDuration()
{
   return totalDuration_;
}

/**
 * @param time Time to compute the state at
 * \brief Computes the trajectory state at the specified time
 */
auv_core::Vector13d LongTrajectory::computeState(double time)
{
   if (time < 0)
      return stList_.front()->computeState(time);
   if (time > totalDuration_)
      return stList_.back()->computeState(time);

   for (int i = 0; i < stList_.size(); i++)
   {
      if (time < stTimes_[i])
      {
         double t = (i == 0) ? time : time - stTimes_[i - 1];
         return stList_[i]->computeState(t);
      }
   }
}

/**
 * @param time Time to compute accelerations at
 * \brief Compute the trajectory acceleration at specified time (inertial translational acceleration and time-derivative of angular velocity), 
 * both expressed in B-frame.
 */
auv_core::Vector6d LongTrajectory::computeAccel(double time)
{
   if (time < 0)
      return stList_.front()->computeAccel(time);
   if (time > totalDuration_)
      return stList_.back()->computeAccel(time);

   for (int i = 0; i < stList_.size(); i++)
   {
      if (time < stTimes_[i])
      {
         double t = (i == 0) ? time : time - stTimes_[i - 1];
         return stList_[i]->computeAccel(t);
      }
   }
}
} // namespace auv_guidance