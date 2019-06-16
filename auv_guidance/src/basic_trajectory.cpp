#include "auv_guidance/basic_trajectory.hpp"

namespace AUVGuidance
{
/**
 * @param start Starting waypoint
 * @param end Ending waypoint
 * @param travelDuration Desired travel duration [s]
 */
BasicTrajectory::BasicTrajectory(TGenLimits *tGenLimits)
{
    tGenLimits_ = tGenLimits;

    qDiff_.w() = 1;
    qDiff_.x() = 0;
    qDiff_.y() = 0;
    qDiff_.z() = 0;
    
    xState_.setZero();
    yState_.setZero();
    zState_.setZero();
    angleState_.setZero();
    rotationAxis_.setZero();

    longTrajectory_ = false;
}

/**
 * Initialize the min jerk trajectories
 */
void BasicTrajectory::setWaypoints(Waypoint *wStart, Waypoint *wEnd)
{
    wStart_ = wStart;
    wEnd_ = wEnd;
    BasicTrajectory::prepSlowDown();
    BasicTrajectory::processNewWaypoint();
}

void BasicTrajectory::prepSlowDown()
{
    // Get rest position
    double velXYZ = wStart_->velI().squaredNorm();
    double accelXYZ = wStart_->accelI().squaredNorm();
    double distance = (2.0/3.0) * (velXYZ * velXYZ) / tGenLimits_->maxXYAccel();
    Vector3d travelVec = wStart_->velI().normalized() * distance;
    
    Vector3d xState = Vector3d::Zero();
    Vector3d yState = Vector3d::Zero();
    Vector3d zState = Vector3d::Zero();
    Vector3d zero3d = Vector3d::Zero();
    xState(0) = wStart_->xI()(0) + travelVec(0);
    yState(0) = wStart_->yI()(0) + travelVec(1);
    zState(0) = wStart_->zI()(0) + travelVec(2);

    // Get rest quaternion
    Vector4d angleAxis = Vector4d::Zero();
    angleAxis.tail<3>() = wStart_->angVelB();
    double angVel = wStart_->angVelB().squaredNorm(); // Magnitude
    double angularDistance = (2.0/3.0) * (angVel * angVel) / tGenLimits_->maxRotAccel();
    angleAxis(0) = angularDistance;
    Quaterniond qRotate = AUVMathLib::angleAxis2Quaternion(angleAxis); // Relative to B-frame
    qRest_ = wStart_->quaternion() * qRotate;

    wRest_ = new Waypoint(xState, yState, zState, qRest_, zero3d);

    // Find travel time for translation and rotation, take the larger one
    double timeTrans = 0, timeRot = 0;
    Vector4d transStart = Vector4d::Zero();
    Vector4d transEnd = Vector4d::Zero();
    Vector4d rotStart = Vector4d::Zero();
    Vector4d rotEnd = Vector4d::Zero();

    transStart << 0, velXYZ, accelXYZ, tGenLimits_->xyzJerk(distance);
    transEnd << distance, 0, 0, tGenLimits_->xyzJerk(distance);
    rotStart << 0, angVel, 0, tGenLimits_->rotJerk(angularDistance);
    rotEnd << angularDistance, 0, 0, tGenLimits_->rotJerk(angularDistance);

    MinJerkTimeSolver *mjts;
    mjts = new MinJerkTimeSolver(transStart, transEnd);
    timeTrans = mjts->getTime();
    mjts = new MinJerkTimeSolver(rotStart, rotEnd);
    timeRot = mjts->getTime();
    timeRest_ = std::max(timeTrans, timeRot); // Take longer duration

    stRest_ = new SimultaneousTrajectory(wStart_, wRest_, timeRest_);
}

void BasicTrajectory::processNewWaypoint()
{
    double posX1 = wRest_->xI()(0);
    double posY1 = wRest_->yI()(0);
    double posZ1 = wRest_->zI()(0);
    double posX2 = wEnd_->xI()(0);
    double posY2 = wEnd_->yI()(0);
    double posZ2 = wEnd_->zI()(0);
    double deltaX = wEnd_->xI()(0) - wRest_->xI()(0);
    double deltaY = wEnd_->yI()(0) - wRest_->yI()(0);
    double deltaZ = wEnd_->zI()(0) - wRest_->zI()(0);

    double distanceXY = sqrt(deltaX * deltaX + deltaY * deltaY);
    double distanceZ = fabs(deltaZ);
    double distance = (wEnd_->posI() - wRest_->posI()).squaredNorm();

    if (distanceXY > tGenLimits_->maxXYDistance())
        longTrajectory_ = true;
    else if (distanceZ > tGenLimits_->maxZDistance())
        longTrajectory_ = true;

    // Get translation time
    Vector4d transStart = Vector4d::Zero();
    Vector4d transEnd = Vector4d::Zero();
    transStart << 0, 0, 0, tGenLimits_->xyzJerk(distance);
    transEnd << distance, 0, 0, tGenLimits_->xyzJerk(distance);

    double timeTrans = 0;
    MinJerkTimeSolver *mjts;
    mjts = new MinJerkTimeSolver(transStart, transEnd);
    timeTrans = mjts->getTime();

    double midVelocity = mjts->getMiddleVelocity();

}

/**
 * @param time Time to compute the state at
 * Computes the trajectory state at the specified time
 */
Vector12d BasicTrajectory::computeState(double time)
{
    Vector12d state;
    state.setZero();

    
    return state;
}

/**
 * @param time Time to compute accelerations at
 * Compute inertial translational acceleration and time-derivative of angular veocity, 
 * both expressed in B-frame, at specified time
 */
Vector6d BasicTrajectory::computeAccel(double time)
{
    Vector6d accel;
    accel.setZero();

    return accel;
}
} // namespace AUVGuidance