#include "auv_guidance/parallel_trajectory.hpp"

namespace AUV_GNC
{
namespace Trajectory
{
/**
 * @param initialAttitude Initial attitude [yaw, pitch, roll], in [rad].
 * @param eulerAngle Specific Euler angle to change: EulerRotation::(ROLL, PITCH, or YAW)
 * @param deltaTheta Angle of rotation, in [rad]
 * @param nominalSpeed Nominal travel speed, in [rad/s].
 * @param acceleration Desired (absolute) acceleration, in [deg/s^2].
 * @param seq Acceleration sequence, can be one of SegmentPlanner::(SEQ_NONE, SEQ_START, SEQ_END, SEQ_BOTH)
 */
ParallelTrajectory::ParallelTrajectory(const Ref<const Vector3f> initialAttitude, int eulerAngle, float deltaTheta, float nominalSpeed, 
                    float acceleration, int seq)
{
    initialAttitude_ = initialAttitude;
    if (eulerAngle >= EulerRotation::ROLL && eulerAngle <= EulerRotation::YAW)
        eulerAngle_ = eulerAngle;
    else
    {
        std::stringstream ss;
        ss << "EulerRotation: Euler angle type must be one of EulerRotation::(ROLL, PITCH, YAW)" << std::endl;
        throw std::runtime_error(ss.str());
    }

    deltaTheta_ = deltaTheta; // Both positive and negative values allowed

    if (nominalSpeed <= 0) // TODO: May need to be strictly less than 0, will see later
        cruiseSpeed_ = EulerRotation::DEFAULT_SPEED;
    else
        cruiseSpeed_ = abs(nominalSpeed);
    acceleration_ = acceleration;
    accelSeq_ = seq;

    segPlanner_ = new SegmentPlanner(abs(deltaTheta_), cruiseSpeed_, acceleration_, accelSeq_);
}

/**
 * @brief Get travel time for this line segment, in [s].
 */
float ParallelTrajectory::getTravelTime()
{
    return segPlanner_->getTravelTime();
}

/**
 * @param time Desired time to compute the state vector at.
 * @brief Computes the state vector at a given time instance. Returns attitude and body rates.
 */
Vector12f ParallelTrajectory::computeState(float time)
{
    Vector12f state;
    state.setZero();

    Vector2f segState = segPlanner_->computeState(time);

    Vector3f mask = Vector3f::Zero();
    mask(eulerAngle_) = AUVMathLib::sign(deltaTheta_);
    Vector3f continuousAttitude = initialAttitude_ + mask * segState(0);
    Vector3f constrainedAttitude = AUVMathLib::getConstrainedAttitude(continuousAttitude);
    
    Vector3f eulerDot = mask * segState(1); // Rate of change of Euler angles
    Vector3f pqr = AUVMathLib::eulerDot2PQR(constrainedAttitude, eulerDot);

    state.segment<3>(3) = continuousAttitude;
    state.tail<3>() =pqr;

    return state;
}
} // namespace Trajectory
} // namespace AUV_GNC