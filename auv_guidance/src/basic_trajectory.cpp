#include "auv_guidance/basic_trajectory.hpp"

namespace AUVGuidance
{
BasicTrajectory::BasicTrajectory(Waypoint *start, Waypoint *end, double time1, double time2)
{
    start_ = start;
    end_ = end;
    if (time2 == 0)
    {
        simultaneous_ = true;
        travelTime_ = time1;
        translationTime_ = 0;
        rotationTime_ = 0;
    }
    else
    {
        simultaneous_ = false;
        travelTime_ = 0;
        translationTime_ = time1;
        rotationTime_ = time2;
    }

    qDiff_.w() = 1;
    qDiff_.x() = 0;
    qDiff_.y() = 0;
    qDiff_.z() = 0;
    BasicTrajectory::initTrajectory();
}

void BasicTrajectory::initTrajectory()
{
    
    
    Quaterniond q1 = start_->quaternion().normalized();
    Quaterniond q2 = end_->quaternion().normalized();
    double angle = q1.angularDistance(q2) * M_PI / 180;
    qDiff_ = q1.conjugate() * q2; // Error quaternion wrt B-frame (q2 * q1.conjugate is opposite)

    
}

Vector12d BasicTrajectory::computeState(double time)
{
    Vector12d state;
    state.setZero();

    /*// Translation
    Vector2d transSegState = translationSegPlanner_->computeState(time);
    Vector3d inertialPosition = start_->position() + insertionMap_ * transSegState(0); // wrt I-frame
    Vector3d inertialVelocity = insertionMap_ * transSegState(1); // wrt I-frame

    // Rotation
    Vector2d rotSegState = rotationSegPlanner_->computeState(time);
    double frac = time / rotationTime_;
    Quaterniond qSlerp = start_->quaternion().slerp(frac, end_->quaternion()); // Attitude wrt I-frame
    qSlerp = qSlerp.normalized();
    
    inertialVelocity = qSlerp.conjugate() * inertialVelocity; // wrt B-frame
    Vector3d angularVelocity = qSlerp.vec() * rotSegState(1); // Angular velocity wrt B-frame*/
    
    /*Vector4d q;
    q(0) = qSlerp.w();
    q(1) = qSlerp.x();
    q(2) = qSlerp.y();
    q(3) = qSlerp.z();

    state.segment<3>(AUVControl::AUVModel::xI_) = inertialPosition;
    state.segment<3>(AUVControl::AUVModel::U_) = inertialVelocity;
    state.segment<3>(AUVControl::AUVModel::q1_) = q.tail<3>();
    state.segment<3>(AUVControl::AUVModel::P_) = angularVelocity;*/
    return state;

}
} // namespace AUVGuidance