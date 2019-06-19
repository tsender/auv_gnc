#include "auv_guidance/monotonic_trajectory.hpp"

namespace auv_guidance
{
/**
 * @param start Initial conditions of position, velocity, acceleration, and jerk.
 * @param end Final conditions of position, velocity, acceleration, and jerk
 */
MonotonicTrajectory::MonotonicTrajectory(const Ref<const Vector4d> &start, const Ref<const Vector4d> &end, double maxAccel)
{
    start_ = start;
    end_ = end;
    
    double x0, v0, xf, vf;
    x0 = start_(0);
    v0 = start_(1);
    xf = end_(0);
    vf = end_(1);

    // Add extra monotonic trajectory if initially moving in the opposite direction
    if (x0 < xf) // Moving in the positive direction
    {
        // Jerk must be positive for algorithm to work
        start_(3) = fabs(start_(3));
        end_(3) = fabs(end_(3));

        if (v0 >= 0 && vf >= 0)
        {
            mtbList_.push_back(new MonotonicTrajectoryBase(start_, end_));
        }
        else if (v0 < 0 && vf >= 0) // Initially moving in negative direction
        {
            Vector4d mid = Vector4d::Zero();
            mid(0) = start_(0) - (v0 * v0) / fabs(maxAccel); // Move a little more, come to rest
            mid(1) = 0; // Zero velocity
            mid(2) = 0;
            mid(3) = start_(3);
            mtbList_.push_back(new MonotonicTrajectoryBase(start_, mid));
            mtbList_.push_back(new MonotonicTrajectoryBase(mid, end_));
        }
    }
    else if (xf < x0) // Moving in the negative direction
    {
        // Jerk must be negative for algorithm to work
        start_(3) = -fabs(start_(3));
        end_(3) = -fabs(end_(3));

        if (v0 <= 0 && vf <= 0)
        {
            mtbList_.push_back(new MonotonicTrajectoryBase(start_, end_));
        }
        else if (v0 > 0 && vf <= 0) // Initially moving in positive direction
        {
            Vector4d mid = Vector4d::Zero();
            mid(0) = start_(0) + (v0 * v0) / fabs(maxAccel); // Move a little more, come to rest
            mid(1) = 0; // Zero velocity
            mid(2) = 0;
            mid(3) = start_(3);
            mtbList_.push_back(new MonotonicTrajectoryBase(start_, mid));
            mtbList_.push_back(new MonotonicTrajectoryBase(mid, end_));
        }
    }

    // Append trajectory times to end
    totalTime_ = 0;
    for (int i = 0; i < mtbList_.size(); i++)
    {
        mtbTimes_.push_back(mtbList_.at(i)->getTime());
        totalTime_ += mtbTimes_.at(i);
    }
}

double MonotonicTrajectory::getTime()
{
    return totalTime_;
}

/**
 * @param time Time instance for which to compute the state of the trajectory
 * Compute the state of the trajectory at specified time
 */
Vector3d MonotonicTrajectory::computeState(double time)
{
    Vector3d state = Vector3d::Zero();
    
    if (time <= 0)
    {
        state(0) = start_(0); // Pos
        state(1) = start_(1); // Vel
        state(2) = start_(2); // Accel
        return state;
    }
    else if (time >= totalTime_)
    {
        state(0) = end_(0); // Pos
        state(1) = end_(1); // Vel
        state(2) = end_(2); // Accel
        return state;
    }

    if (time <= mtbTimes_.front())
    {
        state = mtbList_.front()->computeState(time);
        return state;
    }
    else
    {
        double time1 = mtbTimes_.front();
        state = mtbList_.back()->computeState(time - time1);
        return state;
    }
}

double MonotonicTrajectory::getMiddleVelocity()
{
    double tMid = mtbList_.back() / 2;
    

}
} // namespace auv_guidance