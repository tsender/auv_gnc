#include "riptide_gnc/simple_motion_planner.h"

/**
 * @param startPos Starting position
 * @param nominalSpeed Desired cruise speed
 */
DistanceMotionPlanner::DistanceMotionPlanner(float distance, float nominalSpeed, float accel = 0.0, int seq = SEQ_NONE)
{
    distance_ = distance;
    if (nominalSpeed <= 0) // TODO: May need to be strictly less than 0, will see later
        cruiseSpeed_ = DEFAULT_SPEED;
    else
        cruiseSpeed_ = abs(nominalSpeed);

    if (accel == 0 || seq == SEQ_NONE) // Default case
    {
        accelerate_ = false;
        acceleration_ = 0;
        accelSeq_ = seq;
    }
    else
    {
        if (seq >= SEQ_NONE && seq <= SEQ_BOTH) // Valid sequence
        {
            accelerate_ = true;
            acceleration_ = accel;
            accelSeq_ = seq;
        }
        else
        {
            accelerate_ = false; // Default to constant speed (accel = 0)
            acceleration_ = 0;
            accelSeq_ = SEQ_NONE;
        }
    }

    // Initialize to zero
    t1_ = 0, t2_ = 0, tMid_ = 0, tFinal_ = 0;
    cruiseDuration_ = 0;
    velocity0_ = 0, acceleration1_ = 0, acceleration2_ = 0;

    DistanceMotionPlanner::initMotionPlanner();
}

// Initialize Motion Planner - calculate travel parameters
void DistanceMotionPlanner::initMotionPlanner()
{
    // Exit if distance is 0
    if (distance_ == 0)
        return;

    if (!accelerate_) // Constant speed
    {
        tEnd_ = distance_ / cruiseSpeed_;
        cruiseDuration_ = tEnd_;
        t2_ = tEnd_;
    }
    else // Will be accelerating for certain portions of travel
    {
        float accelDuration = cruiseSpeed_ / acceleration_;           // Assuming accelerating from rest
        float accelDist = 0.5 * pow(cruiseSpeed_, 2) / acceleration_; // Or 0.5*acceleration*t^2

        if (accelSeq_ == SEQ_START)
        {
            if (accelDist <= distance_) // Possible: Will be traveling at cruiseSpeed at destination
            {
                cruiseDuration_ = (distance_ - accelDist) / cruiseSpeed_;
                t1_ = accelDuration;
                tEnd_ = t1_ + cruiseDuration_;
                t2_ = tEnd_;
                acceleration1_ = acceleration_;
            }
            else // Impossible: Will be traveling slower than cruiseSpeed at destination
            {
                stringstream ss;
                ss << "DistanceMotionPlanner: SEQ_START - Will be traveling slower than cruiseSpeed at destination. Decrease speed or increase acceleration." << endl;
                throw std::runtime_error(ss.str());
            }
        }
        else if (accelSeq_ == SEQ_END)
        {
            if (accelDist <= distance_) // Possible: Will be able to accelerate from cruiseSpeed to rest
            {
                cruiseDuration_ = (distance_ - accelDist) / cruiseSpeed_;
                t1_ = 0;
                t2_ = cruiseDuration_;
                tEnd_ = t2_ + accelDuration;
                velocity0_ = cruiseSpeed_;
                acceleration2_ = -acceleration_;
            }
            else // Impossible: Will have non-zero speed when you reach the destination
            {
                stringstream ss;
                ss << "DistanceMotionPlanner: SEQ_END -  Will have non-zero speed at destination. Decrease speed or increase acceleration." << endl;
                throw std::runtime_error(ss.str());
            }
        }
        else if (accelSeq_ == SEQ_BOTH)
        {
            if (2 * accelDist <= distance_) // Will reach cruise speed for some duration >= 0
            {
                cruiseDuration_ = (distance_ - 2 * accelDist) / cruiseSpeed_;
                t1_ = accelDuration;
                t2_ = accelDuration + cruiseDuration_;
                tEnd_ = t2_ + accelDuration;
                acceleration1_ = acceleration_;
                acceleration2_ = -acceleration_;
            }
            else // Will not reach cruiseSpeed during travel
            {
                tMid_ = sqrt(distance_ / acceleration_);
                t1_ = tMid_;
                t2_ = tMid_;
                tEnd_ = tMid_ * 2;
            }
        }
    }
}

float DistanceMotionPlanner::getTravelTime()
{
    return tEnd_;
}

/**
 * @param t Current time for state to be computed (state = [position, speed])
 **/
Vector2f DistanceMotionPlanner::computeState(float t)
{
    Vector2f state; // [position; speed]
    state.setZero();

    if (!accelerate_) // Constant Speed
    {
        if (t >= 0 && t <= tEnd) // Valid time instance
        {
            state(0) = cruiseSpeed_ * t;
            state(1) = cruiseSpeed_;
        }
        else if (t > tEnd) // Do not know anything about times outside this interval
        {
            state(0) = distance_;
            state(1) = cruiseSpeed_;
        }
        else if (t < 0)
        {
            state(0) = 0;
            state(1) = cruiseSpeed_;
        }
    }
    else if (accelerate_)
    {
        if (t >= 0 && t <= tEnd) // Valid time instance
        {
            float time1 = 0, time2 = 0, time3 = 0;

            // t in [t, t1] range - accelarate from rest to cruiseSpeed
            time1 = (t <= t1) ? t : t1;
            state(0) = start + vel0 * time1 + 0.5 * accel1 * pow(time1, 2);
            state(1) = accel1 * time1;

            // t in (t1, t2] range - move at cruiseSpeed
            time2 = (t > t1 && t <= t2) ? t - t1 : 0;
            time2 = (t > t2) ? t2 - t1 : time2;
            state(0) = state(0) + cruiseSpeed * time2;

            // t in (t2, tEnd] range - accelerate from cruiseSPeed to rest
        }
    }
}