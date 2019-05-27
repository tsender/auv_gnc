#include "auv_gnc/distance_motion_planner.hpp"

namespace AUV_GNC
{
/**
 * @param startPos Starting position
 * @param nominalSpeed Desired cruise speed
 */
DistanceMotionPlanner::DistanceMotionPlanner(float distance, float nominalSpeed, float accel, int seq)
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
    t1_ = 0, t2_ = 0, tMid_ = 0, tEnd_ = 0;
    cruiseDuration_ = 0;
    initialSpeed_ = 0, maxSpeed_ = 0, finalSpeed_ = 0;

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
        initialSpeed_ = cruiseSpeed_;
        maxSpeed_ = cruiseSpeed_;
        finalSpeed_ = cruiseSpeed_;
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
                finalSpeed_ = cruiseSpeed_;
            }
            else // Impossible: Will be traveling slower than cruiseSpeed at destination
            {
                std::stringstream ss;
                ss << "DistanceMotionPlanner: SEQ_START - Will be traveling slower than cruiseSpeed at destination. Decrease speed or increase acceleration." << std::endl;
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
                initialSpeed_ = cruiseSpeed_;
            }
            else // Impossible: Will have non-zero speed when you reach the destination
            {
                std::stringstream ss;
                ss << "DistanceMotionPlanner: SEQ_END -  Will have non-zero speed at destination. Decrease speed or increase acceleration." << std::endl;
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
            }
            else // Will not reach cruiseSpeed during travel
            {
                tMid_ = sqrt(distance_ / acceleration_);
                t1_ = tMid_;
                t2_ = tMid_;
                tEnd_ = tMid_ * 2;
            }
        }
        maxSpeed_ = acceleration_ * t1_;
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
        if (t >= 0 && t <= tEnd_) // Valid time instance
        {
            state(0) = cruiseSpeed_ * t;
            state(1) = cruiseSpeed_;
        }
        else if (t < 0)
        {
            state(0) = 0;
            state(1) = cruiseSpeed_;
        }
        else if (t > tEnd_) 
        {
            state(0) = distance_;
            state(1) = cruiseSpeed_;
        }
    }
    else if (accelerate_)
    {
        if (t >= 0 && t <= tEnd_) // Valid time instance
        {
            float time1 = 0, time2 = 0, time3 = 0;

            // time1 in [t, t1] range - accelarate from rest to cruiseSpeed
            time1 = (t <= t1_) ? t : t1_;
            state(0) = initialSpeed_ * time1 + 0.5 * acceleration_ * pow(time1, 2);
            state(1) = acceleration_ * time1;

            // time2 in (t1, t2] range - traveling at cruiseSpeed
            time2 = (t > t1_ && t <= t2_) ? (t - t1_) : 0;
            time2 = (t > t2_) ? (t2_ - t1_) : time2;
            if (time2 > 0)
                state(0) = state(0) + cruiseSpeed_ * time2; // state(1) remains as is

            // time3 in (t2, tEnd] range - accelerate from cruiseSPeed to rest
            time3 = (t > t2_) ? (t - t2_) : 0;
            if (time3 > 0)
            {
                state(0) = state(0) + maxSpeed_ * time3 - 0.5 * acceleration_ * pow(time3, 2);
                state(1) = maxSpeed_ - acceleration_ * time3;
            }
        }
        else if (t < 0)
        {
            state(0) = 0;
            state(1) = initialSpeed_;
        }
        else if (t > tEnd_)
        {
            state(0) = distance_;
            state(1) = finalSpeed_;
        }
    }
    return state;
}
}