#include "auv_guidance/waypoint_trajectory.hpp"

namespace AUVGuidance
{
WaypointTrajectory::WaypointTrajectory(Waypoint *start, Waypoint *end)
{
    waypoints_.clear();
    waypoints_.push_back(start);
    waypoints_.push_back(end);
    travelTime_ = 0;
    WaypointTrajectory::initNextTrajectory();
}

void WaypointTrajectory::addWaypoint(Waypoint *waypoint)
{
    waypoints_.push_back(waypoint);
}

void WaypointTrajectory::initNextTrajectory()
{
    if (waypoints_.size() > 0)
    {
        lastWaypoint_ = waypoints_.at(0);
        waypoints_.erase(waypoints_.begin()); // Erase first element
    }
}

Vector12d WaypointTrajectory::computeState(double time)
{

}
} // namespace AUVGuidance