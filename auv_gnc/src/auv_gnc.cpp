#include "auv_gnc/auv_trajectory.hpp"

namespace AUV_GNC
{
AUVTrajectory::AUVTrajectory()
{
    //#include <ct/optcon/optcon.h>
    //ct::optcon::LQR<state_dim, control_dim> lqrSolver;
    // use size_t for state_dim and control_dim
    //lqrSolver.compute(Q, R, A, B, K);
}

void AUVTrajectory::setWaypointTrajectory()
{
}

Vector12d computeState(double time)
{
}
} // namespace AUVGuidance
