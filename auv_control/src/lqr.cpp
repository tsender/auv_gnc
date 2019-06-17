#include "auv_gnc/auv_gnc.hpp"

namespace AUV_GNC
{
LQR::LQR()
{
    
    //ct::optcon::LQR<state_dim, control_dim> lqrSolver;
    // use size_t for state_dim and control_dim
    //lqrSolver.compute(Q, R, A, B, K);
}

void LQR::computeGain()
{
}

Vector12d computeState(double time)
{
}
} // namespace AUVGuidance
