#include "auv_guidance/test_node.hpp"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_edkf");
    AUVGuidance::TestNode testNode;
    ros::spin();
}

namespace AUVGuidance
{

TestNode::TestNode() : nh("~")
{

    // Ceres stuff min jerk time solver /////////////////////
    ceres::Problem problemMinJerkTime_;
    ceres::Solver::Options optionsMinJerkTime_;
    ceres::Solver::Summary summaryMinJerkTime_;

    double x0, v0, a0, j0, xf, vf, af, jf;
    nh.getParam("x0", x0);
    nh.getParam("v0", v0);
    nh.getParam("a0", a0);
    nh.getParam("j0", j0);
    nh.getParam("xf", xf);
    nh.getParam("vf", vf);
    nh.getParam("af", af);
    nh.getParam("jf", jf);

    Vector4d start = Vector4d::Zero();
    Vector4d end = Vector4d::Zero();
    start(0) = x0;
    start(1) = v0;
    start(2) = a0;
    start(3) = j0;
    end(0) = xf;
    end(1) = vf;
    end(2) = af;
    end(3) = jf;

    double minTime_ = 0;
    problemMinJerkTime_.AddResidualBlock(new ceres::AutoDiffCostFunction<MonotonicVelocityTimeSolver, 1, 1>(new MonotonicVelocityTimeSolver(start, end)), NULL, &minTime_);
    problemMinJerkTime_.SetParameterLowerBound(&minTime_, 0, 0.0);
    optionsMinJerkTime_.max_num_iterations = 100;
    optionsMinJerkTime_.linear_solver_type = ceres::DENSE_QR;
    ceres::Solve(optionsMinJerkTime_, &problemMinJerkTime_, &summaryMinJerkTime_);
    cout << "Min Jerk time: " << minTime_ << endl;
}
} // namespace AUVGuidance