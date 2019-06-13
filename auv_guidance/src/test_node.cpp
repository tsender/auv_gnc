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
    double minTime_ = 0;

    double x0, v0, a0, xf, vf, af;
    nh.getParam("x0", x0);
    nh.getParam("v0", v0);
    nh.getParam("a0", a0);
    nh.getParam("xf", xf);
    nh.getParam("vf", vf);
    nh.getParam("af", af);

    Vector3d start = Vector3d::Zero();
    Vector3d end = Vector3d::Zero();
    start(0) = x0;
    start(1) = v0;
    start(2) = a0;
    end(0) = xf;
    end(1) = vf;
    end(2) = af;

    problemMinJerkTime_.AddResidualBlock(new ceres::AutoDiffCostFunction<MinJerkTimeSolver, 1, 1>(new MinJerkTimeSolver(start, end)), NULL, &minTime_);
    optionsMinJerkTime_.max_num_iterations = 100;
    optionsMinJerkTime_.linear_solver_type = ceres::DENSE_QR;
    ceres::Solve(optionsMinJerkTime_, &problemMinJerkTime_, &summaryMinJerkTime_);
    cout << "Min Jerk time: " << minTime_ << endl;
}
} // namespace AUVGuidance