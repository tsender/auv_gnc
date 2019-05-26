#ifndef POSE_EKF_SUITE
#define POSE_EKF_SUITE

#include "riptide_gnc/kalman_filter.h"
#include "eigen3/Eigen/Dense"
#include "math.h"

using namespace Eigen;
using namespace std;
typedef Matrix<float, 3, 9> Matrix39f;
typedef Matrix<float, 4, 3> Matrix43f;

// Pose Extended Kalman Filter (EKF) Suite
// This class is deigned to work with the Pose EDKF class, as the Pose EDKF class instantiates
// numerous Pose EKF Suites to better estimate the vehicle's state.
// Each Pose EKF Suite is initialized to handle a specific collection of measurements, indicated
// in the dataAvail parameter. The Pose EKF Suite uses this parameter to pass the appropriate
// information to each Kalman Filter (X, Y, and Z axes).
// This class is written mostly in a generic sense to handle all possible combinations of information.
class PoseEKFSuite
{
private:
  vector<KalmanFilter*> KFSuite; // std::vector of pointers to KFs
  Vector3i posMask;           // Available absolute position data
  Matrix<int, 3, 2> bfData; // Available body-frame data
  bool needKF[3];             // Indicate if there is an X, Y, and/or Z KF
  int KFindeces[3];           // Contains the index within KFSuite for the X, Y, and/or Z KF

public:
  PoseEKFSuite(Matrix3i dataAvail, Vector3f Rpos, Vector3f Rvel, Vector3f Raccel, Matrix39f Q);
  Matrix43f UpdatePoseEKFSuite(Matrix3f Xpredict, Vector3f attitude, Matrix3f Z, Matrix39f Anew);
  Matrix3f GetRotationRPY2Body(float roll, float pitch, float yaw);
};

#endif