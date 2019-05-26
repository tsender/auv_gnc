#ifndef LINEAR_MOTION_EKF_SUITE
#define LINEAR_MOTION_EKF_SUITE

#include "riptide_gnc/kalman_filter.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/StdVector"
#include "math.h"

using namespace Eigen;
using namespace std;
typedef Matrix<int, 3, Dynamic> Matrix3Xi;
typedef Matrix<float, 3, Dynamic> Matrix3Xf;
typedef Matrix<float, Dynamic, 3> MatrixX3f;

// Linear Motion Extended Kalman Filter (EKF) Suite

// This class is designed to work with the Pose EDKF class, as the Pose EDKF class instantiates
// numerous Linear Motion EKF Suites to better estimate the vehicle's linear state.
// Each Linear Motion EKF Suite is initialized to handle a specific collection of measurements,
// indicated in the posIn, velIn, and accelIn parameters. The Pose EKF Suite uses this parameter to
// pass the appropriate information to each Kalman Filter (X, Y, and Z axes).
// This class is written mostly in a generic sense to handle all possible combinations of information.
class LinearMotionEKFSuite
{
private:
  vector<KalmanFilter *> KFSuite; // std::vector of pointers to KFs
  Vector3i states;                // Indicates which states are being tracked (pos, vel, and/or accel)
  int n;                          // Number of States
  Matrix3Xi posMask;              // Available position data
  Matrix3Xi velMask;              // Available velocity data
  Matrix3Xi accelMask;            // Available acceleration data
  int colsPos, colsVel, colsAccel;
  int activeKFPerAxis[3];           // Number of active KFs per axis
  vector<vector<int> > msmtsPerAxis; // Row -> axis; Col -> num msmts per KF w/in the axis
  vector<vector<bool> > skipKF;      // Row -> axis; Col -> skip KF w/in the axis
  int maxMsmtsPerAxis[3];

public:
  LinearMotionEKFSuite(Vector3i kf_states, Matrix3Xi posIn, Matrix3Xi velIn, Matrix3Xi accelIn,
                       Matrix3Xf Rpos, Matrix3Xf Rvel, Matrix3Xf Raccel, MatrixXf Q);
  MatrixX3f UpdateEKFSuite(MatrixX3f Xpredict, Matrix3Xf Zpos, Matrix3Xf Zvel, Matrix3Xf Zaccel, MatrixXf Anew);
};

#endif