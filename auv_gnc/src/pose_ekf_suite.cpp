#include "riptide_gnc/pose_ekf_suite.h"

// dataAvail = matrix indicating which measurements are provided to this EKF (1 indicates provided, 0 otherwise)
//             Row 0 = X; Row 1 = Y; Row 2 = Z
//             Col 0 = abs. pos; Col 1 = body-frame vel; Col 2 = body-frame accel
// Rpos, Rvel, and Raccel contain the main-diagonal elements for the sensor noise covariance matrices
// Qx, Qy, and Qz are the process-noise coviariance matrices for each axis
PoseEKFSuite::PoseEKFSuite(Matrix3i dataAvail, Vector3f Rpos, Vector3f Rvel, Vector3f Raccel, Matrix39f Q)
{
    posMask = dataAvail.col(0);      // Available absolute position data
    bfData = dataAvail.rightCols(2); // Available body-frame data

    // Deterine which KFs need to be created (X, Y, and/or Z)
    for (int i = 0; i < 3; i++) // Check for absolute position measurements
        needKF[i] = (bool)posMask(i);
    if (bfData.sum() > 0) // Check for relative measurements
        for (int i = 0; i < 3; i++)
            needKF[i] = true;

    // Create required KFs and initiailize w/A, H, Q, and R matrices
    // The actual A and H matrices will be calculated by the Jacobian
    Matrix3f A = Matrix3f::Zero();              // Create A matrix
    MatrixXf Rbf(bfData.rows(), bfData.cols()); // Will make a 3x2 matrix
    Rbf << Rvel, Raccel;                        // Concatenate all body-frame R-vectors
    for (int axis = 0; axis < 3; axis++)
    {
        int numMsmts = 0, Rindex = 0;
        KFindeces[axis] = 0; // Initialize to 0

        if (needKF[axis])
        {
            numMsmts = posMask(axis) + bfData.sum();
            MatrixXf H(numMsmts, 3); // Create H matrix
            H.setZero();

            MatrixXf R(numMsmts, numMsmts); // Create R matrix
            R.setZero();
            if (posMask(axis))
                R(Rindex, Rindex++) = Rpos(axis);
            for (int k = 0; k < bfData.cols(); k++)
                for (int j = 0; j < bfData.rows(); j++)
                    if (bfData(j, k)) // All body-frame measurements are needed
                        R(Rindex, Rindex++) = Rbf(j, k);

            // Add new KF to KFSuite
            KFSuite.push_back(new KalmanFilter(A, H, Q.block(3, 3, 0, axis * 3), R));
            KFindeces[axis] = KFSuite.size() - 1;
        }
    }
}

// Update Pose EKF Suite
// Xpredict = concatenation of time-predictions for each axis
//            Row 0 = abs. pos, Row 1 = abs. vel, Row 2 = abs. accel
//            Col 0 = X-axis KF, Col 1 = Y-axis KF, Col 2 = Z-axis KF
// attitude = roll, pitch, yaw [rad]
// Z = matrix of measurements (same order as for dataAvail in constructor)
// Anew = concatenation of each KF's A matrix (Ax, Ay, Az)
Matrix43f PoseEKFSuite::UpdatePoseEKFSuite(Matrix3f Xpredict, Vector3f attitude, Matrix3f Z, Matrix39f Anew)
{
    Matrix3f X = Matrix3f::Zero();
    Matrix<float, 1, 3> newDataAvail;
    newDataAvail.setZero();
    for (int axis = 0; axis < 3; axis++)
    {
        if (needKF[axis]) // Update each KF (X, Y, and/or Z) as needed
        {
            newDataAvail(axis) = 1; // 1 indicates new data available for KF
            int numMsmts = posMask(axis) + bfData.sum();
            MatrixXf Hnew(numMsmts, 3);
            MatrixXf Znew(numMsmts, 1);
            Matrix3f R_w2b = GetRotationRPY2Body(attitude(0), attitude(1), attitude(2));
            Hnew.setZero();
            Znew.setZero();
            int Hrow = 0;

            // Calculate Jacobian for H matrix
            if (posMask(axis))
            {
                Hnew(Hrow, 0) = 1;
                Znew(Hrow++) = Z(axis, 0);
            }
            for (int k = 0; k < bfData.cols(); k++)
            {
                for (int j = 0; j < bfData.rows(); j++)
                {
                    if (bfData(j, k))
                    {
                        Hnew(Hrow, k + 1) = R_w2b(j, axis);
                        Znew(Hrow++) = Z(j, k + 1);
                    }
                }
            }

            // Update KF
            X.col(axis) = KFSuite[KFindeces[axis]]->UpdateKFOverride(Xpredict.col(axis), Znew, Anew.block(3, 3, 0, axis * 3), Hnew);
        }
    }

    Matrix43f output;
    output << newDataAvail, X;
    return output;
}

// Get rotation matrix from world-frame to body-frame using Euler Angles (roll, pitch, yaw)
Matrix3f PoseEKFSuite::GetRotationRPY2Body(float roll, float pitch, float yaw)
{
    Matrix3f R = Matrix3f::Zero();
    float s_phi = sin(roll);
    float c_phi = cos(roll);
    float s_theta = sin(pitch);
    float c_theta = cos(pitch);
    float s_psi = sin(yaw);
    float c_psi = cos(yaw);

    R(0, 0) = c_theta * c_psi;
    R(0, 1) = c_theta * s_psi;
    R(0, 2) = -s_theta;
    R(1, 0) = s_phi * s_theta * c_psi - c_phi * s_psi;
    R(1, 1) = s_phi * s_theta * s_psi + c_phi * c_psi;
    R(1, 2) = s_phi * c_theta;
    R(2, 0) = c_phi * s_theta * c_psi + s_phi * s_psi;
    R(2, 1) = c_phi * s_theta * s_psi - s_phi * c_psi;
    R(2, 2) = c_phi * c_theta;

    return R;
}