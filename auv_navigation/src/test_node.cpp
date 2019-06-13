#include "auv_navigation/test_node.hpp"

namespace Eigen
{
template <typename X, typename BinOp>
struct ScalarBinaryOpTraits<CppAD::AD<X>, X, BinOp>
{
    typedef CppAD::AD<X> ReturnType;
};

template <typename X, typename BinOp>
struct ScalarBinaryOpTraits<X, CppAD::AD<X>, BinOp>
{
    typedef CppAD::AD<X> ReturnType;
};
} // namespace Eigen

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_edkf");
    AUV_GNC::TestNode testNode;
    ros::spin();
}

namespace AUV_GNC
{
void GetRotationYPR2Body(Ref<MatrixXf> R, float yaw, float pitch, float roll)
{
    //Matrix3f R = Matrix3f::Zero();
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
}

TestNode::TestNode() : nh("~")
{
    /*int n = 2;
    int m = 3;
    vector<MatrixXi> mat;
    MatrixXi m1(n,m);
    m1.setZero();
    MatrixXi m2(m,n);
    m2.setOnes();

    mat.push_back(m1);
    mat.push_back(m2);
    cout << "m1: \n" << mat[0] << endl;
    cout << "m2: \n" << mat[1] << endl;

    Matrix3i m3 = Matrix3i::Ones();
    m1.row(0) << 1, 1, 1;
    cout << "m3: \n" << m3 << endl;
    cout << "Replaced row 1 with m3 in m1: \n" << m1 << endl;

    Matrix3Xf posIn = MatrixXf::Zero(3, 1);
    cout << "posIn \n" << posIn << endl;

    Matrix3f test = Matrix3f::Zero();
    GetRotationYPR2Body(test, 0.0, 0.0, 0.0);
    cout << "Rot: \n" << test << endl;

    copy(test);
    cout << "Mat copied: \n" << mat << endl;

    test(1,1) = 5;
    cout << "Rot: \n" << test << endl;
    cout << "Mat copied, test changed: \n" << mat << endl;

    const float var1 = 10.5;
    float var2 = var1 + 2.5;
    cout << "var2: " << endl << var2 << endl;

    int a = 5;
    stringstream ss;
    ss << "The number 'is': " << a;
    ROS_INFO("%s", ss.str().c_str());

    Vector3f v1, v2; 
    v1 << -1, -2, -3;
    v2 << 4, 5, 6;
    Vector3f v3 = v1.array() * v2.array();
    cout << "v1: " << endl << v1 << endl;
    cout << "v2: " << endl << v2 << endl;
    cout << "v1 dot v2 = " << endl << v3 << endl;

    Matrix3f skew;
    Vector3f pqr;
    pqr << 1, 2, 3;
    skew = AUVMathLib::skewSym(pqr);
    cout << "skew 123: " << endl << skew << endl;

    cout << "v1: " << endl << v1 << endl;
    v1 = v1.array().abs();
    cout << "abs of v1: " << endl << v1 << endl;

    int b = 6, m = 3, n = 4;
    try
    {
        stringstream ss;
        ss << "UpdateEKF, Dimension mismatch: Input 'Hnew' of size(" << a << ", " << b << ") does not match expected size(" << m << ", " << n << ")" << endl;
        throw std::runtime_error(ss.str());
        a = 7;
    }
    catch (exception &e)
    {
        cout << e.what();
    }
    ROS_INFO("a = %i", a);*/

    /*int a = 3;
    cout << "sgn(a) = " << Sgn(a) << endl;

    size_t n = 1, m = 2;
    vector<CppAD::AD<double>> X(n), Y(m), A(n);
    X[0] = -4.;
    CppAD::Independent(X);
    vector<double> x(n); // domain space vector
    x[0] = -2;           // argument value for derivative
    Y[0] = -Sgn(x[0]) * X[0] * X[0];
    Y[1] = X[0] * CppAD::exp(X[0]);
    CppAD::ADFun<double> f(X, Y);
    vector<double> jac(m * n);
    jac = f.Jacobian(x); // Jacobian for operation sequence
    cout << "f'(-2),1 = " << jac[0] << endl;
    cout << "f'(-2),2 = " << jac[1] << endl;

    X[0] = -4.;
    CppAD::Independent(X);
    A[0] = X[0] * X[0] * X[0]; // Declare a sub-expression, for modularity
    Y[1] = 5 * A[0] * CppAD::exp(X[0]);
    f.Dependent(X, Y);
    jac = f.Jacobian(x); // Jacobian for operation sequence
    cout << "New f'(-2),1 = " << jac[0] << endl;
    cout << "New f'(-2),2 = " << jac[1] << endl;

    Matrix3d id = Matrix3d::Identity();
    Matrix3d mat;
    mat << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    size_t n = 6, m = 3;
    ADVectorXd X(m), Y(m), B(m);
    vector<double> x(m);
    x[0] = 1;
    x[1] = 6;
    x[2] = 20;

    CppAD::Independent(X);
    Y = X.head<3>().cross(mat * X.head<3>());
    //Y = (2*id).inverse() * B;
    CppAD::ADFun<double> f(X, Y);
    vector<double> jac(m * m);
    jac = f.Jacobian(x);
    cout << "Eigen CppAD jac[0,0] = " << jac[0] << endl;
    cout << "Eigen CppAD jac[0,0] = " << jac[1] << endl;
    cout << "Eigen CppAD jac[0,0] = " << jac[2] << endl;
    cout << "Eigen CppAD jac[0,0] = " << jac[3] << endl;
    cout << "Eigen CppAD jac[0,0] = " << jac[4] << endl;
    cout << "Eigen CppAD jac[0,0] = " << jac[5] << endl;
    cout << "Eigen CppAD jac[0,0] = " << jac[6] << endl;
    cout << "Eigen CppAD jac[0,0] = " << jac[7] << endl;
    cout << "Eigen CppAD jac[0,0] = " << jac[8] << endl;
    
    //cout << "New f'(-2),2 = " << jac[1] << endl;

    VectorXf test(3,1), test1(3,1), test2(3,1);
    test.resize(3,1);
    test1 << 1, 2, 3;
    test2 << 2, 4, 6;
    test = test1.cwiseProduct(test2);
    cout << "test: " << test << endl;*/

    double array[4] = {2, 0, 1, -3};
    double *aPtr = array;
    Eigen::Map<const Eigen::Matrix<double, 4, 1> > arrayEigen(aPtr);
    cout << "Mapped array pointer: " << arrayEigen << endl;

    //Eigen::Map<const Eigen::Quaternion<double> > quat2(aPtr);
    Eigen::Quaterniond quat1(aPtr);
    Eigen::Quaterniond quat2(2, 0, 1, -3);
    Vector3d vec1;
    vec1 << 1, 2, 3;
    cout << "Mapped quaternion from array pointer: " << quat1.w() << endl
         << quat1.vec() << endl;
    cout << "Mapped quaternion2 from array: " << quat2.w() << endl
         << quat2.vec() << endl;
    cout << "Quaternion2 * Vector3d: " << quat2 * vec1 << endl;

    float w1, x1, y1, z1, w2, x2, y2, z2;
    nh.getParam("w1", w1);
    nh.getParam("x1", x1);
    nh.getParam("y1", y1);
    nh.getParam("z1", z1);
    nh.getParam("w2", w2);
    nh.getParam("x2", x2);
    nh.getParam("y2", y2);
    nh.getParam("z2", z2);
    Eigen::Quaternionf quat3(w1, x1, y1, z1);
    Vector3f vWorld;
    vWorld << 0.707, 0.707, 0;
    cout << "Eigen directly, Body roll 90 deg, rotated vector: " << endl
         << quat3 * vWorld << endl;
    cout << "Eigen toRotMat, Body roll 90 deg, rotated vector: " << endl
         << quat3.toRotationMatrix() * vWorld << endl;

    Eigen::Matrix3f Rquat;
    Eigen::Matrix<float, 4, 1> q;
    q << w1, x1, y1, z1;
    Rquat(0, 0) = q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3);
    Rquat(1, 1) = q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3);
    Rquat(2, 2) = q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
    Rquat(0, 1) = 2 * q(1) * q(2) + 2 * q(0) * q(3);
    Rquat(1, 0) = 2 * q(1) * q(2) - 2 * q(0) * q(3);
    Rquat(0, 2) = 2 * q(1) * q(3) - 2 * q(0) * q(2);
    Rquat(2, 0) = 2 * q(1) * q(3) + 2 * q(0) * q(2);
    Rquat(1, 2) = 2 * q(2) * q(3) + 2 * q(0) * q(1);
    Rquat(2, 1) = 2 * q(2) * q(3) - 2 * q(0) * q(1);
    cout << "Mine, Body roll 90 deg, rotated vector: " << endl
         << Rquat * vWorld << endl;
    // Using Eigen::Quaternion: quaternion * vector = rotates vector by the axis-angle specified, expressed in original frame
    // So: B-frame vector = quaternion.conjugate() * I-frame vector
    // So: I-frame vector = quaternion * B-frame vector
    // The correct format:
    // I-frame --(q)--> B-frame ==> Rquat (rotation matrix)
    // B-frame vector = q * v * q^(-1) = Rquat * I-frame vector
    // I-frame vector = q^(-1) * v * q = Rquat^T * B-frame vector

    // Quaternion * Quaternion
    Eigen::Quaternionf q1(w1, x1, y1, z1);
    Eigen::Quaternionf q2(w2, x2, y2, z2);
    Eigen::Quaternionf qdiffI = q2 * (q1.conjugate());
    cout << "Eigen: q2 * (q1.conj) = " << endl
         << qdiffI.w() << endl
         << qdiffI.vec() << endl;
    Eigen::Quaternionf q1q2 = q1 * q2;
    cout << "Eigen: q1 * q2 = " << endl
         << q1q2.w() << endl
         << q1q2.vec() << endl;

    Vector4f q1V, q2V;
    q1V << w1, x1, y1, z1;
    q2V << w2, x2, y2, z2;
    Vector4f qFV = multiplyQuaternions(q2V, q1V);
    cout << "Mine: q1 * q2 = " << endl
         << qFV << endl;
    // Quaternion Compositions: q = q1 * q2
    // The correct format: I-frame --(q2)--> Frame 2 --(q1)--> B-frame (quaternions are left multiplied)
    // Eigen does local frame composition, as in q2 is applied FROM q1
}

void TestNode::copy(const Ref<const MatrixXf> &m)
{
    mat = m;
}

// Multiplies quaternions wrt the WORLD frame
Vector4f TestNode::multiplyQuaternions(Vector4f q1, Vector4f q2)
{
    return (TestNode::quaternionMatrix(q1) * q2);
}

Matrix4f TestNode::quaternionMatrix(Vector4f q)
{
    Matrix4f Q, diag;
    Q.setZero(), diag.setIdentity();
    diag = diag * q(0);
    Q.block<3, 3>(1, 1) = -AUVMathLib::skewSym(q.tail<3>());
    Q.block<1, 3>(0, 1) = -q.tail<3>().transpose();
    Q.block<3, 1>(1, 0) = q.tail<3>();
    return (Q + diag);
}
} // namespace AUV_GNC