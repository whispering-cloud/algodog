#ifndef __THIRDPARTIES_H__
#define __THIRDPARTIES_H__

#include <eigen3/Eigen/Dense>


#define I3 Eigen::MatrixXd::Identity(3, 3)
#define pi 3.1415926
#define ll long long

using Vec2 = typename Eigen::Matrix<double, 2, 1>;
using Vec3 = typename Eigen::Matrix<double, 3, 1>;
using Vec4 = typename Eigen::Matrix<double, 4, 1>;
using Vec6 = typename Eigen::Matrix<double, 6, 1>;
using Vec12 = typename Eigen::Matrix<double, 12, 1>;

using Mat3 = typename Eigen::Matrix<double, 3, 3>;
using Mat4 = typename Eigen::Matrix<double, 4, 4>;
using Mat6 = typename Eigen::Matrix<double, 6, 6>;
using Mat12 = typename Eigen::Matrix<double, 12, 12>;

using Mat4x3 = typename Eigen::Matrix<double, 4, 3>;
using Mat3x4 = typename Eigen::Matrix<double, 3, 4>;


#endif