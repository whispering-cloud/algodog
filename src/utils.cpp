#include "utils.h"

Mat3 rpy2Matrix(Vec3 eulerAngle){
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle(2),Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle(1),Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle(0),Eigen::Vector3d::UnitZ())); 
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix=yawAngle*pitchAngle*rollAngle;
    return rotation_matrix;
}

Mat3 crossMultiplyMat(Vec3 vec){
    Mat3 res;
    res(0,0) = 0;
    res(1,1) = 0;
    res(2,2) = 0;
    res(0,1) = -vec(2);
    res(1,0) = vec(2);
    res(0,2) = vec(1);
    res(2,0) = -vec(1);
    res(1,2) = -vec(0);
    res(2,1) = vec(0);
    return res;
}
 
 Mat3 inverseMat(Mat3 mat){
    return mat.inverse();
 }

 Mat4x3 foldVector(Vec12 vec){
    Mat4x3 res;
    for (int i=0;i<4;i++){
        res.col(i) = vec.segment(3*i,3);
    }
    return res;
 }

Mat3 quat2Matrix(Vec4 quat){
    Mat3 res;
    double e0 = quat(0);
    double e1 = quat(1);
    double e2 = quat(2);
    double e3 = quat(3);
    res << 1 - 2 * (e2 * e2 + e3 * e3), 2 * (e1 * e2 - e0 * e3),
            2 * (e1 * e3 + e0 * e2), 2 * (e1 * e2 + e0 * e3),
            1 - 2 * (e1 * e1 + e3 * e3), 2 * (e2 * e3 - e0 * e1),
            2 * (e1 * e3 - e0 * e2), 2 * (e2 * e3 + e0 * e1),
            1 - 2 * (e1 * e1 + e2 * e2);
    return res;

}