#include "legKinetic.h"
#include "moduleIO.h"
#include <cmath>
#include <iostream>

Vec3 Leg::forwardKinetic(Vec3 theta){
    double s1 = std::sin(theta(0));
    double s2 = std::sin(theta(1));
    double s3 = std::sin(theta(2));
    double c1 = std::cos(theta(0));
    double c2 = std::cos(theta(1));
    double c3 = std::cos(theta(2));
    double s23 = s2*c3+s3*c2;
    double c23 = c2*c3-s2*s3;
    Vec3 p;
    p(0) = lx(2)*s23+lx(1)*s2;
    p(1) = -lx(2)*s1*c23+lx(0)*c1-lx(1)*s1*c2;
    p(2) = lx(2)*c1*c23 + lx(0)*s1+lx(1)*c1*c2;
    return p;
}

Vec3 Leg::backwardKinetic(Vec3 p){
    Vec3 q;
    // for theta1
    double tmpl = std::sqrt(p(1)*p(1) + p(2)*p(2) - lx(0)*lx(0));
    q(0) = std::atan2(tmpl * p(1) + lx(0) * p(2), lx(0) * p(1) - tmpl * p(2));
    // for theta3
    double tmpq = (lx(1)*lx(1) + lx(2)*lx(2) - tmpl - p(0)*p(0)) / (2*lx(1)*lx(2));
    q(2) = -pi + std::acos(tmpq);
    // for theta2
    double a1 = p(1) * std::sin(q(0)) - p(2) * std::cos(q(0));
    double a2 = p(0);
    double m1 = lx(2) * std::sin(q(2));
    double m2 = lx(2) * std::cos(q(2)) + lx(1);
    q(1) = std::atan2(a1*m1+a2*m2, a2*m1-a1*m2); 
    return q;
}

Mat3 Leg::getJacobi(Vec3 theta){
    double s1 = std::sin(theta(0));
    double s2 = std::sin(theta(1));
    double s3 = std::sin(theta(2));
    double c1 = std::cos(theta(0));
    double c2 = std::cos(theta(1));
    double c3 = std::cos(theta(2));
    double s23 = s2*c3+s3*c2;
    double c23 = c2*c3-s2*s3;
    Mat3 res;
    res(0, 0) = 0;
    res(1, 0) = -lx(2) * c1 * c23 - lx(1) * c1 * c2 - lx(0) * s1;
    res(2, 0) = -lx(2) * s1 * c23 - lx(1) * c2 * s1 + lx(0) * c1;
    res(0, 1) = lx(2) * c23 + lx(1) * c2;
    res(1, 1) = lx(2) * s1 * s23 + lx(1) * s1 * s2;
    res(2, 1) = -lx(2) * c1 * s23 - lx(1) * c1 * s2;
    res(0, 2) = lx(2) * c23;
    res(1, 2) = lx(2) * s1 * s23;
    res(2, 2) = -lx(2) * c1 * s23;
    return res;
}

Vec3 Leg::deriveBackwardKinetic(Vec3 vel, Vec3 theta){
    return getJacobi(theta).inverse() * vel;
}

Vec3 Leg::getPb_i(){
    Vec3 offset;
    offset.setZero();
    offset<< ((id & 2)?-1:1) * fl_x, ((id & 1)?-1:1) * fl_y, 0;

    return offset + forwardKinetic(theta);
}

Leg::Leg(int index, Vec3 length){
    id = index;
    lx(0) = length(0) * side();
    lx(1) = -length(1);
    lx(2) = -length(2);
}