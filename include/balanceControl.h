#ifndef __BALANCECONTROL_H__
#define __BALANCECONTROL_H__

#include "thirdparties.h"
#include "gait.h"
#include "legKinetic.h"
class Balancer{
public:
    Balancer(){};
    Balancer(Vec3 l, Vec12 vecW, Vec12 vecU, Vec6 vecS, double alpha, double beta, double fric);
    ~Balancer();
    Vec12 solveKinetic();
    Mat3 Rs_b;
    Leg* legs[4];
    FSM* gt;
    Vec3 vecAcc;
    Vec3 omegaAcc;
    double configAlpha, configBeta, configFric;

private:
    Vec12 prevSupportF;
    Vec3 vecG;
    Vec6 vecb_d;
    Mat12 matW,matU;
    Mat6 matS;
    Eigen::Matrix<double, 5, 3> fricMat;
};


#endif