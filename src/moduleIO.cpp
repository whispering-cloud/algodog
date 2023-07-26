#include "moduleIO.h"
#include "utils.h"
#include "balanceControl.h"
#include "inverseKinetic.h"
#include <iostream>

Chronostasis chron;
Mat3 INTERTIA;
Balancer *dogrun;
Vec3 Pg_b;
KineticControl taucalc;
double mass;

GaitState_Stand* STAND;
GaitState_Trott* TROTT;

extern "C" int init(initParams param){
    chron.init();
    double intertia[9] = {param.i00,param.i01,param.i02,param.i10,param.i11,param.i12,param.i20,param.i21,param.i22};
    INTERTIA = Mat3(intertia);
    Vec3 length;
    Vec12 vecW, vecU;
    Vec6 vecS;
    vecW << param.w0, param.w1, param.w2, param.w3, param.w4, param.w5, param.w6, param.w7, param.w8, param.w9, param.w10, param.w11;
    vecU << param.u0, param.u1, param.u2, param.u3, param.u4, param.u5, param.u6, param.u7, param.u8, param.u9, param.u10, param.u11;
    vecS << param.s0, param.s1, param.s2, param.s3, param.s4, param.s5;

    Vec4 trt;
    trt << 0, 0.5, 0.5, 0;
    STAND = new GaitState_Stand(500000,1,Vec4::Zero());
    TROTT = new GaitState_Trott(500000,0.6,trt);

    
    length << param.l1, param.l2, param.l3;
    dogrun = new Balancer(length, vecW, vecU, vecS, param.alpha, param.beta, param.fric);
    taucalc = KineticControl();
    mass = param.m;
    Pg_b(0) = param.pg_b0;
    Pg_b(1) = param.pg_b1;
    Pg_b(2) = param.pg_b2;
    return 0;
}


extern "C" updateResult update(updateParams param){
    updateResult res;
    dogrun->vecAcc(0) = param.a00;
    dogrun->vecAcc(1) = param.a01;
    dogrun->vecAcc(2) = param.a02;
    dogrun->omegaAcc(0) = param.oa0;
    dogrun->omegaAcc(1) = param.oa1;
    dogrun->omegaAcc(2) = param.oa2;
    //dogrun->Rs_b = rpy2Matrix(Vec3({param.y,param.p,param.r}));
    dogrun->Rs_b = quat2Matrix(Vec4({param.quat0, param.quat1, param.quat2, param.quat3}));
    dogrun->legs[0]->theta(0) = param.t00;
    dogrun->legs[0]->theta(1) = param.t01;
    dogrun->legs[0]->theta(2) = param.t02;
    dogrun->legs[1]->theta(0) = param.t10;
    dogrun->legs[1]->theta(1) = param.t11;
    dogrun->legs[1]->theta(2) = param.t12;
    dogrun->legs[2]->theta(0) = param.t20;
    dogrun->legs[2]->theta(1) = param.t21;
    dogrun->legs[2]->theta(2) = param.t22;
    dogrun->legs[3]->theta(0) = param.t30;
    dogrun->legs[3]->theta(1) = param.t31;
    dogrun->legs[3]->theta(2) = param.t32;


    Vec12 supportF;

    dogrun->gt;

    supportF = dogrun->solveKinetic();
    Mat4x3 t;
    t = taucalc.invKinetic(supportF, dogrun);
    res.t00 = t(0,0);
    res.t01 = t(0,1);
    res.t02 = t(0,2);
    res.t10 = t(1,0);
    res.t11 = t(1,1);
    res.t12 = t(1,2);
    res.t20 = t(2,0);
    res.t21 = t(2,1);
    res.t22 = t(2,2);
    res.t30 = t(3,0);
    res.t31 = t(3,1);
    res.t32 = t(3,2);
    
    return res;
}
