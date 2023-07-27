#ifndef __MODULEIO_H__
#define __MODULEIO_H__

#include "thirdparties.h"
#include "balanceControl.h"
#include "inverseKinetic.h"
#include "utils.h"
#include "gait.h"

const Vec3 G({0,0,-9.8});

extern Mat3 INTERTIA;
extern Balancer *dogrun;
extern KineticControl taucalc;
extern Vec3 Pg_b;
extern double mass;
extern Chronostasis chron;

extern GaitState_Stand* STAND;
extern GaitState_Trott* TROTT;


const double fl_x = 0.1745;
const double fl_y = 0.062; 

extern "C" struct initParams{
    // 机器人转动惯量矩阵3x3
    double i00, i01, i02, i10, i11, i12, i20, i21, i22; // 对角矩阵
    // 机器人坐标原点到重心的向量(常量)
    double pg_b0, pg_b1, pg_b2; // 0, 0, 0
    // 机器人质量
    double m; // 单位kg

    // 机器人腿长
    double l1, l2, l3; // 单位m
    // 平衡器参数 W(12) U(12) S(6) 
    double w0, w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11; //  10, 10, 4, 10, 10, 4, 10, 10, 4, 10, 10, 4
    double u0, u1, u2, u3, u4, u5, u6, u7, u8, u9, u10, u11; //  3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3
    double s0, s1, s2, s3, s4, s5;  // 20, 20, 50, 450, 450, 450
    double alpha, beta, fric; // 3, 5, 0.4
};

extern "C" struct updateParams{
    // 刚体加速度
    double a00, a01, a02;
    // 刚体角加速度
    double oa0, oa1, oa2;
    
    // 当前刚体欧拉角
    double quat0, quat1, quat2, quat3;
    // 腿实际电机角度
    double t00,t01,t02;
    double t10,t11,t12;
    double t20,t21,t22;
    double t30,t31,t32;
};

extern "C" struct updateResult{
    // 扭矩矩阵(未检查)
    double t00, t01, t02;
    double t10, t11, t12;
    double t20, t21, t22;
    double t30, t31, t32;
};

extern "C" int init(initParams param);

extern "C" updateResult update(updateParams param);

#endif