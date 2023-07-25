#include "balanceControl.h"
#include "Array.hh"
#include "QuadProg++.hh"
#include "utils.h"
#include "moduleIO.h"

quadprogpp::Matrix<double> tM(Eigen::Matrix<double, -1, -1> arg){
    quadprogpp::Matrix<double> res;
    res.resize(arg.rows(),arg.cols());
    for (int i = 0; i < arg.rows(); i++){
        for (int j = 0; j < arg.cols(); j++){
            res[i][j] = arg(i,j);
        }
    }
    return res;
}
quadprogpp::Vector<double> tV(Eigen::Matrix<double, -1, 1> arg){
    quadprogpp::Vector<double> res;
    res.resize(arg.size());
    for (int i = 0; i < arg.size(); i++){
        res[i] = arg(i);
    }
    return res;
}

Vec12 solveQP(Mat12 G,Vec12 g0,Eigen::Matrix<double, -1, -1> CE,Eigen::VectorXd ce0,Eigen::Matrix<double, -1, -1> CI,Eigen::VectorXd ci0){
    quadprogpp::Vector<double> res;
    Vec12 ans;
    quadprogpp::Matrix<double> _G,_CE,_CI;
    quadprogpp::Vector<double> _g0,_ce0,_ci0;
    _G = tM(G);
    _g0 = tV(g0);
    _CE = tM(CE);
    _ce0 = tV(ce0);
    _CI = tM(CI);
    _ci0 = tV(ci0);
    quadprogpp::solve_quadprog(_G,_g0,_CE,_ce0,_CI,_ci0,res);
    for (int i=0;i<12;i++){
        ans(i) = res[i];
    }
    return ans;
}


Balancer::Balancer(Vec3 l, Vec12 vecW, Vec12 vecU, Vec6 vecS){
    prevSupportF.setZero();
    for (int i=0;i<4;i++){
        legs[i] = new Leg();
        legs[i]->configLeg(i, l);
    }
    gt = new FSM(STAND);
    vecG << 0, 0, -9.8;
    fricMat <<   1,  0, configFric,
                -1,  0, configFric,
                 0,  1, configFric,
                 0, -1, configFric,
                 0,  0, 1;
    matS = vecS.asDiagonal();
    matW = vecW.asDiagonal();
    matU = vecU.asDiagonal();

}
Balancer::~Balancer(){
    for (int i=0;i<4;i++){
        delete legs[i];
    }
    delete gt;
}
Vec12 Balancer::solveKinetic(){

    Eigen::Matrix<double, 6, 12> matA;
    for (int i = 0;i < 4;i++){
        matA.block(0, 3*i, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
        Vec3 pgi;
        pgi = Rs_b * (legs[i]->getPb_i()) - Rs_b * Pg_b;
        matA.block(3,3*i,3,3) = crossMultiplyMat(pgi);
    }
    Mat12 matG = matA.transpose()*matS*matA + configAlpha*matW + configBeta*matU;
    Vec12 x;
    Vec12 g0;
    vecb_d.head(3) = mass * (vecAcc - vecG);
    vecb_d.tail(3) = Rs_b * INTERTIA * Rs_b.transpose() * omegaAcc;
    g0 = -vecb_d.transpose() * matS * matA - prevSupportF.transpose() * configBeta * matU;
    
    bool contactState[4];
    int n=0, m=0;
    for (int i = 0;i < 4;i++){
        contactState[i] = gt->getContactState(i);
        if (contactState[i])
            m++;
        else
            n++;
    }
    Eigen::Matrix<double, -1, -1> CE, CI;
    Eigen::VectorXd ce0, ci0;
    ce0.resize(3*4-n);
    ce0.setZero();
    CE.resize(3*4-n,12);
    int tmpn=0, tmpm=0;
    for (int i=0;i<4;i++){
        if (contactState[i]){
            CE.block(tmpn*3,i*3,3,3) = Eigen::MatrixXd::Identity(3, 3);
            tmpn++;
        }else{
            CI.block(tmpn*5,i*3,5,3) = fricMat;
            tmpm++;
        }
    }
    x = solveQP(matG, g0, CE.transpose(), ce0, CI.transpose(), ci0);
    return x;
}
