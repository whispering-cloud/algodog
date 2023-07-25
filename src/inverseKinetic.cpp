#include "inverseKinetic.h"
#include "balanceControl.h"
#include "utils.h"

Mat4x3 KineticControl::invKinetic(Vec12 supportF, Balancer* dog){
    Mat4x3 sF = foldVector(supportF);
    Mat4x3 torque;
    for (int i=0;i<4;i++){
        //torque.col(i) = - dog->legs[i]->getJacobi(dog->legs[i]->theta).transpose() * dog->Rs_b.transpose() * sF.col(i);
    }
    return torque;
}