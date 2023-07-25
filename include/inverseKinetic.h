#ifndef __INVERSEKINETIC_H__
#define __INVERSEKINETIC_H__

#include "thirdparties.h"
#include "balanceControl.h"

class KineticControl{
public:
    Mat4x3 invKinetic(Vec12 supportF, Balancer* dog);
};

#endif