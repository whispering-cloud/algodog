#ifndef __LEGKINETIC_H__
#define __LEGKINETIC_H__

#include "thirdparties.h"

class Leg{
public:
    int id;
    Vec3 lx;
    Vec3 theta;
    // 0,2 --> left 1; 1,3 --> right -1
    int side(){
        return (id&1)?-1:1;
    }

    // input theta return delta p 
    Vec3 forwardKinetic(Vec3 theta);
    // input delta p return theta
    Vec3 backwardKinetic(Vec3 p);
    // input theta return Jacobi Matrix
    Mat3 getJacobi(Vec3 theta);
    // input velocity return angular velocity vector
    Vec3 deriveBackwardKinetic(Vec3 vel, Vec3 theta);
    // get vector from b to i
    Vec3 getPb_i();
    // set leg
    void configLeg(int index, Vec3 length);
};


#endif
