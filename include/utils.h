#ifndef __UTILS_H__
#define __UTILS_H__
#include "thirdparties.h"
#include <chrono>

extern Mat3 rpy2Matrix(Vec3 rpy);
extern Mat3 quat2Matrix(Vec4 quat);
extern Mat3 crossMultiplyMat(Vec3 vec);
extern Mat3 inverseMat(Mat3 mat);
extern Mat4x3 foldVector(Vec12 vec);

class Chronostasis{
    std::chrono::system_clock::time_point startTime;
public:
    void init(){
        startTime = std::chrono::system_clock::now();
    }
    ll getus(){
        auto duration = std::chrono::system_clock::now() - startTime;
        return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
    }
    ll getms(){
        auto duration = std::chrono::system_clock::now() - startTime;
        return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
    }
    ll gets(){
        auto duration = std::chrono::system_clock::now() - startTime;
        return std::chrono::duration_cast<std::chrono::seconds>(duration).count();
    }
};


#endif