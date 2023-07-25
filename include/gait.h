#ifndef __GAIT_H__
#define __GAIT_H__
#include "thirdparties.h"

class GaitState{
public:
    GaitState(ll p, double r, Vec4 of)
        :period(p), configR(r), offset(of), startTime(0){};
    
    virtual void enter() = 0;
    virtual void changeInto(GaitState* to) = 0;
    virtual bool calcContact(int id) = 0;

    ll startTime;
    ll period;
    double configR;
    Vec4 offset;
};

class GaitState_Stand : public GaitState{
public:
    GaitState_Stand(ll p, double r, Vec4 of):GaitState(p, r, of){};
    void enter();
    void changeInto(GaitState* to);
    bool calcContact(int id);
};

class GaitState_Trott : public GaitState{
public:
    GaitState_Trott(ll p, double r, Vec4 of):GaitState(p, r, of){};
    void enter();
    void changeInto(GaitState* to);
    bool calcContact(int id);
};



class FSM{
public:
    FSM(GaitState* gtype);
    GaitState* type;
    //~FSM();
    void changeGait(GaitState* gtype);
    bool getContactState(int id);
};

#endif