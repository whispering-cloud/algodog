#include "gait.h"
#include "moduleIO.h"
#include <sys/time.h>

FSM::FSM(GaitState* gtype){
    type = gtype;
}

bool FSM::getContactState(int id){
    return true;
}


void FSM::changeGait(GaitState* gtype){
    if (type == gtype) return;
    type->changeInto(gtype);
    gtype->enter();
    type = gtype;
}

void GaitState_Stand::enter(){
    startTime = chron.getus();
    return;
}

void GaitState_Stand::changeInto(GaitState* to){
    return;
}

bool GaitState_Stand::calcContact(int id){
    return true;
}

void GaitState_Trott::enter(){
    startTime = chron.getus();

    return;
}

void GaitState_Trott::changeInto(GaitState* to){
    return;
}

bool GaitState_Trott::calcContact(int id){
    double pt = double((chron.getus() - startTime - (ll)(offset(id) * period) + period) % period) / period;
    if (pt <= configR){
        return true;
    }else{
        return false;
    }
    return true;
}
