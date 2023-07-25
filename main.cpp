#include <iostream>
#include "legKinetic.h"
#include "moduleIO.h"



using namespace std;

int main(){
    initParams a;
    updateParams b;
    a = {1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0};
    init(a);
    int ind = 0;
    cout<<INTERTIA<<endl;
    while(ind < 1000){
        //update(b);
        ind++;
        cout<<ind<<endl;
        int t = 1000000;
        while(t--);
    }
    return 0;
}