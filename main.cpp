#include <iostream>
#include "legKinetic.h"
#include "moduleIO.h"



using namespace std;

int main(){
    initParams a;
    updateParams b;
    a = {0.2,0.0,0.0,0.0,0.17,0.0,0.0,0.0,0.2,
        0.0,0.0,0.0, 5, 
        0.2,0.2,0.2, 
        10, 10, 4, 10, 10, 4, 10, 10, 4, 10, 10, 4,
        3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3,
        20, 20, 50, 450, 450, 450,
        3, 5, 0.4};
    b = {0.0, 0.0, 0.0, 
        0, 0, 0,
        1.0, 0, 0, 0,
        0, 0.2, 0.2,
        0, 0.2, 0.2,
        0, 0.2, 0.2,
        0, 0.2, 0.2
        };
    init(a);
    int ind = 0;
    while(ind < 1){
        updateResult rs = update(b);
        cout<< rs.t00<<' '<<rs.t01<<' '<<rs.t02<<' '<<rs.t10<<' '<<rs.t11<<' '<<rs.t12<<' '<<rs.t20<<' '<<rs.t21<<' '<<rs.t22<<' '<<rs.t30<<' '<<rs.t31<<' '<<rs.t32<< std::endl;
        ind++;
        int t = 1000000;
        while(t--);
    }
    return 0;
}