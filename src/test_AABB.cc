#include "include/AABB.h"

int main(){
    point p1(1, 1, 1); point p2(3, 2, 2);
    AABB temp(p1, p2);
    printf("%f", temp.volume());
}