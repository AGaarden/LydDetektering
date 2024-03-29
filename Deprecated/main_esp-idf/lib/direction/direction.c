#include <math.h>
#include "direction.h"

float yy1(float x, float da, float l){
    return (1/2.0)*sqrt( ( (4*pow(x-(0+l/2),2)-pow(da,2))*(pow(l,2)-pow(da,2)) ) / (pow(da,2.0)) );
}

angles direction(float ta, float tb, float tc){
    angles temp;
    int c = 343;
    float da = c*ta;
    float db = c*tb;
    float dc = c*tc;
    float l = 0.25;
    float x;
    float y;
    float z;
    float xOffset = 0;
    float yOffset = 0;
    float zOffset = 0.20;
    float kDist;
    float dist;
    float theta;
    if (da*db < 0){
        x = (pow(l,3)*pow(da,2) + pow(l,3)*pow(db,2) - 2*l*pow(da,2)*pow(db,2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2))) + sqrt(pow(da,2)*pow(db,2)*pow(l,2)*pow(-2*pow(l,2) + pow(da,2) + pow(db,2.0),2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2)));
    } else {
        x = (pow(l,3)*pow(da,2) + pow(l,3)*pow(db,2) - 2*l*pow(da,2)*pow(db,2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2))) - sqrt(pow(da,2)*pow(db,2)*pow(l,2)*pow(-2*pow(l,2) + pow(da,2) + pow(db,2.0),2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2)));
    }
    
    dist = sqrt(pow(yy1(x, da, l),2)+pow(x,2));

    z = (pow(l,2) - pow(dc,2) - 2*dc*dist) / (2*l);

    theta = asin(z/yy1(x,da,l));

    y = yy1(x,da,l)*cos(theta);

    kDist = sqrt(pow(x+xOffset,2)+pow(y+yOffset,2)+pow((z+zOffset),2));

    temp.xAngle = acos(x/sqrt(pow(x,2)+pow(y,2)))*(180/M_PI);
    temp.yAngle = acos(sqrt(pow(x,2)+pow(y,2))/kDist)*(180/M_PI)+90;
    //test
    return temp;
}