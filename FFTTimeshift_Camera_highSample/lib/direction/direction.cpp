#include <math.h>
#include "direction.h"
#include <stdio.h>
#include <Camera.h>

double yy1(double x, double da, double l){
    return (1/2.0)*sqrt( ( (4*pow(x-(0+l/2),2)-pow(da,2))*(pow(l,2)-pow(da,2)) ) / (pow(da,2.0)) );
}

Point direction(double ta, double tb, double tc){
    angleSet temp;
    Point temp_point;
    int c = 343;
    double da = c*ta;
    double db = c*tb;
    double dc = c*tc;
    double l = 0.135;
    double x;
    double y;
    double z;
    double xOffset = 0;
    double yOffset = 0.10;
    double zOffset = 0.83;
    double kDist;
    double dist;
    double theta;
    double max_delay = l / c * 0.995;
    double theta_check;
    //printf("Max delay: %f\n", max_delay);
    if (abs(ta) > max_delay) {
        // Return no point
        return Point{NAN, NAN, NAN};
    // } else if ((da < 0) && (db > 0)) {
    //     x = (pow(l,3)*pow(da,2) + pow(l,3)*pow(db,2) - 2*l*pow(da,2)*pow(db,2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2))) + sqrt(pow(da,2)*pow(db,2)*pow(l,2)*pow(-2*pow(l,2) + pow(da,2) + pow(db,2.0),2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2)));
    // } else if ((da > 0) && (db < 0)) {
    //     x = (pow(l,3)*pow(da,2) + pow(l,3)*pow(db,2) - 2*l*pow(da,2)*pow(db,2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2))) - sqrt(pow(da,2)*pow(db,2)*pow(l,2)*pow(-2*pow(l,2) + pow(da,2) + pow(db,2.0),2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2)));
    // } else {
    //     x = 0;
    // }
    } else if ((da == db) || ((abs(da - db)) < 1e-6)) {
        x = 0;
    } else if (da*db < 0) {
        x = (pow(l,3)*pow(da,2) + pow(l,3)*pow(db,2) - 2*l*pow(da,2)*pow(db,2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2))) + sqrt(pow(da,2)*pow(db,2)*pow(l,2)*pow(-2*pow(l,2) + pow(da,2) + pow(db,2.0),2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2)));
    } else {
        x = (pow(l,3)*pow(da,2) + pow(l,3)*pow(db,2) - 2*l*pow(da,2)*pow(db,2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2))) - sqrt(pow(da,2)*pow(db,2)*pow(l,2)*pow(-2*pow(l,2) + pow(da,2) + pow(db,2.0),2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2)));
    }


    dist = sqrt(pow(yy1(x, da, l),2)+pow(x,2));

    // printf("pow(l, 2): %f\n", pow(l, 2));
    // printf("pow(dc, 2): %f\n", pow(dc, 2));
    // printf("2*dc*dist: %f\n", 2*dc*dist);
    z = (pow(l,2) - pow(dc,2) - 2*dc*dist) / (2*l);

    //printf("yy1(x,da,l): %f\n", yy1(x,da,l));
    //printf("z/yy1(x,da,l): %f\n", z/yy1(x,da,l));
    theta_check = z/yy1(x,da,l);

    if (theta_check > 1) {
        theta_check = 1;
    } else if (theta_check < -1) {
        theta_check = -1;
    }
    theta = asin(theta_check);
    if (isnan(theta)) {
        printf("Theta is nan: %f\n", z/yy1(x,da,l));
    }
    //printf("theta: %f\n", theta);
    y = yy1(x,da,l)*cos(theta);

    kDist = sqrt(pow(x+xOffset,2)+pow(y+yOffset,2)+pow((z+zOffset),2));

    temp.xAngle = acos(x/sqrt(pow(x,2)+pow(y,2)))*(180/M_PI);
    temp.yAngle = acos(sqrt(pow(x,2)+pow(y,2))/kDist)*(180/M_PI)+90;
    //test
    //printf("%f\t %f\t %f\n", x, y, z);
    return Point {x, y, z};
}

angleSet direction_angle(double ta, double tb, double tc){
    angleSet temp;
    int c = 343;
    double da = c*ta;
    double db = c*tb;
    double dc = c*tc;
    double l = 0.15;
    double x;
    double y;
    double z;
    double xOffset = 0;
    double yOffset = 0.10;
    double zOffset = 0.83;
    double kDist;
    double dist;
    double theta;
    double max_delay = l / c * 0.995;
    double theta_check;
    //printf("Max delay: %f\n", max_delay);
    if (abs(ta) > max_delay) {
        // Return no angles
        return angleSet{NAN, NAN};
    // } else if ((da < 0) && (db > 0)) {
    //     x = (pow(l,3)*pow(da,2) + pow(l,3)*pow(db,2) - 2*l*pow(da,2)*pow(db,2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2))) + sqrt(pow(da,2)*pow(db,2)*pow(l,2)*pow(-2*pow(l,2) + pow(da,2) + pow(db,2.0),2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2)));
    // } else if ((da > 0) && (db < 0)) {
    //     x = (pow(l,3)*pow(da,2) + pow(l,3)*pow(db,2) - 2*l*pow(da,2)*pow(db,2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2))) - sqrt(pow(da,2)*pow(db,2)*pow(l,2)*pow(-2*pow(l,2) + pow(da,2) + pow(db,2.0),2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2)));
    // } else {
    //     x = 0;
    // }
    } else if ((da == db) || ((abs(da - db)) < 1e-6)) {
        x = 0;
    } else if (da*db < 0) {
        x = (pow(l,3)*pow(da,2) + pow(l,3)*pow(db,2) - 2*l*pow(da,2)*pow(db,2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2))) + 
            sqrt(pow(da,2)*pow(db,2)*pow(l,2)*pow(-2*pow(l,2) + pow(da,2) + pow(db,2),2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2)));
        // if (da > db) {
        //     x = (pow(l,3)*pow(da,2) + pow(l,3)*pow(db,2) - 2*l*pow(da,2)*pow(db,2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2))) + 
        //     sqrt(pow(da,2)*pow(db,2)*pow(l,2)*pow(-2*pow(l,2) + pow(da,2) + pow(db,2),2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2)));
        // } else {
        //     x = -((pow(l,3)*pow(da,2) + pow(l,3)*pow(db,2) - 2*l*pow(da,2)*pow(db,2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2))) + 
        //     sqrt(pow(da,2)*pow(db,2)*pow(l,2)*pow(-2*pow(l,2) + pow(da,2) + pow(db,2),2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2))));
        // }
    } else {
        x = (pow(l,3)*pow(da,2) + pow(l,3)*pow(db,2) - 2*l*pow(da,2)*pow(db,2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2))) - 
        sqrt(pow(da,2)*pow(db,2)*pow(l,2)*pow(-2*pow(l,2) + pow(da,2) + pow(db,2),2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2)));
    }

    dist = sqrt(pow(yy1(x, da, l),2)+pow(x,2));

    // printf("pow(l, 2): %f\n", pow(l, 2));
    // printf("pow(dc, 2): %f\n", pow(dc, 2));
    // printf("2*dc*dist: %f\n", 2*dc*dist);
    z = (pow(l,2) - pow(dc,2) - 2*dc*dist) / (2*l);

    //printf("yy1(x,da,l): %f\n", yy1(x,da,l));
    //printf("z/yy1(x,da,l): %f\n", z/yy1(x,da,l));
    theta_check = z/yy1(x,da,l);

    if (theta_check > 1) {
        theta_check = 1;
    } else if (theta_check < -1) {
        theta_check = -1;
    }
    theta = asin(theta_check);
    if (isnan(theta)) {
        printf("Theta is nan: %f\n", z/yy1(x,da,l));
    }
    //printf("theta: %f\n", theta);
    y = yy1(x,da,l)*cos(theta);

    kDist = sqrt(pow(x-xOffset,2)+pow(y-yOffset,2)+pow((z-zOffset),2));

    temp.xAngle = acos((x - xOffset)/sqrt(pow(x - xOffset,2)+pow(y - yOffset,2)))*(180/M_PI);

    if (z < 0) {
        temp.yAngle = -acos(sqrt(pow(x - xOffset ,2)+pow(y - yOffset,2))/kDist)*(180/M_PI)+90;
    } else {
        temp.yAngle = acos(sqrt(pow(x - xOffset,2)+pow(y - yOffset,2))/kDist)*(180/M_PI)+90;
    }
    
    //test
    //printf("%f\t %f\t %f\n", x, y, z);
    return temp;
}


// Old method without sanity checks:
// angleSet direction_angle(double ta, double tb, double tc){
//     angleSet temp;
//     int c = 343;
//     double da = c*ta;
//     double db = c*tb;
//     double dc = c*tc;
//     double l = 0.15;
//     double x;
//     double y;
//     double z;
//     double xOffset = 0;
//     double yOffset = 0;
//     double zOffset = 0.20;
//     double kDist;
//     double dist;
//     double theta;
//     if (da*db < 0){
//         x = (pow(l,3)*pow(da,2) + pow(l,3)*pow(db,2) - 2*l*pow(da,2)*pow(db,2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2))) + sqrt(pow(da,2)*pow(db,2)*pow(l,2)*pow(-2*pow(l,2) + pow(da,2) + pow(db,2.0),2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2)));
//     } else {
//         x = (pow(l,3)*pow(da,2) + pow(l,3)*pow(db,2) - 2*l*pow(da,2)*pow(db,2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2))) - sqrt(pow(da,2)*pow(db,2)*pow(l,2)*pow(-2*pow(l,2) + pow(da,2) + pow(db,2.0),2))/(2*pow(l,2)*(-pow(da,2) + pow(db,2)));
//     }
    
//     dist = sqrt(pow(yy1(x, da, l),2)+pow(x,2));

//     z = (pow(l,2) - pow(dc,2) - 2*dc*dist) / (2*l);

//     theta = asin(z/yy1(x,da,l));

//     y = yy1(x,da,l)*cos(theta);

//     kDist = sqrt(pow(x+xOffset,2)+pow(y+yOffset,2)+pow((z+zOffset),2));

//     temp.xAngle = acos(x/sqrt(pow(x,2)+pow(y,2)))*(180/M_PI);
//     temp.yAngle = acos(sqrt(pow(x,2)+pow(y,2))/kDist)*(180/M_PI)+90;
//     //test
//     return temp;
// }