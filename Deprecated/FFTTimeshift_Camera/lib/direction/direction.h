#include <math.h>
#include <Camera.h>

typedef struct {
    double x;
    double y;
    double z;
} Point;

double yy1(double x, double da, double l);
Point direction(double ta, double tb, double tc); 
angleSet direction_angle(double ta, double tb, double tc); 