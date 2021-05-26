#include <math.h>

typedef struct {
    double xAngle;
    double yAngle;
} angleSet;

typedef struct {
    double x;
    double y;
    double z;
} Point;

double yy1(double x, double da, double l);

Point direction(double ta, double tb, double tc); 
angleSet direction_angle(double ta, double tb, double tc); 

