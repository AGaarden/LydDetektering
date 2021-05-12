#include <Arduino.h>

#include <Camera.h>

#include <direction.h>

Camera camera; // Same as: Camera camera(19, 18, 50, 3);

angleSet resetAngles = { resetAngles.xAngle = 0, resetAngles.yAngle = 0};

angleSet angles{ 0, 0 };
angleSet angles2{ 2, 2 };
angleSet angles3{ 3, 3 };
angleSet angles4{ 20, 20 };

angleSet angles5{ 110, 110 };
angleSet angles6{ 80, 80 };
angleSet angles7{ 88, 88 };

void setup() {
  Serial.begin(115200);
  delay(3000);
}

#define R 10
#define STEP_SIZE 0.1
#define c 343.0

// typedef struct {
//     double x;
//     double y;
//     double z;
// } Point;

const Point mic_a {-0.15, 0, 0};
const Point mic_b {0.15, 0, 0};
const Point mic_c {0, 0.15, 0};
const Point ref {0, 0, 0};

double dist(Point a, Point b);

double dist(Point a, Point b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2) + pow(a.z - b.z, 2));
}

void loop() {
  const double R_SQUARED = R * R;
    for (double x = -R; x <= R; x += STEP_SIZE) {
        double lim = sqrt(R_SQUARED - pow(x, 2));
        for (double y = -lim; y <= lim; y += STEP_SIZE / 10) {
            double dist_a, dist_b, dist_c, dist_ref, ta, tb, tc;
            double z_squared = R_SQUARED - pow(x, 2) - pow(y, 2);
            double z;

            if (z_squared < 0) {
                z = 0;
            } else {
                z = sqrt(z_squared);
            }

            Point sound_source = Point {x, y, z};

            dist_a = dist(sound_source, mic_a);
            dist_b = dist(sound_source, mic_b);
            dist_c = dist(sound_source, mic_c);
            dist_ref = dist(sound_source, ref);

            ta = (dist_a - dist_ref) / c;
            tb = (dist_b - dist_ref) / c;
            tc = (dist_c - dist_ref) / c;

            Point calculated_point = direction(ta, tb, tc);
            Point corrected_point = Point {-calculated_point.x, calculated_point.z, calculated_point.y};
            angleSet calculated_angle = direction_angle(ta, tb, tc);
            camera.move(calculated_angle);
            
            calculated_point.y = -calculated_point.y;
            double dist_to_calculated = dist(corrected_point, sound_source);
        
            printf("%f, %f, %f\t %f, %f, %f\t %f, %f, %f\t %f\t %f°, %f°\n", ta, tb, tc, sound_source.x, sound_source.y, sound_source.z, corrected_point.x, corrected_point.y, corrected_point.z, dist_to_calculated, calculated_angle.xAngle, calculated_angle.yAngle);
        }
    }
}

// void loop() {
  /*
  camera.move(angles);
  delay(1500);
  camera.move(angles2);
  delay(1500);
  camera.move(angles3);
  delay(1500);
  camera.move(angles4);
  delay(1500);
  */

  // camera.move(resetAngles);

  // camera.testServos();

  /* This first tests 20 degrees in steps, then 30 in the opposite direction, then 8 steps to see if it goes to normal move */
  
  /*
  camera.stepMove(angles5);
  delay(1500);
  camera.stepMove(angles6);
  delay(1500);
  camera.stepMove(angles7);
  delay(1500);
  */
// }