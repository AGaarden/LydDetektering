/*
 * This file includes the necessary functions and variables to move a camera set on two servos. 
 * Requirements: 
 * - ESP32Servo.h
 */

// make a structure with the necessary angles to point towards an occurrence
typedef struct {
  float xAngle;
  float yAngle;
} angleSet;

// Define the two servo objects, attach pins in setup
Servo servo_1;
Servo servo_2;

/* 
 *  Name: servoSetup
 *  Input: Servo pin 1 (optional), servo pin 2 (optional), period hertz (optional)
 *  Output: None
 *  Remarks:
 *  The following function sets up servos as per the ESP32Servo library
 *  Base servo pins are 18 and 19, and base period hertz is 50
 *  NOTE: Servos only available on pins 2, 4, 5, 12-19, 21-23, 25-27, 32-33
 */
static void servoSetup(uint8_t servo_pin_1 = 18, uint8_t servo_pin_2 = 19, uint16_t period_hertz = 50); /* Prototype built to make base values */
static void servoSetup(uint8_t servo_pin_1, uint8_t servo_pin_2, uint16_t period_hertz) {
  // Allow allocation of all timers for the servo library
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Set the frequenzy for each servo
  servo_1.setPeriodHertz(period_hertz);
  servo_2.setPeriodHertz(period_hertz);

  Serial.print("Servo period initialized at "); Serial.print(period_hertz); Serial.println("Hz");

  // Attach pins to servos
  servo_1.attach(servo_pin_1);
  servo_2.attach(servo_pin_2);

  Serial.print("Servo 1 initialized at pin "); Serial.println(servo_pin_1);
  Serial.print("Servo 2 initialized at pin "); Serial.println(servo_pin_2);
}

/*
 * Name: moveCamera
 * Input: angleSet structure as shown at the top of this file
 * Output: None
 * Remarks:
 * Moves the camera into the position given
 * Has no inherent error correction
 */
static void moveCamera(angleSet angles) {
  int xAngle = round(angles.xAngle); /* Converted for use with the ESP32Servo library */
  int yAngle = round(angles.yAngle); /* -||- */

  // Serial print line below will print "Moving camera to position: ({xAngle}, {yAngle})"
  Serial.print("Moving camera to position: (");Serial.print(xAngle);Serial.print(", ");Serial.print(yAngle);Serial.println(")");

  servo_1.write(xAngle);
  servo_2.write(yAngle);

  Serial.println("Camera moved.");
}

/*
 * Name: testServos
 * Input: None
 * Output: None
 * Remarks:
 * Moves the two set up servos from 0 to 180 degrees
 * Can only be called once servoSetup is done
 */
static void testServos() {
  Serial.println("Starting servo test.");
  
  servo_1.write(0);
  servo_2.write(0);
  Serial.print("Servo 1 position: "); Serial.println(servo_1.read());
  Serial.print("Servo 2 position: "); Serial.println(servo_2.read());

  /* Move servo 1 from 0 to 180 degrees */
  for (int pos = 0; pos <= 180; pos += 1) {
    servo_1.write(pos);
    delay(15);
  }
  Serial.print("Servo 1 position: "); Serial.println(servo_1.read());
  Serial.print("Servo 2 position: "); Serial.println(servo_2.read());
  delay(3000);

  /* Move servo 2 from 0 to 180 degrees */
  for (int pos = 0; pos <= 180; pos += 1) {
    servo_2.write(pos);
    delay(15);
  }
  Serial.print("Servo 1 position: "); Serial.println(servo_1.read());
  Serial.print("Servo 2 position: "); Serial.println(servo_2.read());
  delay(3000);

  /* Move servo 1 from 180 to 0 degrees */
  for (int pos = 180; pos >= 0; pos -= 1) {
    servo_1.write(pos);
    delay(15);
  }
  Serial.print("Servo 1 position: "); Serial.println(servo_1.read());
  Serial.print("Servo 2 position: "); Serial.println(servo_2.read());
  delay(3000);

  /* Move servo 2 from 180 to 0 degrees */
  for (int pos = 180; pos >= 0; pos -= 1) {
    servo_2.write(pos);
    delay(15);
  }
  Serial.print("Servo 1 position: "); Serial.println(servo_1.read());
  Serial.print("Servo 2 position: "); Serial.println(servo_2.read());
  delay(3000);
  
  Serial.println("Servo testing complete.");
}
