/* RESOLUTION RANGE LOOKUP TABLE (From 0-180 on MG996R Servo motors. Parentheses are minimum and maximum outside of 0-180 range)
 * 8 bit: 5-30 (5-33)
 * 10 bit: 20-119 (18-134)
 * 11 bit: 39-238 (36-268) (+5 for map, 2.1%)
 * 12 bit: 78-477 (71-537) (+5 for map, 1%)
 * 16 bit: 1220-7600 (1134-8600) (+152 for map, 2%)
 */

/* Servo pins */
const uint8_t SERVO_PIN_1 = 18; /* GPIO18, 8 pins down from ground */
const uint8_t SERVO_PIN_2 = 19; /* GPIO19, 9 pins down from ground */

/* Setting necessary PWM properties */
const double PWM_FREQ = 50;
const uint8_t PWM_RESOLUTION = 16; /* PWM Resolution of 11 bits gives the value range of 39-238 from 0 to 180 degs */
const uint8_t PWM_CHANNEL_1 = 0; /* PWM channels are zero indexed */
const uint8_t PWM_CHANNEL_2 = 1;

/* Values based on PWM_RESOLUTION, check lookup table */
const uint16_t PWM_MIN = 1220;
const uint16_t PWM_MAX = 7600;
const uint8_t PWM_MAP_EXTRA = 152; /* Arbitrary value for getting closer to 180 degrees when using map */

uint16_t angleToPwm(uint8_t angle) {
  return map(angle, 0, 180, PWM_MIN, PWM_MAX + PWM_MAP_EXTRA);
}

uint8_t pwmToAngle(uint16_t pwm) {
  return map(pwm, PWM_MIN, PWM_MAX, 0, 180);
}

static void testServos() {
  Serial.println("Starting servo test.");

  ledcWrite(PWM_CHANNEL_1, angleToPwm(0));
  ledcWrite(PWM_CHANNEL_2, angleToPwm(0));

  /* Move servo 1 from 0 to 180 degrees */
  for (int pos = 0; pos <= 180; pos += 1) {
    ledcWrite(PWM_CHANNEL_1, angleToPwm(pos));
    
    delay(15);
  }
  Serial.print("Servo 1 position: "); Serial.println(pwmToAngle(ledcRead(PWM_CHANNEL_1)));
  Serial.print("Servo 2 position: "); Serial.println(pwmToAngle(ledcRead(PWM_CHANNEL_2)));
  delay(3000);

  /* Move servo 2 from 0 to 180 degrees */
  for (int pos = 0; pos <= 180; pos += 1) {
    ledcWrite(PWM_CHANNEL_2, angleToPwm(pos));
    delay(15);
  }
  Serial.print("Servo 1 position: "); Serial.println(pwmToAngle(ledcRead(PWM_CHANNEL_1)));
  Serial.print("Servo 2 position: "); Serial.println(pwmToAngle(ledcRead(PWM_CHANNEL_2)));
  delay(3000);

  /* Move servo 1 from 180 to 0 degrees */
  for (int pos = 180; pos >= 0; pos -= 1) {
    ledcWrite(PWM_CHANNEL_1, angleToPwm(pos));
    delay(15);
  }
  Serial.print("Servo 1 position: "); Serial.println(pwmToAngle(ledcRead(PWM_CHANNEL_1)));
  Serial.print("Servo 2 position: "); Serial.println(pwmToAngle(ledcRead(PWM_CHANNEL_2)));
  delay(3000);

  /* Move servo 2 from 180 to 0 degrees */
  for (int pos = 180; pos >= 0; pos -= 1) {
    ledcWrite(PWM_CHANNEL_2, angleToPwm(pos));
    delay(15);
  }
  Serial.print("Servo 1 position: "); Serial.println(pwmToAngle(ledcRead(PWM_CHANNEL_1)));
  Serial.print("Servo 2 position: "); Serial.println(pwmToAngle(ledcRead(PWM_CHANNEL_2)));
  delay(1000);
  
  Serial.println("Servo testing complete.");
}

void setup() {
  Serial.begin(115200);
  
  /* Set up the PWM channels */
  ledcSetup(PWM_CHANNEL_1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_2, PWM_FREQ, PWM_RESOLUTION);
  
  /* Attach pins to the right PWM channels */
  ledcAttachPin(SERVO_PIN_1, PWM_CHANNEL_1);
  ledcAttachPin(SERVO_PIN_2, PWM_CHANNEL_2);

  /* Set both servos at their middle positions */
  ledcWrite(PWM_CHANNEL_1, ((PWM_MAX - PWM_MIN)/2) + PWM_MIN);
  ledcWrite(PWM_CHANNEL_2, ((PWM_MAX - PWM_MIN)/2) + PWM_MIN);
}

void loop() {
  testServos();
  delay(3000);
}
