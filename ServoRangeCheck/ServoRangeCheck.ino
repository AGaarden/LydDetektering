const int servoPin = 19;  /* GPIO19 */
const int SERVO_PIN_2 = 18; /* GPIO18 */

int dutyCycle = 0;

/* Setting PWM properties */
const double PWMFreq = 50;
const int PWMChannel1 = 0; /* Zero indexed */
const int PWMChannel2 = 1; /* Zero indexed */
const int PWMResolution = 11;
//const int MAX_DUTY_CYCLE = (int)(pow(2, PWMResolution) - 1);

void setup() {
  Serial.begin(115200);
  ledcSetup(PWMChannel1, PWMFreq, PWMResolution);
  ledcSetup(PWMChannel2, PWMFreq, PWMResolution);
  /* Attach the LED PWM Channel to the GPIO Pin */
  ledcAttachPin(servoPin, PWMChannel1);
  ledcAttachPin(SERVO_PIN_2, PWMChannel2);
  ledcWrite(PWMChannel1, dutyCycle);
  ledcWrite(PWMChannel2, dutyCycle);
}

void loop() {
  while(Serial.available()) {
    String in_char = Serial.readStringUntil('\n');
    dutyCycle = in_char.toInt();
    Serial.println(dutyCycle);
    ledcWrite(PWMChannel1, dutyCycle);
    ledcWrite(PWMChannel2, dutyCycle + 20);
    delay(10);
  }
}

/*
 * Herunder er de forskellige grader, der kan opnås på forskellige PWM opløsninger med vores sæt motorer (MG996R).
 * I parentesen står den maksimale værdi ved opløsningerne, men disse er udover de 180 grader vi skal bruge
 */
// Servo 8 bit: 5-30 (5-33)
// Servo 10 bit: 20-119 (18-134)
// Servo 11 bit: 39-238 (36-268)
// Servo 12 bit: 78-477 (71-537)
// Servo 16 bit: 1220-7600 (1134-8600)
