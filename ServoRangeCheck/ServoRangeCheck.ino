const int SERVO_PIN_1 = 19;  /* GPIO19 */
const int SERVO_PIN_2 = 18; /* GPIO18 */

int duty_cycle = 0;

/* Setting PWM properties */
const double PWM_FREQ = 50;
const int PWM_CHANNEL_1 = 0; /* Zero indexed */
const int PWM_CHANNEL_2 = 1; /* Zero indexed */
const int PWM_RESOLUTION = 11;
//const int MAX_DUTY_CYCLE = (int)(pow(2, PWM_RESOLUTION) - 1);

void setup() {
  Serial.begin(115200);
  ledcSetup(PWM_CHANNEL_1, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_2, PWM_FREQ, PWM_RESOLUTION);
  /* Attach the LED PWM Channel to the GPIO Pin */
  ledcAttachPin(SERVO_PIN_1, PWM_CHANNEL_1);
  ledcAttachPin(SERVO_PIN_2, PWM_CHANNEL_2);
  ledcWrite(PWM_CHANNEL_1, duty_cycle);
  ledcWrite(PWM_CHANNEL_2, duty_cycle);
}

void loop() {
  while(Serial.available()) {
    String in_char = Serial.readStringUntil('\n');
    duty_cycle = in_char.toInt();
    Serial.println(duty_cycle);
    ledcWrite(PWM_CHANNEL_1, duty_cycle);
    ledcWrite(PWM_CHANNEL_2, duty_cycle);
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
