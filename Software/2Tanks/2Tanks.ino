#include <PID_v1.h>

// Tank heights
const float tankHeight1 = 13.0;
const float tankHeight2 = 15.0;

// Ultrasonic pins
const int trig1 = 2, echo1 = 3; // Tank 1
const int trig2 = 4, echo2 = 5; // Tank 2

// Pump control
const int pumpPWM = 9;

// PID variables
double setpoint = 6;
double input, output;
double Kp = 100, Ki = 1, Kd = 0.1;

// PID setup
PID myPID(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup() {
  Serial.begin(115200);

  pinMode(trig1, OUTPUT);
  pinMode(echo1, INPUT);
  pinMode(trig2, OUTPUT);
  pinMode(echo2, INPUT);
  pinMode(pumpPWM, OUTPUT);

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255); // PWM range
}

float getLevel(int trigPin, int echoPin, float tankHeight) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  float distance = duration * 0.034 / 2.0;

  float level = tankHeight - distance;
  if (level < 0) level = 0;
  if (level > tankHeight) level = tankHeight;
  return level;
}

void loop() {
  // Read tank levels
  float tank1Level = getLevel(trig1, echo1, tankHeight1);
  input = getLevel(trig2, echo2, tankHeight2);

  // Compute PID
  myPID.Compute();
  analogWrite(pumpPWM, (int)output);

  // --- Serial Receive Format: "Kp,Ki,Kd,Setpoint\n" ---
  if (Serial.available()) {
    String s = Serial.readStringUntil('\n');
    s.trim(); // Remove trailing newline
    int idx1 = s.indexOf(',');
    int idx2 = s.indexOf(',', idx1 + 1);
    int idx3 = s.indexOf(',', idx2 + 1);

    if (idx1 > 0 && idx2 > idx1 && idx3 > idx2) {
      // Extract values
      Kp = s.substring(0, idx1).toFloat();
      Ki = s.substring(idx1 + 1, idx2).toFloat();
      Kd = s.substring(idx2 + 1, idx3).toFloat();
      setpoint = s.substring(idx3 + 1).toFloat();

      // Update PID tunings
      myPID.SetTunings(Kp, Ki, Kd);
    }
  }

  // --- Send to LabVIEW ---
  Serial.print(tank1Level); Serial.print(",");
  Serial.print(input); Serial.print(",");
  Serial.println(output);

  delay(500);
}