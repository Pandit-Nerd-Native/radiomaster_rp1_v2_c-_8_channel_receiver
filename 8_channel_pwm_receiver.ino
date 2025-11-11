#include <AlfredoCRSF.h>
#include <HardwareSerial.h>

// === Pin Definitions ===
#define PIN_RX 16
#define PIN_TX 17

// Motor Pins (L298N)
const int motor1Pin1 = 4;
const int motor1Pin2 = 5;
const int motor2Pin1 = 18;
const int motor2Pin2 = 19;
const int enable1Pin = 2;   // PWM Left
const int enable2Pin = 15;  // PWM Right

// PWM Settings
#define PWM_FREQ 1000
#define PWM_RES 8

// === CRSF Setup ===
HardwareSerial crsfSerial(2);
AlfredoCRSF crsf;

void setup() {
  Serial.begin(115200);

  // Motor pin modes
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  ledcSetup(0, PWM_FREQ, PWM_RES);
  ledcSetup(1, PWM_FREQ, PWM_RES);
  ledcAttachPin(enable1Pin, 0);
  ledcAttachPin(enable2Pin, 1);

  // CRSF Init
  crsfSerial.begin(CRSF_BAUDRATE, SERIAL_8N1, PIN_RX, PIN_TX);
  crsf.begin(crsfSerial);
}

void loop() {
  crsf.update();

  if (crsf.isLinkUp()) {
    const crsf_channels_t* ch = crsf.getChannelsPacked();

    bool isArmed = ch->ch4 > 1000;

    if (isArmed) {
      // More accurate floating-point scaled mapping
      int throttle = mapCRSF(ch->ch2, 172, 1811, 0, 255);     // Forward
      int pitch    = mapCRSF(ch->ch1, 172, 1811, 0, 255);     // Reverse
      int roll     = mapCRSF(ch->ch0, 172, 1811, -255, 255);  // Steering
      int yawSpin  = mapCRSF(ch->ch3, 172, 1811, -255, 255);  // Spot rotate

      yawSpin = -yawSpin;

      int baseSpeed = throttle - pitch;
      int leftSpeed = baseSpeed + roll - yawSpin;
      int rightSpeed = baseSpeed - roll + yawSpin;

      leftSpeed = constrain(leftSpeed, -255, 255);
      rightSpeed = constrain(rightSpeed, -255, 255);

      setMotor(leftSpeed, rightSpeed);

      Serial.print("Armed | Throttle: "); Serial.print(throttle);
      Serial.print(" | Pitch: "); Serial.print(pitch);
      Serial.print(" | Roll: "); Serial.print(roll);
      Serial.print(" | Yaw: "); Serial.print(yawSpin);
      Serial.print(" | L: "); Serial.print(leftSpeed);
      Serial.print(" | R: "); Serial.println(rightSpeed);
    } else {
      setMotor(0, 0);
      Serial.println("Disarmed | Motor stopped.");
    }

  } else {
    setMotor(0, 0);
    Serial.println("Signal lost!");
  }

  delay(10);
}

// === Floating-Point CRSF Mapping ===
int mapCRSF(int value, int minIn, int maxIn, int minOut, int maxOut) {
  float scale = (float)(value - minIn) / (maxIn - minIn);
  scale = constrain(scale, 0.0, 1.0);
  return minOut + scale * (maxOut - minOut);
}

// === Motor Control ===
void setMotor(int speedL, int speedR) {
  digitalWrite(motor1Pin1, speedL >= 0 ? HIGH : LOW);
  digitalWrite(motor1Pin2, speedL < 0 ? HIGH : LOW);
  digitalWrite(motor2Pin1, speedR >= 0 ? HIGH : LOW);
  digitalWrite(motor2Pin2, speedR < 0 ? HIGH : LOW);

  ledcWrite(0, abs(speedL));
  ledcWrite(1, abs(speedR));
}
