/*
  BAXTER - Consola de mandos ESP32
  ---------------------------------
  Protocolo USB serial:
      V,x,y,z,grip,arm

  x, y, z = -1.0 ... +1.0
  grip     = 0 abierto / 1 cerrado
  arm      = 0 izquierdo / 1 derecho

  Hardware:
      Joystick X  -> GPIO34 (ADC)
      Joystick Y  -> GPIO35 (ADC)
      Potenciómetro Z -> GPIO32 (ADC)

      Botón GRIP -> GPIO25
      Botón ARM  -> GPIO26

  Todos los botones se conectan entre GPIO y GND.
*/

const int JOY_X = 34;
const int JOY_Y = 35;
const int POT_Z = 32;

const int BTN_GRIP = 25;
const int BTN_ARM  = 26;

bool gripClosed = false;
bool rightArm = false;

bool lastGripReading = HIGH;
bool lastArmReading = HIGH;

unsigned long lastGripDebounce = 0;
unsigned long lastArmDebounce = 0;

const unsigned long DEBOUNCE_MS = 40;
const unsigned long SEND_PERIOD_MS = 30;

unsigned long lastSend = 0;

float normalizeADC(int value, int center = 2048) {
  float v = ((float)value - center) / 2048.0;

  if (fabs(v) < 0.08) {
    return 0.0;
  }

  if (v > 0.0) {
    v = (v - 0.08) / 0.92;
  } else {
    v = (v + 0.08) / 0.92;
  }

  if (v > 1.0) v = 1.0;
  if (v < -1.0) v = -1.0;

  return v;
}

float normalizePot(int value) {
  // Potenciómetro completo: 0..4095 -> -1..+1
  float v = ((float)value / 4095.0) * 2.0 - 1.0;

  if (fabs(v) < 0.05) {
    v = 0.0;
  }

  return constrain(v, -1.0, 1.0);
}

void readButtons() {
  bool gripReading = digitalRead(BTN_GRIP);
  bool armReading = digitalRead(BTN_ARM);

  if (gripReading != lastGripReading) {
    lastGripDebounce = millis();
  }

  if ((millis() - lastGripDebounce) > DEBOUNCE_MS) {
    if (gripReading == LOW && lastGripReading == HIGH) {
      gripClosed = !gripClosed;
    }
  }

  if (armReading != lastArmReading) {
    lastArmDebounce = millis();
  }

  if ((millis() - lastArmDebounce) > DEBOUNCE_MS) {
    if (armReading == LOW && lastArmReading == HIGH) {
      rightArm = !rightArm;
    }
  }

  lastGripReading = gripReading;
  lastArmReading = armReading;
}

void sendCommand() {
  int rawX = analogRead(JOY_X);
  int rawY = analogRead(JOY_Y);
  int rawZ = analogRead(POT_Z);

  float x = normalizeADC(rawX);
  float y = normalizeADC(rawY);
  float z = normalizePot(rawZ);

  Serial.print("V,");
  Serial.print(x, 3);
  Serial.print(",");
  Serial.print(y, 3);
  Serial.print(",");
  Serial.print(z, 3);
  Serial.print(",");
  Serial.print(gripClosed ? 1 : 0);
  Serial.print(",");
  Serial.println(rightArm ? 1 : 0);
}

void setup() {
  Serial.begin(115200);

  pinMode(BTN_GRIP, INPUT_PULLUP);
  pinMode(BTN_ARM, INPUT_PULLUP);

  analogReadResolution(12);

  delay(1000);
  Serial.println("BAXTER_ESP32_READY");
}

void loop() {
  readButtons();

  if (millis() - lastSend >= SEND_PERIOD_MS) {
    lastSend = millis();
    sendCommand();
  }

  delay(2);
}
