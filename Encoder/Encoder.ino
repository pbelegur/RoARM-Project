// uses ESP32C3 and make manual connections to the LHS via pin 5 and pin 6

#include <Arduino.h>

#define A1 18    // Encoder 1 channel A
#define B1 19    // Encoder 1 channel B
#define A2 5     // Encoder 2 channel A
#define B2 6     // Encoder 2 channel B
#define CPR 2400 // counts per revolution (600 PPR × 4 edges)

volatile long pos1 = 0;
volatile long pos2 = 0;
long last_full_rot1 = 0;
long last_full_rot2 = 0;
long rev_count1 = 0;
long rev_count2 = 0;

// --- Encoder 1 ISR ---
void IRAM_ATTR handleA1() {
  int a = digitalRead(A1);
  int b = digitalRead(B1);
  if (a == b) pos1++; else pos1--;
}

// --- Encoder 2 ISR ---
void IRAM_ATTR handleA2() {
  int a = digitalRead(A2);
  int b = digitalRead(B2);
  if (a == b) pos2++; else pos2--;
}

void setup() {
  Serial.begin(115200);

  // Encoder 1 setup
  pinMode(A1, INPUT_PULLUP);
  pinMode(B1, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(A1), handleA1, CHANGE);

  // Encoder 2 setup
  pinMode(A2, INPUT_PULLUP);
  pinMode(B2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(A2), handleA2, CHANGE);

  Serial.println("Dual Encoder Verification Started!");
  Serial.println("Rotate slowly for best accuracy.\n");
}

void loop() {
  static uint32_t lastPrint = 0;
  uint32_t now = millis();
  if (now - lastPrint >= 200) {

    // --- Encoder 1 calculations ---
    long wrapped1 = ((pos1 % CPR) + CPR) % CPR;
    float angle1 = (wrapped1 / (float)CPR) * 360.0f;
    long diff1 = pos1 - last_full_rot1;
    if (abs(diff1) >= CPR) {
      rev_count1 += (diff1 > 0) ? 1 : -1;
      last_full_rot1 = pos1;
      Serial.printf("*** ENCODER 1 FULL ROTATION #%ld ***\n", rev_count1);
    }

    // --- Encoder 2 calculations ---
    long wrapped2 = ((pos2 % CPR) + CPR) % CPR;
    float angle2 = (wrapped2 / (float)CPR) * 360.0f;
    long diff2 = pos2 - last_full_rot2;
    if (abs(diff2) >= CPR) {
      rev_count2 += (diff2 > 0) ? 1 : -1;
      last_full_rot2 = pos2;
      Serial.printf("*** ENCODER 2 FULL ROTATION #%ld ***\n", rev_count2);
    }

    // --- Print both encoders' data ---
    Serial.printf(
      "E1 -> pos=%ld  angle=%.2f°  revs=%ld | "
      "E2 -> pos=%ld  angle=%.2f°  revs=%ld\n",
      pos1, angle1, rev_count1,
      pos2, angle2, rev_count2
    );

    lastPrint = now;
  }
}
