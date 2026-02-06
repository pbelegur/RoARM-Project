#include <Wire.h>

static const int SDA_A = 41;
static const int SCL_A = 40;
static const uint8_t ADDR = 0x68;

void setup() {
  Serial.begin(115200);
  delay(1200);

  Serial.println("\n--- I2C Ping Test: SDA=41 SCL=40, addr=0x68 ---");
  Wire.begin(SDA_A, SCL_A, 400000);
  delay(50);

  Wire.beginTransmission(ADDR);
  uint8_t err = Wire.endTransmission();

  if (err == 0) {
    Serial.println("✅ Device ACKed at 0x68 (sensor is responding on this bus)");
  } else {
    Serial.print("❌ No ACK at 0x68. I2C error code = ");
    Serial.println(err);
  }
}

void loop() {}


// 0x69
#include <Wire.h>

static const int SDA_A = 41;
static const int SCL_A = 40;
static const uint8_t ADDR = 0x69;

void setup() {
  Serial.begin(115200);
  delay(1200);

  Serial.println("\n--- I2C Ping Test: SDA=41 SCL=40, addr=0x68 ---");
  Wire.begin(SDA_A, SCL_A, 400000);
  delay(50);

  Wire.beginTransmission(ADDR);
  uint8_t err = Wire.endTransmission();

  if (err == 0) {
    Serial.println("✅ Device ACKed at 0x68 (sensor is responding on this bus)");
  } else {
    Serial.print("❌ No ACK at 0x68. I2C error code = ");
    Serial.println(err);
  }
}

void loop() {}
