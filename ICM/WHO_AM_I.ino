// Scan 2 I2C buses for connected addresses
#include <Wire.h>

// Define I2C bus parameters
#define BUS1_SDA 41
#define BUS1_SCL 40
#define BUS2_SDA 7
#define BUS2_SCL 

TwoWire I2C_BUS1 = TwoWire(0);
TwoWire I2C_BUS2 = TwoWire(1);

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Initialize I2C buses
  I2C_BUS1.begin(BUS1_SDA, BUS1_SCL, 100000);  // Bus 1 on pins 21, 22
  I2C_BUS2.begin(BUS2_SDA, BUS2_SCL, 100000);  // Bus 2 on pins 32, 33
  
  Serial.println("I2C Bus Scanner - Scanning 2 buses...\n");
}

void loop() {
  scanBus(&I2C_BUS1, "Bus 1");
  delay(500);
  scanBus(&I2C_BUS2, "Bus 2");
  delay(2000);
}

void scanBus(TwoWire* bus, const char* busName) {
  Serial.print(busName);
  Serial.println(" scan:");
  
  int count = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    bus->beginTransmission(addr);
    if (bus->endTransmission() == 0) {
      Serial.print("  Device found at address 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      count++;
    }
  }
  
  if (count == 0) {
    Serial.println("  No devices found");
  } else {
    Serial.print("  Total: ");
    Serial.print(count);
    Serial.println(" device(s)\n");
  }
}



// WHO AM I MORE DETAILED
#include <Wire.h>

// -------------------------
// Two I2C buses
// -------------------------
TwoWire I2C_A = TwoWire(0);  // Bus A: SDA=41 SCL=40
TwoWire I2C_B = TwoWire(1);  // Bus B: SDA=7  SCL=6

static const int SDA_A = 41;
static const int SCL_A = 40;

static const int SDA_B = 7;
static const int SCL_B = 6;

// ICM-20948
#define ICM_ADDR_68   0x68
#define ICM_ADDR_69   0x69
#define ICM_WHO_AM_I  0x00   // expected 0xEA

uint8_t readRegister8(TwoWire &bus, uint8_t address, uint8_t reg) {
  bus.beginTransmission(address);
  bus.write(reg);
  if (bus.endTransmission(false) != 0) return 0xFF; // no ACK
  if (bus.requestFrom((int)address, 1) != 1) return 0xFF;
  return bus.read();
}

bool devicePresent(TwoWire &bus, uint8_t addr) {
  bus.beginTransmission(addr);
  return (bus.endTransmission() == 0);
}

void scanBus(TwoWire &bus, const char *name) {
  Serial.print("\n--- Scanning ");
  Serial.print(name);
  Serial.println(" ---");

  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    if (devicePresent(bus, addr)) {
      Serial.print("Found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      found++;
    }
  }
  if (!found) Serial.println("No devices found.");
  else {
    Serial.print("Total devices: ");
    Serial.println(found);
  }
}

void checkICM(TwoWire &bus, uint8_t addr, const char *label) {
  Serial.print(label);
  Serial.print(" @0x");
  if (addr < 16) Serial.print("0");
  Serial.print(addr, HEX);

  if (!devicePresent(bus, addr)) {
    Serial.println(" -> NOT PRESENT");
    return;
  }

  uint8_t id = readRegister8(bus, addr, ICM_WHO_AM_I);
  Serial.print(" WHO_AM_I=0x");
  if (id < 16) Serial.print("0");
  Serial.println(id, HEX);

  if (id == 0xEA) Serial.println("  SUCCESS: ICM-20948 detected");
  else if (id == 0xFF) Serial.println("  ERROR: read failed");
  else Serial.println("  ERROR: wrong device/chip");
}

void setup() {
  Serial.begin(115200);
  unsigned long start = millis();
  while (!Serial && millis() - start < 3000) delay(10);
  delay(200);

  Serial.println("\nQT Py ESP32-S3: 3x ICM-20948 bring-up");

  // Start both I2C buses
  I2C_A.begin(SDA_A, SCL_A, 400000);
  I2C_B.begin(SDA_B, SCL_B, 400000);

  // Initial scans
  scanBus(I2C_A, "Bus A (SDA=41 SCL=40)");
  scanBus(I2C_B, "Bus B (SDA=7  SCL=6)");

  Serial.println("\n--- WHO_AM_I checks (expect 0xEA) ---");
  checkICM(I2C_A, ICM_ADDR_68, "Bus A Sensor #1");
  checkICM(I2C_A, ICM_ADDR_69, "Bus A Sensor #2");
  checkICM(I2C_B, ICM_ADDR_68, "Bus B Sensor #3");
}

void loop() {
  delay(2000);

  Serial.println("\n[loop] re-check all sensors");
  checkICM(I2C_A, ICM_ADDR_68, "Bus A Sensor #1");
  checkICM(I2C_A, ICM_ADDR_69, "Bus A Sensor #2");
  checkICM(I2C_B, ICM_ADDR_68, "Bus B Sensor #3");
}
