#include <Wire.h>
#include "ICM_20948.h"

// -------------------------
// Pins / Buses
// -------------------------
static const int SDA_A = 41;
static const int SCL_A = 40;

static const int SDA_B = 7;
static const int SCL_B = 6;

// Bus objects
TwoWire &BUS_A = Wire;     // I2C0
TwoWire &BUS_B = Wire1;    // I2C1

// Addresses
static const uint8_t ADDR_68 = 0x68;
static const uint8_t ADDR_69 = 0x69;

// Labels
static const char *NAME_A1 = "A_41_40_68";
static const char *NAME_A2 = "A_41_40_69";
static const char *NAME_B3 = "B_7_6_68";

// -------------------------
// SparkFun DMP (Bus A)
// -------------------------
ICM_20948_I2C imuA1;
ICM_20948_I2C imuA2;

// IMPORTANT: In SparkFun lib, the DMP data struct type is lowercase:
icm_20948_DMP_data_t dmpDataA1;
icm_20948_DMP_data_t dmpDataA2;

// -------------------------
// Raw I2C for Bus B + Madgwick
// (we avoid SparkFun begin() on Wire1 since it was failing for you)
// -------------------------

// Minimal register constants (names chosen to avoid collisions with SparkFun enums)
static const uint8_t ICM_BANK_SEL_REG   = 0x7F;
static const uint8_t ICM_WHO_AM_I_REG   = 0x00; // Bank 0 -> should read 0xEA

static const uint8_t ICM_PWR_MGMT_1     = 0x06; // Bank 0
static const uint8_t ICM_PWR_MGMT_2     = 0x07; // Bank 0

// Bank 0 data regs
static const uint8_t ICM_ACCEL_XOUT_H   = 0x2D; // Bank 0, start of accel/gyro block

// Bank 2 config regs
static const uint8_t ICM_GYRO_CONFIG_1  = 0x01; // Bank 2
static const uint8_t ICM_ACCEL_CONFIG   = 0x14; // Bank 2

// -------------------------
// DMP init helper (Bus A)
// -------------------------
static bool initDMPQuat6(ICM_20948_I2C &imu, TwoWire &bus, uint8_t addr, const char *name)
{
  Serial.print("\nInit "); Serial.print(name);
  Serial.print(" @0x"); Serial.println(addr, HEX);

  // SparkFun ICM-20948 library I2C begin typically wants AD0 value (0 or 1),
  // not the full 0x68/0x69 address.
  // 0x68 -> AD0=0, 0x69 -> AD0=1
  uint8_t ad0 = (addr == 0x69) ? 1 : 0;

  imu.begin(bus, ad0);

  if (imu.status != ICM_20948_Stat_Ok) {
    Serial.print("  ❌ imu.begin failed, status=");
    Serial.println((int)imu.status);
    return false;
  }

  // Start the DMP
  ICM_20948_Status_e st;

  st = imu.initializeDMP();
  if (st != ICM_20948_Stat_Ok) {
    Serial.print("  ❌ initializeDMP failed, status=");
    Serial.println((int)st);
    return false;
  }

  // Enable Quat6 (game rotation vector)
  st = imu.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR);
  if (st != ICM_20948_Stat_Ok) {
    Serial.print("  ❌ enableDMPSensor(Quat6) failed, status=");
    Serial.println((int)st);
    return false;
  }

  // Set output data rate for Quat6
  // 0 is typically the fastest rate supported by the DMP for that sensor
  st = imu.setDMPODRrate(DMP_ODR_Reg_Quat6, 0);
  if (st != ICM_20948_Stat_Ok) {
    Serial.print("  ❌ setDMPODRrate(Quat6) failed, status=");
    Serial.println((int)st);
    return false;
  }

  // Enable FIFO + DMP
  st = imu.enableFIFO();
  if (st != ICM_20948_Stat_Ok) {
    Serial.print("  ❌ enableFIFO failed, status=");
    Serial.println((int)st);
    return false;
  }

  st = imu.enableDMP();
  if (st != ICM_20948_Stat_Ok) {
    Serial.print("  ❌ enableDMP failed, status=");
    Serial.println((int)st);
    return false;
  }

  // Clean start
  imu.resetFIFO();
  imu.resetDMP();

  Serial.println("  ✅ DMP Quat6 ready");
  return true;
}

// -------------------------
// Madgwick quaternion for Bus B
// -------------------------
struct MadgwickIMU {
  // quaternion (w,x,y,z)
  float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
  float beta = 0.10f; // tune if needed (0.05-0.2 typical)

  void updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;

    float norm = ax * ax + ay * ay + az * az;
    if (norm <= 0.0f) return;
    recipNorm = 1.0f / sqrtf(norm);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0;
    float _4q1 = 4.0f * q1;
    float _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1;
    float _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    norm = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
    if (norm > 0.0f) {
      recipNorm = 1.0f / sqrtf(norm);
      s0 *= recipNorm; s1 *= recipNorm; s2 *= recipNorm; s3 *= recipNorm;

      qDot1 -= beta * s0;
      qDot2 -= beta * s1;
      qDot3 -= beta * s2;
      qDot4 -= beta * s3;
    }

    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    norm = q0*q0 + q1*q1 + q2*q2 + q3*q3;
    recipNorm = 1.0f / sqrtf(norm);
    q0 *= recipNorm; q1 *= recipNorm; q2 *= recipNorm; q3 *= recipNorm;
  }
};

MadgwickIMU filtB;
uint32_t lastMicrosB = 0;

// -------------- Raw I2C helpers (Bus B) --------------
static bool i2cWriteByte(TwoWire &bus, uint8_t addr, uint8_t reg, uint8_t val) {
  bus.beginTransmission(addr);
  bus.write(reg);
  bus.write(val);
  return (bus.endTransmission() == 0);
}

static bool i2cReadBytes(TwoWire &bus, uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  bus.beginTransmission(addr);
  bus.write(reg);
  if (bus.endTransmission(false) != 0) return false;
  uint8_t got = bus.requestFrom((int)addr, (int)n);
  if (got != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = bus.read();
  return true;
}

static bool icmSelectBank(TwoWire &bus, uint8_t addr, uint8_t bank) {
  return i2cWriteByte(bus, addr, ICM_BANK_SEL_REG, (uint8_t)(bank << 4));
}

static uint8_t icmReadWhoAmI_B(TwoWire &bus, uint8_t addr) {
  uint8_t v = 0xFF;
  icmSelectBank(bus, addr, 0);
  i2cReadBytes(bus, addr, ICM_WHO_AM_I_REG, &v, 1);
  return v;
}

static bool initICM_B_rawQuat(uint8_t addr) {
  if (!icmSelectBank(BUS_B, addr, 0)) return false;
  if (!i2cWriteByte(BUS_B, addr, ICM_PWR_MGMT_1, 0x01)) return false;
  delay(10);
  if (!i2cWriteByte(BUS_B, addr, ICM_PWR_MGMT_2, 0x00)) return false;

  if (!icmSelectBank(BUS_B, addr, 2)) return false;
  if (!i2cWriteByte(BUS_B, addr, ICM_GYRO_CONFIG_1, 0x06)) return false; // 2000 dps
  if (!i2cWriteByte(BUS_B, addr, ICM_ACCEL_CONFIG, 0x02)) return false;  // 4g

  if (!icmSelectBank(BUS_B, addr, 0)) return false;

  uint8_t who = icmReadWhoAmI_B(BUS_B, addr);
  return (who == 0xEA);
}

static bool readAccelGyro_B(uint8_t addr,
                            float &ax_g, float &ay_g, float &az_g,
                            float &gx_rads, float &gy_rads, float &gz_rads)
{
  uint8_t buf[12];
  if (!icmSelectBank(BUS_B, addr, 0)) return false;
  if (!i2cReadBytes(BUS_B, addr, ICM_ACCEL_XOUT_H, buf, 12)) return false;

  auto be16 = [&](int i) -> int16_t {
    return (int16_t)((uint16_t)buf[i] << 8 | (uint16_t)buf[i + 1]);
  };

  int16_t ax = be16(0), ay = be16(2), az = be16(4);
  int16_t gx = be16(6), gy = be16(8), gz = be16(10);

  const float ACC_LSB_PER_G = 8192.0f;
  const float GYRO_LSB_PER_DPS = 16.4f;

  ax_g = (float)ax / ACC_LSB_PER_G;
  ay_g = (float)ay / ACC_LSB_PER_G;
  az_g = (float)az / ACC_LSB_PER_G;

  float gx_dps = (float)gx / GYRO_LSB_PER_DPS;
  float gy_dps = (float)gy / GYRO_LSB_PER_DPS;
  float gz_dps = (float)gz / GYRO_LSB_PER_DPS;

  const float DEG2RAD = 0.01745329251994329577f;
  gx_rads = gx_dps * DEG2RAD;
  gy_rads = gy_dps * DEG2RAD;
  gz_rads = gz_dps * DEG2RAD;

  return true;
}

// -------------------------
// DMP read (Bus A)
// -------------------------
static bool tryPrintDMPQuat(ICM_20948_I2C &imu, icm_20948_DMP_data_t &d, const char *name) {
  ICM_20948_Status_e st = imu.readDMPdataFromFIFO(&d);
  if (st != ICM_20948_Stat_Ok) return false;

  // Only print when this packet actually contains Quat6
  if ((d.header & DMP_header_bitmap_Quat6) == 0) return false;

  const float q30 = 1073741824.0f; // 2^30 (Q30 format)

  float qx = (float)d.Quat6.Data.Q1 / q30;
  float qy = (float)d.Quat6.Data.Q2 / q30;
  float qz = (float)d.Quat6.Data.Q3 / q30;

  float qw2 = 1.0f - (qx*qx + qy*qy + qz*qz);
  float qw  = (qw2 > 0.0f) ? sqrtf(qw2) : 0.0f;

  float q0 = qw, q1 = qx, q2 = qy, q3 = qz;

  Serial.print(name); Serial.print(",");
  Serial.print(q0, 6); Serial.print(",");
  Serial.print(q1, 6); Serial.print(",");
  Serial.print(q2, 6); Serial.print(",");
  Serial.println(q3, 6);

  return true;
}

// -------------------------
// Setup / Loop
// -------------------------
bool okA1=false, okA2=false, okB=false;

void setup() {
  Serial.begin(115200);
  delay(1500);
  Serial.println("\n--- Quaternions: BusA=DMP (2 sensors), BusB=Madgwick (1 sensor) ---");
  Serial.println("Format: name,w,x,y,z");

  // Start buses
  BUS_A.begin(SDA_A, SCL_A, 400000);
  BUS_B.begin(SDA_B, SCL_B, 100000);

  // Init Bus A DMP sensors
  okA1 = initDMPQuat6(imuA1, BUS_A, ADDR_68, NAME_A1);
  okA2 = initDMPQuat6(imuA2, BUS_A, ADDR_69, NAME_A2);

  // Init Bus B raw + Madgwick
  Serial.print("\nInit "); Serial.print(NAME_B3); Serial.print(" @0x68 (raw + Madgwick)\n");
  uint8_t who = icmReadWhoAmI_B(BUS_B, ADDR_68);
  Serial.print("  WHO_AM_I=0x"); Serial.println(who, HEX);

  okB = initICM_B_rawQuat(ADDR_68);
  if (okB) {
    Serial.println("  ✅ Raw accel/gyro ready for Madgwick");
    lastMicrosB = micros();
  } else {
    Serial.println("  ❌ Bus B init failed (but WHO_AM_I may still work).");
  }

  Serial.print("\nInit OK: ");
  Serial.print((int)okA1 + (int)okA2 + (int)okB);
  Serial.println("/3\n");
}

void loop() {
  // ---- Bus A: print quats whenever FIFO has them ----
  if (okA1) (void)tryPrintDMPQuat(imuA1, dmpDataA1, NAME_A1);
  if (okA2) (void)tryPrintDMPQuat(imuA2, dmpDataA2, NAME_A2);

  // ---- Bus B: read accel/gyro and run Madgwick at ~100 Hz ----
  if (okB) {
    uint32_t now = micros();
    float dt = (now - lastMicrosB) * 1e-6f;
    if (dt >= 0.01f) {
      lastMicrosB = now;

      float ax, ay, az, gx, gy, gz;
      if (readAccelGyro_B(ADDR_68, ax, ay, az, gx, gy, gz)) {
        filtB.updateIMU(gx, gy, gz, ax, ay, az, dt);

        Serial.print(NAME_B3); Serial.print(",");
        Serial.print(filtB.q0, 6); Serial.print(",");
        Serial.print(filtB.q1, 6); Serial.print(",");
        Serial.print(filtB.q2, 6); Serial.print(",");
        Serial.println(filtB.q3, 6);
      }
    }
  }

  delay(1);
}
