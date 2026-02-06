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
static const char *NAME_B1 = "B_7_6_68";
static const char *NAME_B2 = "B_7_6_69";

// -------------------------
// SparkFun DMP (Bus A)
// -------------------------
ICM_20948_I2C imuA1;
ICM_20948_I2C imuA2;

icm_20948_DMP_data_t dmpDataA1;
icm_20948_DMP_data_t dmpDataA2;

// -------------------------
// Raw I2C regs (Bus B)
// -------------------------
static const uint8_t ICM_BANK_SEL_REG   = 0x7F;
static const uint8_t ICM_WHO_AM_I_REG   = 0x00; // Bank 0 -> 0xEA

static const uint8_t ICM_PWR_MGMT_1     = 0x06; // Bank 0
static const uint8_t ICM_PWR_MGMT_2     = 0x07; // Bank 0
static const uint8_t ICM_ACCEL_XOUT_H   = 0x2D; // Bank 0

static const uint8_t ICM_GYRO_CONFIG_1  = 0x01; // Bank 2
static const uint8_t ICM_ACCEL_CONFIG   = 0x14; // Bank 2

// -------------------------
// DMP init helper (Bus A)
// -------------------------
static bool initDMPQuat6(ICM_20948_I2C &imu, TwoWire &bus, uint8_t addr, const char *name)
{
  Serial.print("\nInit "); Serial.print(name);
  Serial.print(" @0x"); Serial.println(addr, HEX);

  uint8_t ad0 = (addr == 0x69) ? 1 : 0;   // SparkFun wants AD0, not 0x68/0x69
  imu.begin(bus, ad0);

  if (imu.status != ICM_20948_Stat_Ok) {
    Serial.print("  ❌ imu.begin failed, status=");
    Serial.println((int)imu.status);
    return false;
  }

  ICM_20948_Status_e st;

  st = imu.initializeDMP();
  if (st != ICM_20948_Stat_Ok) { Serial.println("  ❌ initializeDMP failed"); return false; }

  st = imu.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR);
  if (st != ICM_20948_Stat_Ok) { Serial.println("  ❌ enableDMPSensor(Quat6) failed"); return false; }

  st = imu.setDMPODRrate(DMP_ODR_Reg_Quat6, 0);
  if (st != ICM_20948_Stat_Ok) { Serial.println("  ❌ setDMPODRrate(Quat6) failed"); return false; }

  st = imu.enableFIFO();
  if (st != ICM_20948_Stat_Ok) { Serial.println("  ❌ enableFIFO failed"); return false; }

  st = imu.enableDMP();
  if (st != ICM_20948_Stat_Ok) { Serial.println("  ❌ enableDMP failed"); return false; }

  imu.resetFIFO();
  imu.resetDMP();

  Serial.println("  ✅ DMP Quat6 ready");
  return true;
}

static bool tryPrintDMPQuat(ICM_20948_I2C &imu, icm_20948_DMP_data_t &d, const char *name) {
  ICM_20948_Status_e st = imu.readDMPdataFromFIFO(&d);
  if (st != ICM_20948_Stat_Ok) return false;

  if ((d.header & DMP_header_bitmap_Quat6) == 0) return false;

  const float q30 = 1073741824.0f; // 2^30
  float qx = (float)d.Quat6.Data.Q1 / q30;
  float qy = (float)d.Quat6.Data.Q2 / q30;
  float qz = (float)d.Quat6.Data.Q3 / q30;

  float qw2 = 1.0f - (qx*qx + qy*qy + qz*qz);
  float qw  = (qw2 > 0.0f) ? sqrtf(qw2) : 0.0f;

  Serial.print(name); Serial.print(",");
  Serial.print(qw, 6); Serial.print(",");
  Serial.print(qx, 6); Serial.print(",");
  Serial.print(qy, 6); Serial.print(",");
  Serial.println(qz, 6);
  return true;
}

// -------------------------
// Madgwick IMU (no mag) for Bus B
// -------------------------
struct MadgwickIMU {
  float q0=1, q1=0, q2=0, q3=0;
  float beta = 0.10f;

  void updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;

    float norm = ax*ax + ay*ay + az*az;
    if (norm <= 0.0f) return;
    recipNorm = 1.0f / sqrtf(norm);
    ax *= recipNorm; ay *= recipNorm; az *= recipNorm;

    qDot1 = 0.5f * (-q1*gx - q2*gy - q3*gz);
    qDot2 = 0.5f * ( q0*gx + q2*gz - q3*gy);
    qDot3 = 0.5f * ( q0*gy - q1*gz + q3*gx);
    qDot4 = 0.5f * ( q0*gz + q1*gy - q2*gx);

    float _2q0 = 2.0f*q0, _2q1 = 2.0f*q1, _2q2 = 2.0f*q2, _2q3 = 2.0f*q3;
    float _4q0 = 4.0f*q0, _4q1 = 4.0f*q1, _4q2 = 4.0f*q2;
    float _8q1 = 8.0f*q1, _8q2 = 8.0f*q2;
    float q0q0=q0*q0, q1q1=q1*q1, q2q2=q2*q2, q3q3=q3*q3;

    s0 = _4q0*q2q2 + _2q2*ax + _4q0*q1q1 - _2q1*ay;
    s1 = _4q1*q3q3 - _2q3*ax + 4.0f*q0q0*q1 - _2q0*ay - _4q1 + _8q1*q1q1 + _8q1*q2q2 + _4q1*az;
    s2 = 4.0f*q0q0*q2 + _2q0*ax + _4q2*q3q3 - _2q3*ay - _4q2 + _8q2*q1q1 + _8q2*q2q2 + _4q2*az;
    s3 = 4.0f*q1q1*q3 - _2q1*ax + 4.0f*q2q2*q3 - _2q2*ay;

    norm = s0*s0 + s1*s1 + s2*s2 + s3*s3;
    if (norm > 0.0f) {
      recipNorm = 1.0f / sqrtf(norm);
      s0*=recipNorm; s1*=recipNorm; s2*=recipNorm; s3*=recipNorm;
      qDot1 -= beta*s0; qDot2 -= beta*s1; qDot3 -= beta*s2; qDot4 -= beta*s3;
    }

    q0 += qDot1*dt; q1 += qDot2*dt; q2 += qDot3*dt; q3 += qDot4*dt;

    norm = q0*q0 + q1*q1 + q2*q2 + q3*q3;
    recipNorm = 1.0f / sqrtf(norm);
    q0*=recipNorm; q1*=recipNorm; q2*=recipNorm; q3*=recipNorm;
  }
};

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
  for (uint8_t i=0;i<n;i++) buf[i]=bus.read();
  return true;
}

static bool icmSelectBank(TwoWire &bus, uint8_t addr, uint8_t bank) {
  return i2cWriteByte(bus, addr, ICM_BANK_SEL_REG, (uint8_t)(bank << 4));
}

static uint8_t icmReadWhoAmI(TwoWire &bus, uint8_t addr) {
  uint8_t v=0xFF;
  icmSelectBank(bus, addr, 0);
  i2cReadBytes(bus, addr, ICM_WHO_AM_I_REG, &v, 1);
  return v;
}

static bool initICM_raw(TwoWire &bus, uint8_t addr) {
  if (!icmSelectBank(bus, addr, 0)) return false;
  if (!i2cWriteByte(bus, addr, ICM_PWR_MGMT_1, 0x01)) return false;
  delay(10);
  if (!i2cWriteByte(bus, addr, ICM_PWR_MGMT_2, 0x00)) return false;

  if (!icmSelectBank(bus, addr, 2)) return false;
  if (!i2cWriteByte(bus, addr, ICM_GYRO_CONFIG_1, 0x06)) return false; // 2000 dps
  if (!i2cWriteByte(bus, addr, ICM_ACCEL_CONFIG, 0x02)) return false;  // 4g

  if (!icmSelectBank(bus, addr, 0)) return false;

  return (icmReadWhoAmI(bus, addr) == 0xEA);
}

static bool readAccelGyro_raw(TwoWire &bus, uint8_t addr,
                              float &ax_g, float &ay_g, float &az_g,
                              float &gx_rads, float &gy_rads, float &gz_rads)
{
  uint8_t buf[12];
  if (!icmSelectBank(bus, addr, 0)) return false;
  if (!i2cReadBytes(bus, addr, ICM_ACCEL_XOUT_H, buf, 12)) return false;

  auto be16 = [&](int i)->int16_t { return (int16_t)((uint16_t)buf[i]<<8 | (uint16_t)buf[i+1]); };

  int16_t ax = be16(0), ay = be16(2), az = be16(4);
  int16_t gx = be16(6), gy = be16(8), gz = be16(10);

  const float ACC_LSB_PER_G = 8192.0f;   // 4g
  const float GYRO_LSB_PER_DPS = 16.4f;  // 2000 dps
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
// Bus B (two sensors) state
// -------------------------
MadgwickIMU filtB68, filtB69;
uint32_t lastMicrosB68 = 0;
uint32_t lastMicrosB69 = 0;

// -------------------------
// Setup / Loop
// -------------------------
bool okA68=false, okA69=false;
bool okB68=false, okB69=false;

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("\n--- 4x ICM20948 Quaternions ---");
  Serial.println("Bus A (41/40): DMP Quat6 for 0x68 and 0x69");
  Serial.println("Bus B (7/6):   Raw accel/gyro + Madgwick for 0x68 and 0x69");
  Serial.println("Format: name,w,x,y,z");

  BUS_A.begin(SDA_A, SCL_A, 400000);
  BUS_B.begin(SDA_B, SCL_B, 100000);

  // Bus A DMP
  okA68 = initDMPQuat6(imuA1, BUS_A, ADDR_68, NAME_A1);
  okA69 = initDMPQuat6(imuA2, BUS_A, ADDR_69, NAME_A2);

  // Bus B raw init both
  Serial.print("\nInit "); Serial.print(NAME_B1); Serial.println(" (raw+Madgwick)");
  Serial.print("  WHO_AM_I=0x"); Serial.println(icmReadWhoAmI(BUS_B, ADDR_68), HEX);
  okB68 = initICM_raw(BUS_B, ADDR_68);
  if (okB68) lastMicrosB68 = micros();

  Serial.print("\nInit "); Serial.print(NAME_B2); Serial.println(" (raw+Madgwick)");
  Serial.print("  WHO_AM_I=0x"); Serial.println(icmReadWhoAmI(BUS_B, ADDR_69), HEX);
  okB69 = initICM_raw(BUS_B, ADDR_69);
  if (okB69) lastMicrosB69 = micros();

  Serial.print("\nInit OK: ");
  Serial.print((int)okA68 + (int)okA69 + (int)okB68 + (int)okB69);
  Serial.println("/4\n");
}

void loop() {
  // Bus A: DMP
  if (okA68) (void)tryPrintDMPQuat(imuA1, dmpDataA1, NAME_A1);
  if (okA69) (void)tryPrintDMPQuat(imuA2, dmpDataA2, NAME_A2);

  // Bus B: Madgwick for each sensor ~100Hz each
  if (okB68) {
    uint32_t now = micros();
    float dt = (now - lastMicrosB68) * 1e-6f;
    if (dt >= 0.01f) {
      lastMicrosB68 = now;
      float ax,ay,az,gx,gy,gz;
      if (readAccelGyro_raw(BUS_B, ADDR_68, ax,ay,az,gx,gy,gz)) {
        filtB68.updateIMU(gx,gy,gz,ax,ay,az,dt);
        Serial.print(NAME_B1); Serial.print(",");
        Serial.print(filtB68.q0,6); Serial.print(",");
        Serial.print(filtB68.q1,6); Serial.print(",");
        Serial.print(filtB68.q2,6); Serial.print(",");
        Serial.println(filtB68.q3,6);
      }
    }
  }

  if (okB69) {
    uint32_t now = micros();
    float dt = (now - lastMicrosB69) * 1e-6f;
    if (dt >= 0.01f) {
      lastMicrosB69 = now;
      float ax,ay,az,gx,gy,gz;
      if (readAccelGyro_raw(BUS_B, ADDR_69, ax,ay,az,gx,gy,gz)) {
        filtB69.updateIMU(gx,gy,gz,ax,ay,az,dt);
        Serial.print(NAME_B2); Serial.print(",");
        Serial.print(filtB69.q0,6); Serial.print(",");
        Serial.print(filtB69.q1,6); Serial.print(",");
        Serial.print(filtB69.q2,6); Serial.print(",");
        Serial.println(filtB69.q3,6);
      }
    }
  }

  delay(1);
}
