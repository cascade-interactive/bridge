#pragma once
#include <cstdint>

enum class SimPayloadType : uint8_t {
  PHYSICS   = 0x40,
  BAROMETER = 0x41,
};

#pragma pack(push, 1)

struct PhysicsStatePayload {
  double pos_x, pos_y, pos_z;
  float vel_x, vel_y, vel_z;
  float accel_x, accel_y, accel_z;
  float quat_x, quat_y, quat_z, quat_w;
  float gyro_x, gyro_y, gyro_z;
};

struct BarometerPayload {
  float pressure_pa;
  float temperature_c;
};

#pragma pack(pop)