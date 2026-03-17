#pragma once
#include <stdint.h>

struct EntityState {

  double x, y, z;  // position

  float vx, vy, vz;  // velocity
  float ax, ay, az;  // acceleration

  float ox, oy, oz, ow;  // orientation (quaternion)

  float wx, wy, wz;  // angular rate

  uint64_t last_pos_update_us;
  uint64_t last_vel_update_us;
  uint64_t last_acc_update_us;
  uint64_t last_ori_update_us;
  uint64_t last_angular_rate_update_us;

  uint8_t custom_data[256][64];
  bool has_custom_data[256];
  uint64_t custom_data_timestamp_us[256];
};

struct WorldState {

  EntityState entities[8];
  bool active_entities[8];
};