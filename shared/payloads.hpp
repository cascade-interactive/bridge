#pragma once
#include "packet.hpp"

#pragma pack(push, 1)

// Core protocol payloads (0x00–0x1F)

// HEARTBEAT (0x00)
struct HeartbeatPayload {
  uint32_t uptime_ms;  // sender uptime in milliseconds
};

// ACK (0x01) sent in response to any packet with FLAG_ACK_REQ set
struct AckPayload {
  uint32_t ack_sequence;  // sequence number of the packet being acknowledged
  uint8_t ack_device;     // device_id of the packet being acknowledged
  uint8_t status;         // 0 = ok, nonzero = node-defined error code
};

// LOG (0x02)
struct LogPayload {
  uint8_t level;     // 0=debug  1=info  2=warn  3=error
  char message[59];  // null-terminated, 59 bytes keeps struct at 60B
};

// Sensor agnostic payloads (0x20–0x3F)

// POSITION (0x20)
struct PositionPayload {
  double x, y, z;
  FrameID frame_id;
};

// VELOCITY (0x21)
struct VelocityPayload {
  float vx, vy, vz;  // m/s
  FrameID frame_id;
};

// ACCELERATION (0x22)
struct AccelerationPayload {
  float ax, ay, az;  // m/s²
  FrameID frame_id;
};

// ORIENTATION (0x23)
// Always body -- world.
struct OrientationPayload {
  float x, y, z, w;  // scalar-last quaternion
};

// ANGULAR_RATE (0x24)
struct AngularRatePayload {
  float wx, wy, wz;  // rad/s
  FrameID frame_id;
};

// DRIVER (0x25) single-channel generic
// For multi-channel driver groups (fins, wheels) use a project-defined type.
struct DriverPayload {
  uint8_t driver_id;  // which driver this packet describes
  float command;      // desired position units are project-defined
  float feedback;     // actual position if known, NAN if not available
};

#pragma pack(pop)