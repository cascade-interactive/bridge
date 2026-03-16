#pragma once
#include "packet.hpp"

// All payload structs live here. The rule is simple:
//   - Core protocol payloads (HEARTBEAT, ACK, LOG) are at the top.
//   - Common physical quantity payloads (POSITION, VELOCITY, etc.) follow.
//   - Project-specific payloads belong in a separate file (e.g. sim_payloads.hpp)
//     that includes this one and adds structs in the 0x40+ range.
//
// Quaternion convention throughout: scalar-last (x, y, z, w).
// This matches Unity and UE5 natively -- no conversion needed at the endpoints.
//
// Position uses double for precision (GPS needs it).
// Everything else uses float -- physical noise floors are well within float range.

#pragma pack(1)

// ── Core protocol payloads (0x00–0x1F) ───────────────────────────────────────

// HEARTBEAT (0x00) -- keepalive, sent by any node at a low fixed rate
struct HeartbeatPayload {
  uint32_t uptime_ms;  // sender uptime in milliseconds
};

// ACK (0x01) -- sent in response to any packet with FLAG_ACK_REQ set
struct AckPayload {
  uint32_t ack_sequence;  // sequence number of the packet being acknowledged
  uint8_t ack_device;     // device_id of the packet being acknowledged
  uint8_t status;         // 0 = ok, nonzero = node-defined error code
};

// LOG (0x02) -- debug string, human readable, never used in production logic
struct LogPayload {
  uint8_t level;     // 0=debug  1=info  2=warn  3=error
  char message[59];  // null-terminated, 59 bytes keeps struct at 60B
};

// ── Common physical quantity payloads (0x20–0x3F) ─────────────────────────────
// These describe what a body IS doing, not what sensor measured it.
// A barometer, GPS, and wheel odometry all produce position -- same type on wire.

// POSITION (0x20)
struct PositionPayload {
  double x, y, z;    // double -- GPS lat/lon loses ~1m precision with float
  FrameID frame_id;  // FrameID -- always specify, never assume
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
// Always body→world. No frame_id needed -- convention is fixed protocol-wide.
struct OrientationPayload {
  float x, y, z, w;  // scalar-last quaternion (Unity/UE5 native)
};

// ANGULAR_RATE (0x24)
struct AngularRatePayload {
  float wx, wy, wz;  // rad/s
  FrameID frame_id;
};

// ACTUATOR (0x25) -- single-channel generic
// For multi-channel actuator groups (fins, wheels) use a project-defined type.
struct ActuatorPayload {
  uint8_t actuator_id;  // which actuator this packet describes
  float command;        // desired position -- units are project-defined
  float feedback;       // actual position if known, NAN if not available
};

#pragma pack()