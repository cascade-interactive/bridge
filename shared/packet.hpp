#pragma once
#include <cstdint>
#include <cstdio>
#include <cstring>

// ── Identity ──────────────────────────────────────────────────────────────────

static constexpr uint32_t PACKET_MAGIC  = 0x4C594E4E;
static constexpr uint8_t PACKET_VERSION = 0x01;

// ── Flags ─────────────────────────────────────────────────────────────────────

enum PacketFlags : uint8_t {
  FLAG_NONE      = 0x00,
  FLAG_ACK_REQ   = 0x01,  // sender wants an ack back
  FLAG_IS_ACK    = 0x02,  // this packet is the ack
  FLAG_BROADCAST = 0x04,  // bridge forwards to all targets, ignore filters
  FLAG_NO_CRC    = 0x08,  // crc32 is 0x00000000, skip compute and verify
};

// ── Payload type ──────────────────────────────────────────────────────────────

enum class PayloadType : uint8_t {

  // Core protocol (0x00–0x1F) -- project-agnostic, never change
  HEARTBEAT = 0x00,
  ACK       = 0x01,
  LOG       = 0x02,

  // Common physical quantities (0x20–0x3F) -- quantities, not sensors
  // These are what a thing IS doing, not what hardware measured it
  POSITION     = 0x20,
  VELOCITY     = 0x21,
  ACCELERATION = 0x22,
  ORIENTATION  = 0x23,  // always body→world, scalar-last quaternion
  ANGULAR_RATE = 0x24,
  ACTUATOR     = 0x25,  // single-channel generic

  // Project-defined (0x40–0xFF) -- each project owns this range entirely
  PROJECT_BASE = 0x40,

  UNKNOWN = 0xFF,
};

// ── Device ID ─────────────────────────────────────────────────────────────────

enum class DeviceID : uint8_t {
  SIM    = 0x00,
  BRIDGE = 0x01,
  ESP_0  = 0x02,
  ESP_1  = 0x03,
  ESP_2  = 0x04,
  ESP_3  = 0x05,
};

// ── Reference frame ───────────────────────────────────────────────────────────

enum class FrameID : uint8_t {
  BODY  = 0x00,  // relative to vehicle
  WORLD = 0x01,  // arbitrary world origin
  NED   = 0x02,  // North-East-Down
  ECEF  = 0x03,  // Earth-Centered Earth-Fixed
};

// ── Header (24 bytes) ─────────────────────────────────────────────────────────

#pragma pack(push, 1)
struct PacketHeader {
  uint32_t magic;         // 4B -- always PACKET_MAGIC
  uint8_t version;        // 1B -- PACKET_VERSION
  uint8_t payload_type;   // 1B -- PayloadType
  uint8_t device_id;      // 1B -- DeviceID of sender
  uint8_t flags;          // 1B -- PacketFlags bitmask
  uint32_t sequence;      // 4B -- rolling counter per device, never reused
  uint16_t length;        // 2B -- payload size in bytes only
  uint16_t reserved;      // 2B -- zero for now, free for future use
  uint64_t timestamp_us;  // 8B -- sender-side sim clock, microseconds
};
#pragma pack(pop)

static_assert(sizeof(PacketHeader) == 24, "PacketHeader must be 24 bytes");

// ── Template packet wrapper ───────────────────────────────────────────────────
// Wire layout: [PacketHeader 24B] [T payload] [uint32_t crc32 4B]
// CRC covers header + payload bytes only, not the crc32 field itself.
// If FLAG_NO_CRC is set, crc32 is always 0x00000000 on the wire.
#pragma pack(push, 1)
template <typename T>
struct Packet {
  PacketHeader header;
  T payload;
  uint32_t crc32;
};
#pragma pack(pop)

// ── CRC32 (Ethernet/ZIP polynomial 0xEDB88320) ────────────────────────────────

inline uint32_t crc32Compute(const void* data, size_t length) {
  const auto* buf = static_cast<const uint8_t*>(data);
  uint32_t crc    = 0xFFFFFFFFu;
  for (size_t i = 0; i < length; ++i) {
    crc ^= buf[i];
    for (int j = 0; j < 8; ++j)
      crc = (crc >> 1) ^ (0xEDB88320u & -(crc & 1u));
  }
  return ~crc;
}

template <typename T>
inline uint32_t packetCRC(const Packet<T>& p) {
  // CRC over header + payload, not the crc32 field
  return crc32Compute(&p.header, sizeof(PacketHeader) + sizeof(T));
}

// ── Build ─────────────────────────────────────────────────────────────────────

template <typename T>
inline Packet<T> buildPacket(const T& payload, PayloadType type,
                             DeviceID device, uint8_t flags, uint32_t sequence,
                             uint64_t timestamp_us) {
  Packet<T> p{};
  p.header.magic        = PACKET_MAGIC;
  p.header.version      = PACKET_VERSION;
  p.header.payload_type = static_cast<uint8_t>(type);
  p.header.device_id    = static_cast<uint8_t>(device);
  p.header.flags        = flags;
  p.header.sequence     = sequence;
  p.header.length       = static_cast<uint16_t>(sizeof(T));
  p.header.reserved     = 0x0000;
  p.header.timestamp_us = timestamp_us;
  p.payload             = payload;
  p.crc32               = (flags & FLAG_NO_CRC) ? 0x00000000u : packetCRC(p);
  return p;
}

// ── Validate ──────────────────────────────────────────────────────────────────

inline bool validatePacket(const uint8_t* buf, int size, bool verbose = true) {
  if (size < static_cast<int>(sizeof(PacketHeader))) {
    if (verbose)
      printf("[packet] Too small (%d bytes)\n", size);
    return false;
  }

  PacketHeader h{};
  std::memcpy(&h, buf, sizeof(PacketHeader));

  if (h.magic != PACKET_MAGIC) {
    if (verbose)
      printf("[packet] Bad magic 0x%08X (expected 0x%08X)\n", h.magic,
             PACKET_MAGIC);
    return false;
  }
  if (h.version != PACKET_VERSION) {
    if (verbose)
      printf("[packet] Bad version %u (expected %u)\n", h.version,
             PACKET_VERSION);
    return false;
  }

  const int expected =
      static_cast<int>(sizeof(PacketHeader) + h.length + sizeof(uint32_t));
  if (expected != size) {
    if (verbose)
      printf("[packet] Length mismatch (expected %d, got %d)\n", expected,
             size);
    return false;
  }

  if (!(h.flags & FLAG_NO_CRC)) {
    uint32_t computed = crc32Compute(buf, size - sizeof(uint32_t));
    uint32_t received = 0;
    std::memcpy(&received, buf + size - sizeof(uint32_t), sizeof(uint32_t));
    if (computed != received) {
      if (verbose)
        printf("[packet] CRC mismatch (computed 0x%08X, got 0x%08X)\n",
               computed, received);
      return false;
    }
  }

  return true;
}

// ── Extraction helpers ────────────────────────────────────────────────────────

inline PacketHeader extractHeader(const uint8_t* buf) {
  PacketHeader h{};
  std::memcpy(&h, buf, sizeof(PacketHeader));
  return h;
}

template <typename T>
inline T extractPayload(const uint8_t* buf) {
  T payload{};
  std::memcpy(&payload, buf + sizeof(PacketHeader), sizeof(T));
  return payload;
}