#include <Arduino.h>
#include <cmath>
#include "packet.hpp"
#include "payloads.hpp"
#include "payloads/sim_payloads.hpp"

// ── Config ────────────────────────────────────────────────────────────────────

static constexpr int BAUD_RATE         = 115200;
static constexpr float TARGET_STOP_ALT = 5.0f;  // m -- where we want to stop
static constexpr float SAFETY_MARGIN   = .1f;   // m -- engage early for latency
static constexpr float BRAKE_FORCE     = 7500.0f;  // N -- must match sim
static constexpr float BRAKE_MASS      = 500.0f;   // kg -- must match sim
static constexpr float GRAVITY         = 9.81f;    // m/s²

// Net deceleration once brakes are applied, accounting for gravity still pulling down.
// gross decel = F/m, but gravity opposes braking so net = F/m - g
static constexpr float NET_DECEL =
    (BRAKE_FORCE / BRAKE_MASS) - GRAVITY;  // ~5.19 m/s²

// ── State ─────────────────────────────────────────────────────────────────────

static uint32_t g_seq     = 0;
static float g_brakeForce = 0.0f;
static bool g_braking     = false;

// ── Helpers ───────────────────────────────────────────────────────────────────

static uint64_t nowMicros() {
  return static_cast<uint64_t>(esp_timer_get_time());
}

// Distance needed to stop from current speed at NET_DECEL.
// Derived from v² = u² + 2as, solving for s with v=0:
// s = u² / (2 * a)
static float stoppingDistance(float speed) {
  if (NET_DECEL <= 0.0f)
    return 1e9f;  // can't stop -- infinite distance
  return (speed * speed) / (2.0f * NET_DECEL);
}

// Time to stop from current speed at NET_DECEL.
// Derived from v = u + at, solving for t with v=0:
// t = u / a
static float stoppingTime(float speed) {
  if (NET_DECEL <= 0.0f)
    return 1e9f;
  return speed / NET_DECEL;
}

static void sendActuator(float command) {
  ActuatorPayload payload{};
  payload.actuator_id = 0;
  payload.command     = command;
  payload.feedback    = 0.0f;

  auto packet = buildPacket(payload, PayloadType::ACTUATOR, DeviceID::ESP_0,
                            FLAG_NONE,  // serial -- compute CRC
                            g_seq++, nowMicros());

  const int wireSize =
      sizeof(PacketHeader) + sizeof(ActuatorPayload) + sizeof(uint32_t);
  Serial.write(reinterpret_cast<const uint8_t*>(&packet), wireSize);
}

// ── Controller ────────────────────────────────────────────────────────────────

static void onPhysicsPacket(const PhysicsStatePayload& payload) {
  const float pos   = payload.pos_z;  // meters above ground
  const float vel   = payload.vel_z;  // m/s, negative = falling
  const float speed = fabsf(vel);

  // Not falling or already on the ground -- release and do nothing
  if (vel >= 0.0f || pos <= 0.0f) {
    if (g_braking) {
      Serial.printf("[ctrl] Releasing -- not falling  pos=%.2fm\n", pos);
      g_braking    = false;
      g_brakeForce = 0.0f;
    }
    sendActuator(g_brakeForce);
    return;
  }

  const float stopDist     = stoppingDistance(speed);
  const float stopTime     = stoppingTime(speed);
  const float engageAlt    = TARGET_STOP_ALT + SAFETY_MARGIN + stopDist;
  const float distToTarget = pos - TARGET_STOP_ALT;

  if (stopDist >= distToTarget - SAFETY_MARGIN) {
    // Stopping distance has eaten into our margin -- brake now
    if (!g_braking) {
      Serial.printf(
          "[ctrl] BRAKING  pos=%.2fm  speed=%.2fm/s  "
          "stopDist=%.2fm  estStop=%.2fs  engageAlt=%.2fm\n",
          pos, speed, stopDist, stopTime, engageAlt);
      g_braking    = true;
      g_brakeForce = BRAKE_FORCE;
    }
  } else {
    // Still have room -- coast
    if (g_braking) {
      Serial.printf(
          "[ctrl] Coasting  pos=%.2fm  speed=%.2fm/s  "
          "stopDist=%.2fm  distToTarget=%.2fm\n",
          pos, speed, stopDist, distToTarget);
      g_braking    = false;
      g_brakeForce = 0.0f;
    }
  }

  sendActuator(g_brakeForce);
}

// ── Serial framer ─────────────────────────────────────────────────────────────

static uint8_t g_frameBuf[512];
static int g_frameLen = 0;

static void feedFramer(const uint8_t* data, int size) {
  for (int i = 0; i < size && g_frameLen < (int)sizeof(g_frameBuf); i++)
    g_frameBuf[g_frameLen++] = data[i];

  while (true) {
    if (g_frameLen < (int)sizeof(PacketHeader))
      break;

    // Hunt for magic
    int magicPos = -1;
    for (int i = 0; i <= g_frameLen - 4; i++) {
      uint32_t word;
      memcpy(&word, g_frameBuf + i, 4);
      if (word == PACKET_MAGIC) {
        magicPos = i;
        break;
      }
    }

    if (magicPos < 0) {
      int keep = min(g_frameLen, 3);
      memmove(g_frameBuf, g_frameBuf + g_frameLen - keep, keep);
      g_frameLen = keep;
      break;
    }

    if (magicPos > 0) {
      memmove(g_frameBuf, g_frameBuf + magicPos, g_frameLen - magicPos);
      g_frameLen -= magicPos;
    }

    if (g_frameLen < (int)sizeof(PacketHeader))
      break;

    PacketHeader h{};
    memcpy(&h, g_frameBuf, sizeof(PacketHeader));

    if (h.length > 512) {
      memmove(g_frameBuf, g_frameBuf + 4, g_frameLen - 4);
      g_frameLen -= 4;
      continue;
    }

    const int totalSize =
        (int)(sizeof(PacketHeader) + h.length + sizeof(uint32_t));
    if (g_frameLen < totalSize)
      break;

    if (validatePacket(g_frameBuf, totalSize, false)) {
      PacketHeader header = extractHeader(g_frameBuf);
      if (header.payload_type ==
          static_cast<uint8_t>(SimPayloadType::PHYSICS)) {
        onPhysicsPacket(extractPayload<PhysicsStatePayload>(g_frameBuf));
      }
    }

    memmove(g_frameBuf, g_frameBuf + totalSize, g_frameLen - totalSize);
    g_frameLen -= totalSize;
  }
}

// ── Arduino entry points ──────────────────────────────────────────────────────

void setup() {
  Serial.begin(BAUD_RATE);
  Serial.printf("[esp] NET_DECEL = %.2f m/s²\n", NET_DECEL);
  Serial.printf("[esp] TARGET_STOP_ALT = %.1fm  SAFETY_MARGIN = %.1fm\n",
                TARGET_STOP_ALT, SAFETY_MARGIN);
  Serial.println("[esp] Controller ready");
}

void loop() {
  if (Serial.available() > 0) {
    uint8_t buf[256];
    int bytes = Serial.readBytes(buf, sizeof(buf));
    if (bytes > 0)
      feedFramer(buf, bytes);
  }
}