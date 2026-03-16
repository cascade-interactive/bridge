#include <stdio.h>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <string>
#include "bridge/sockets/SerialPort.hpp"
#include "bridge/sockets/UDPSocket.hpp"
#include "packet.hpp"
#include "payloads.hpp"
#include "payloads/sim_payloads.hpp"

// ── Temp Prototypes ──────────────────────────────────────────────────────────

template <typename T>
void forwardToVisualizer(const T& payload, PayloadType type,
                         uint64_t timestamp_us);

// ── Globals ───────────────────────────────────────────────────────────────────

// Port Addresses
static constexpr int BRIDGE_INBOUND_PORT = 6769;
static constexpr int SIM_INBOUND_PORT    = 5000;
static constexpr int VIZ_OUTBOUND_PORT   = 4200;

static SerialPort* g_esp = nullptr;
static UDPSocket* g_sim  = nullptr;
static UDPSocket* g_viz  = nullptr;
static uint32_t g_seq    = 0;

static std::string g_serialBuf;

static uint64_t nowMicros() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

// ── Handlers ──────────────────────────────────────────────────────────────────

void onPhysicsPacket(const PhysicsStatePayload& payload,
                     const PacketHeader& header) {
  printf("[sim] pos=(%.2f, %.2f, %.2f)  vel=(%.2f, %.2f, %.2f)\n",
         payload.pos_x, payload.pos_y, payload.pos_z, payload.vel_x,
         payload.vel_y, payload.vel_z);

  auto packet =
      buildPacket(payload, static_cast<PayloadType>(SimPayloadType::PHYSICS),
                  DeviceID::BRIDGE,
                  FLAG_NONE,  // serial -- compute CRC
                  g_seq++, header.timestamp_us);

  const int wireSize =
      sizeof(PacketHeader) + sizeof(PhysicsStatePayload) + sizeof(uint32_t);
  g_esp->send(reinterpret_cast<const uint8_t*>(&packet), wireSize);

  forwardToVisualizer(payload,
                      static_cast<PayloadType>(SimPayloadType::PHYSICS),
                      header.timestamp_us);
}

void onActuatorPacket(const ActuatorPayload& payload,
                      const PacketHeader& header) {
  printf("[esp] actuator_id=%u  command=%.2f\n", payload.actuator_id,
         payload.command);

  auto packet = buildPacket(payload, PayloadType::ACTUATOR, DeviceID::BRIDGE,
                            FLAG_NO_CRC,  // UDP -- skip CRC
                            g_seq++, nowMicros());

  const int wireSize =
      sizeof(PacketHeader) + sizeof(ActuatorPayload) + sizeof(uint32_t);
  g_sim->send("127.0.0.1", SIM_INBOUND_PORT, &packet, wireSize);

  forwardToVisualizer(payload, PayloadType::ACTUATOR, header.timestamp_us);
}

// ── Dispatch ──────────────────────────────────────────────────────────────────

void dispatchPacket(const uint8_t* buf, int size) {
  PacketHeader header = extractHeader(buf);

  if (header.payload_type == static_cast<uint8_t>(SimPayloadType::PHYSICS)) {
    onPhysicsPacket(extractPayload<PhysicsStatePayload>(buf), header);
  } else if (header.payload_type ==
             static_cast<uint8_t>(PayloadType::ACTUATOR)) {
    onActuatorPacket(extractPayload<ActuatorPayload>(buf), header);
  } else {
    printf("[dispatch] Unknown payload type 0x%02X\n", header.payload_type);
  }
}

void parsePacket(const std::string& data, const std::string& source) {
  const auto* buf = reinterpret_cast<const uint8_t*>(data.data());
  const int size  = static_cast<int>(data.size());

  if (!validatePacket(buf, size)) {
    printf("[%s] Invalid packet (%d bytes)\n", source.c_str(), size);
    return;
  }

  dispatchPacket(buf, size);
}

// ── Serial accumulator ────────────────────────────────────────────────────────

void feedSerial(const std::string& incoming) {
  g_serialBuf += incoming;

  while (true) {
    if (g_serialBuf.size() < sizeof(PacketHeader))
      break;

    // Check magic -- if wrong, drop one byte and retry
    PacketHeader h{};
    std::memcpy(&h, g_serialBuf.data(), sizeof(PacketHeader));

    if (h.magic != PACKET_MAGIC) {
      g_serialBuf.erase(0, 1);
      continue;
    }

    // Sanity check length before waiting to accumulate
    if (h.length > 1024) {
      printf("[serial] Unreasonable length %u, dropping magic\n", h.length);
      g_serialBuf.erase(0, 4);
      continue;
    }

    const int totalSize = sizeof(PacketHeader) + h.length + sizeof(uint32_t);
    if ((int)g_serialBuf.size() < totalSize)
      break;  // wait for more bytes

    parsePacket(g_serialBuf.substr(0, totalSize), "serial");
    g_serialBuf.erase(0, totalSize);
  }
}

// ── Visualizer forwarding ────────────────────────────────────────────────────\

template <typename T>
void forwardToVisualizer(const T& payload, PayloadType type,
                         uint64_t timestamp_us) {
  if (!g_viz)
    return;

  auto packet = buildPacket(payload, type, DeviceID::BRIDGE, FLAG_NO_CRC,
                            g_seq++, timestamp_us);

  const int wireSize = sizeof(PacketHeader) + sizeof(T) + sizeof(uint32_t);

  g_viz->send("127.0.0.1", VIZ_OUTBOUND_PORT, &packet, wireSize);
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main() {
  UDPSocket simListener(BRIDGE_INBOUND_PORT, true);
  UDPSocket simSender;
  UDPSocket vizSender;
  SerialPort esp("COM3", 115200);

  g_esp = &esp;
  g_sim = &simSender;
  g_viz = &vizSender;

  esp.printStatus();
  printf("Bridge running...\n");

  while (true) {
    // UDP ingress (sim → bridge → ESP)
    std::string sim_inbound = simListener.receive();
    if (!sim_inbound.empty())
      parsePacket(sim_inbound, "udp");

    // Serial ingress (ESP → bridge → sim)
    std::string esp_inbound = esp.receive();
    if (!esp_inbound.empty())
      feedSerial(esp_inbound);
  }

  return 0;
}