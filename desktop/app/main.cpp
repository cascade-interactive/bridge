#include <chrono>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <string>
#include <unordered_map>

#include "bridge/sockets/SerialPort.hpp"
#include "bridge/sockets/UDPSocket.hpp"
#include "packet.hpp"
#include "payloads.hpp"
#include "payloads/sim_payloads.hpp"
#include "structs/states.hpp"

// ── Configuration ───────────────────────────────────────────────────────────

namespace Config {
constexpr int BRIDGE_INBOUND_PORT = 6769;
constexpr int SIM_INBOUND_PORT    = 5000;
constexpr int VIZ_OUTBOUND_PORT   = 4200;

constexpr const char* SIM_IP = "127.0.0.1";
constexpr const char* VIZ_IP = "127.0.0.1";

constexpr const char* ESP_PORT = "COM3";
constexpr int ESP_BAUD         = 115200;
}  // namespace Config

// ── Utilities ───────────────────────────────────────────────────────────────

static uint64_t nowMicros() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

// ── SimBridge Core ──────────────────────────────────────────────────────────

class SimBridge {
 public:
  SimBridge()
      : m_simListener(Config::BRIDGE_INBOUND_PORT, true),
        m_esp(Config::ESP_PORT, Config::ESP_BAUD) {}

  // ── Handler Registration ──
  // Maps an EnumType to a specific lambda. Automatically extracts the correct payload type.
  template <typename PayloadT, typename EnumT>
  void on(EnumT type,
          std::function<void(PayloadT, const PacketHeader&)> handler) {
    uint8_t typeId = static_cast<uint8_t>(type);

    m_handlers[typeId] = [handler](const uint8_t* raw_data,
                                   const PacketHeader& header) {
      // The bridge handles the nasty byte-casting automatically
      PayloadT payload = extractPayload<PayloadT>(raw_data);
      handler(payload, header);
    };
  }

  WorldState& getWorld() { return m_world; }

  // Outbound Routing

  template <typename PayloadT, typename EnumT>
  void sendToEsp(const PayloadT& payload, EnumT type, uint64_t timestamp_us) {
    auto packet =
        buildPacket(payload, static_cast<PayloadType>(type), DeviceID::BRIDGE,
                    FLAG_NONE, m_seq++, timestamp_us);
    m_esp.send(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
  }

  template <typename PayloadT, typename EnumT>
  void sendToSim(const PayloadT& payload, EnumT type, uint64_t timestamp_us) {
    auto packet =
        buildPacket(payload, static_cast<PayloadType>(type), DeviceID::BRIDGE,
                    FLAG_NO_CRC, m_seq++, timestamp_us);
    m_simSender.send(Config::SIM_IP, Config::SIM_INBOUND_PORT, &packet,
                     sizeof(packet));
  }

  template <typename PayloadT, typename EnumT>
  void sendToViz(const PayloadT& payload, EnumT type, uint64_t timestamp_us) {
    auto packet =
        buildPacket(payload, static_cast<PayloadType>(type), DeviceID::BRIDGE,
                    FLAG_NO_CRC, m_seq++, timestamp_us);
    m_vizSender.send(Config::VIZ_IP, Config::VIZ_OUTBOUND_PORT, &packet,
                     sizeof(packet));
  }

  // ── Main Loop ──
  void run() {
    std::cout << "Bridge running..." << std::endl;
    while (true) {
      pollSim();
      pollEsp();
    }
  }

 private:
  WorldState m_world{};
  UDPSocket m_simListener;
  UDPSocket m_simSender;
  UDPSocket m_vizSender;
  SerialPort m_esp;

  uint32_t m_seq = 0;
  std::string m_serialBuf;

  // The registry: stores type-agnostic wrappers that point to your typed lambdas
  using RawHandler = std::function<void(const uint8_t*, const PacketHeader&)>;
  std::unordered_map<uint8_t, RawHandler> m_handlers;

  void pollSim() {
    std::string data = m_simListener.receive();
    if (!data.empty()) {
      processPacket(reinterpret_cast<const uint8_t*>(data.data()), data.size(),
                    "udp");
    }
  }

  void pollEsp() {
    std::string data = m_esp.receive();
    if (data.empty())
      return;

    m_serialBuf += data;

    while (m_serialBuf.size() >= sizeof(PacketHeader)) {
      PacketHeader h{};
      std::memcpy(&h, m_serialBuf.data(), sizeof(PacketHeader));

      if (h.magic != PACKET_MAGIC) {
        m_serialBuf.erase(0, 1);
        continue;
      }

      if (h.length > 1024) {
        m_serialBuf.erase(0, 4);
        continue;
      }

      const int totalSize = sizeof(PacketHeader) + h.length + sizeof(uint32_t);
      if (m_serialBuf.size() < totalSize)
        break;  // Wait for the rest of the frame

      processPacket(reinterpret_cast<const uint8_t*>(m_serialBuf.data()),
                    totalSize, "serial");
      m_serialBuf.erase(0, totalSize);
    }
  }

  void processPacket(const uint8_t* buf, int size, const char* source) {
    if (!validatePacket(buf, size, false))
      return;

    PacketHeader header = extractHeader(buf);

    // No more dispatch chain! Just look up the handler.
    auto it = m_handlers.find(header.payload_type);
    if (it != m_handlers.end()) {
      it->second(buf, header);  // Calls the lambda defined in main()
    } else {
      printf("[%s] Unhandled payload type: 0x%02X\n", source,
             header.payload_type);
    }
  }
};

// ── App Entry Point ─────────────────────────────────────────────────────────

int main() {
  SimBridge bridge;

  // 1. Sim -> ESP (Physics)
  bridge.on<
      PhysicsStatePayload>(SimPayloadType::PHYSICS, [&](PhysicsStatePayload
                                                            payload,
                                                        PacketHeader header) {
    // ---- MANIPULATION ZONE ----
    // You have a local copy of `payload`. Modify it however you want before it goes to the ESP.
    // e.g., payload.pos_z += generateGaussianNoise();

    printf("[sim] pos_z=%.2f  vel_z=%.2f\n", payload.pos_z, payload.vel_z);

    // Forwarding
    bridge.sendToEsp(payload, SimPayloadType::PHYSICS, header.timestamp_us);
    bridge.sendToViz(payload, SimPayloadType::PHYSICS, header.timestamp_us);
  });

  // 2. ESP -> Sim (Driver)
  bridge.on<DriverPayload>(PayloadType::DRIVER, [&](DriverPayload payload,
                                                    PacketHeader header) {
    // ---- MANIPULATION ZONE ----
    // Simulate mechanical failure, drop commands, cap limits, etc.
    // e.g., if (simulate_deadband) payload.command = 0.0f;

    printf("[esp] act_id=%u  cmd=%.2f\n", payload.driver_id, payload.command);

    // Forwarding
    bridge.sendToSim(payload, PayloadType::DRIVER, nowMicros());
    bridge.sendToViz(payload, PayloadType::DRIVER, header.timestamp_us);
  });

  bridge.run();

  return 0;
}