#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>
#include "bridge/sockets/SerialPort.hpp"
#include "bridge/sockets/UDPSocket.hpp"
#include "packet.hpp"
#include "payloads.hpp"
#include "payloads/sim_payloads.hpp"

std::string parsePacket(const std::string& data) {
  if (data.size() < sizeof(PacketHeader)) {
    std::cout << "Received data too small to be a valid packet." << std::endl;
    return "Invalid packet: Too small";
  }

  if (!validatePacket(reinterpret_cast<const uint8_t*>(data.data()),
                      data.size())) {
    std::cout << "Received invalid packet." << std::endl;
    return "Invalid packet: Invalid format or CRC";
  }

  PacketHeader header =
      extractHeader(reinterpret_cast<const uint8_t*>(data.data()));
  std::cout << "Received valid packet: Version " << header.version
            << ", Length " << header.length << std::endl;

  // We are only checking for sim payloads here for now
  if (header.payload_type == static_cast<uint8_t>(SimPayloadType::PHYSICS)) {
    PhysicsStatePayload sim_payload;
    std::memcpy(&sim_payload, data.data() + sizeof(PacketHeader),
                sizeof(PhysicsStatePayload));
    std::cout << "Sim Payload: Location " << sim_payload.pos_z
              << ", Acceleration " << sim_payload.accel_z << std::endl;
    return "Valid Sim Payload";
  } else {
    std::cout << "Unknown payload type: " << header.payload_type << std::endl;
    return "Invalid packet: Unknown payload type";
  }
}

int main() {

  uint16_t sequence_number = 0;
  UDPSocket SimListener(6769, true);
  UDPSocket SimSender;

  SerialPort* esp = new SerialPort("COM3", 115200);
  esp->printStatus();

  std::cout << "Listening for UDP packets on port 6967..." << std::endl;

  while (true) {
    std::string data = SimListener.receive();
    if (!data.empty()) {
      parsePacket(data);
    }

    ActuatorPayload payload{};
    payload.actuator_id = 0;
    payload.command     = 6000;
    payload.feedback    = 0;

    auto packet =
        buildPacket(payload, PayloadType::ACTUATOR, DeviceID::ESP_0,
                    FLAG_NO_CRC, sequence_number++,
                    static_cast<uint64_t>(
                        std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count()));

    SimSender.send("127.0.0.1", 5000, &packet, sizeof(packet));
  }

  std::this_thread::sleep_for(
      std::chrono::seconds(10));  // Keep the program running for a while

  delete esp;
  return 0;
}