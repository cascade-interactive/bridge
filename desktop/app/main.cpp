#include <chrono>
#include <cstdint>
#include <iostream>
#include <thread>
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

std::string buildResponse(const uint16_t sequence_number) {
  PacketHeader header{};
  header.magic        = PACKET_MAGIC;
  header.version      = PACKET_VERSION;
  header.payload_type = static_cast<uint8_t>(PayloadType::ACTUATOR);
  header.device_id    = static_cast<uint8_t>(DeviceID::ESP_0);
  // NO CRC
  header.flags        = FLAG_NO_CRC;
  header.sequence     = sequence_number;
  header.length       = sizeof(ActuatorPayload);
  header.reserved     = 0x0000;
  header.timestamp_us = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::steady_clock::now().time_since_epoch())
                            .count();

  ActuatorPayload payload{};
  payload.actuator_id = 1;  // Example actuator command (e.g., 50% throttle)
  payload.command     = 100.0f;  // Command value (e.g., 0.5 for 50% throttle)
  payload.feedback    = 0.0f;    // Optional feedback field, set to 0 for now

  Packet<ActuatorPayload> packet{};
  packet.header  = header;
  packet.payload = payload;
  packet.crc32   = 0;  // No CRC for this example

  // Serialize packet to string
  std::string serialized_packet(
      sizeof(PacketHeader) + sizeof(ActuatorPayload) + sizeof(uint32_t), '\0');
  std::memcpy(&serialized_packet[0], &packet.header, sizeof(PacketHeader));
  std::memcpy(&serialized_packet[sizeof(PacketHeader)], &packet.payload,
              sizeof(ActuatorPayload));
  std::memcpy(
      &serialized_packet[sizeof(PacketHeader) + sizeof(ActuatorPayload)],
      &packet.crc32, sizeof(uint32_t));
  return serialized_packet;
}

int main() {
  uint16_t sequence_number = 0;
  UDPSocket SimListener(6769, true);
  UDPSocket SimSender;
  std::cout << "Listening for UDP packets on port 6967..." << std::endl;

  while (true) {
    std::string data = SimListener.receive();
    if (!data.empty()) {
      parsePacket(data);
    }
    sequence_number++;
    SimSender.send("127.0.0.1", 5000, buildResponse(sequence_number));
  }

  std::this_thread::sleep_for(
      std::chrono::seconds(10));  // Keep the program running for a while
}