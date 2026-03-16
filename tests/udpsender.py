import socket
import time

from build_packet import (build_packet, build_physics_payload, validate_packet, parse_header,
    PayloadType, DeviceID,
    FLAG_NO_CRC)


class Elevator:
    def __init__(self, position: float = 100.0, gravity: float = -9.81, mass: float = 500.0):
        self.position = position
        self.velocity = 0.0
        self.acceleration = 0.0
        self.gravity = gravity
        self.mass = mass
        self.arrested = False
        self.arrest_force = 1000.0

    def step(self, dt: float) -> None:
        # Keep the simple falling model, but store the full state on the object.
        self.acceleration = self.gravity
        self.velocity += self.acceleration * dt
        self.position += self.velocity * dt

# Where do we want to send to?
UDP_IP = "127.0.0.1"
OUTBOUND_PORT = 6769
INBOUND_PORT = 5000

INIT = b"Successfully received initial message."

TIMEOUT = 0.01

# BridgeToSim Inbound Socket (Receiver)
bridgeSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
bridgeSocket.bind((UDP_IP, INBOUND_PORT))
bridgeSocket.settimeout(TIMEOUT)


# SimToBridge Outbound Socket (Sender)
simSocket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
simSocket.sendto(INIT, (UDP_IP, OUTBOUND_PORT))

sequence = 0

elevator = Elevator()
dt = 0.01


while True:

    # Simple falling motion
    elevator.step(dt)

    print(f"Position: {elevator.position:.2f} m")

    # Receive
    try:
        data, addr = bridgeSocket.recvfrom(1024)
        
        if validate_packet(data, verbose=True):
            header = parse_header(data)
            print(f"RX: type=0x{header['payload_type']:02X}  "
                  f"seq={header['sequence']}  "
                  f"device=0x{header['device_id']:02X}  "
                  f"len={header['length']}")
            if header['payload_type'] == PayloadType.ACTUATOR:
                print("Received CONTROL packet")
                               

    except socket.timeout:
        pass

    # Send
    timestamp_us = int(time.monotonic() * 1_000_000)

    physics_payload = build_physics_payload(
        0.0,    # pos_x
        0.0,    # pos_y
        elevator.position,   # pos_z
        0.0,    # vel_x
        0.0,    # vel_y
        elevator.velocity,    # vel_z
        0.0,    # accel_x
        0.0,    # accel_y
        elevator.acceleration,   # accel_z
        0.0,    # quat_x
        0.0,    # quat_y
        0.0,    # quat_z
        1.0,    # quat_w
        0.0,    # gyro_x
        0.0,    # gyro_y
        0.0     # gyro_z
    )

    packet = build_packet(
        physics_payload,
        PayloadType.PHYSICS,
        DeviceID.SIM,
        FLAG_NO_CRC,
        sequence,
        timestamp_us,
    )

    simSocket.sendto(packet, (UDP_IP, OUTBOUND_PORT))
    # print(f"TX: PHYSICS  seq={sequence}  size={len(packet)} bytes")
    sequence += 1

    time.sleep(dt)  # ~100hz
    

