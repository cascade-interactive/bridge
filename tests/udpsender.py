import socket
import time

from build_packet import (build_packet, build_physics_payload, validate_packet, parse_header, parse_actuator_payload, PayloadType, DeviceID, FLAG_NO_CRC)


class Elevator:
    def __init__(self, position: float = 10.0, gravity: float = -9.81, mass: float = 500.0, grounded = False, previous_velocity = 0.0):
        self.position = position
        self.velocity = 0.0
        self.acceleration = 0.0
        self.gravity = gravity
        self.mass = mass
        self.arrested = False
        self.friction_coefficient = 200
        self.brake_force = 0.0
        self.net_force = 0.0
        self.grounded = False

    def step(self, dt: float) -> None:
        gravity_force  = self.mass * self.gravity
        friction_force = -self.friction_coefficient * self.velocity

        if self.velocity < 0:
            braking = +self.brake_force
        elif self.velocity > 0:
            braking = -self.brake_force
        else:
            braking = 0.0

        self.net_force    = gravity_force + friction_force + braking
        self.acceleration = self.net_force / self.mass

        if not self.grounded:
            self.previous_velocity = self.velocity

        self.velocity    += self.acceleration * dt
        self.position    += self.velocity * dt

        if self.position <= 0.0:
            self.grounded = True
            print(f"Impact! Velocity: {self.previous_velocity:.2f} m/s")
            self.position = 0.0
            self.velocity = 0.0

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
    if not elevator.grounded:
        print(f"Position: {elevator.position:.2f} m Velocity: {elevator.velocity:.2f} m/s")
        elevator.step(dt)

    # Receive
    try:
        data, addr = bridgeSocket.recvfrom(1024)
        data, addr = bridgeSocket.recvfrom(1024)

        if validate_packet(data, verbose=True):
            header = parse_header(data)

        if header['payload_type'] == PayloadType.ACTUATOR:
            actuator = parse_actuator_payload(data)
            elevator.brake_force = actuator['command']
            print(f"Received actuator command: {actuator['command']:.2f} N")
                
                               

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
    

