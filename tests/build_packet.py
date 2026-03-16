import ctypes
import struct
import zlib

# Identifiers
PACKET_MAGIC = 0x4C594E4E
PACKET_VERSION = 0x01

# Flags
FLAG_NONE      = 0x00
FLAG_ACK_REQ   = 0x01
FLAG_IS_ACK    = 0x02
FLAG_BROADCAST = 0x04
FLAG_NO_CRC    = 0x08

# Enums
class PayloadType:
    HEARTBEAT    = 0x00
    ACK          = 0x01
    LOG          = 0x02
    POSITION     = 0x20
    VELOCITY     = 0x21
    ACCELERATION = 0x22
    ORIENTATION  = 0x23
    ANGULAR_RATE = 0x24
    ACTUATOR     = 0x25
    PHYSICS      = 0x40
    BAROMETER    = 0x41

# < = little-endian 
# I = uint32  magic
# B = uint8   version
# B = uint8   payload_type
# B = uint8   device_id
# B = uint8   flags
# I = uint32  sequence
# H = uint16  length
# H = uint16  reserved
# Q = uint64  timestamp_us

class DeviceID:
    SIM    = 0x00
    BRIDGE = 0x01
    ESP_0  = 0x02
    ESP_1  = 0x03
    ESP_2  = 0x04
    ESP_3  = 0x05


class SimPayloadType:
    PHYSICS   = 0x40
    BAROMETER = 0x41

# Header 
HEADER_FORMAT = "<IBBBBIHH Q"
HEADER_SIZE   = struct.calcsize(HEADER_FORMAT)  # should be 24


# Packed project payloads from shared/payloads/sim_payloads.hpp.
PHYSICS_FORMAT = "<dddfffffffffffff"
BAROMETER_FORMAT = "<ff"

PHYSICS_SIZE = struct.calcsize(PHYSICS_FORMAT)
BAROMETER_SIZE = struct.calcsize(BAROMETER_FORMAT)


def build_physics_payload(
    pos_x: float, pos_y: float, pos_z: float,
    vel_x: float, vel_y: float, vel_z: float,
    accel_x: float, accel_y: float, accel_z: float,
    quat_x: float, quat_y: float, quat_z: float, quat_w: float,
    gyro_x: float, gyro_y: float, gyro_z: float,
) -> bytes:
    return struct.pack(
        PHYSICS_FORMAT,
        pos_x, pos_y, pos_z,
        vel_x, vel_y, vel_z,
        accel_x, accel_y, accel_z,
        quat_x, quat_y, quat_z, quat_w,
        gyro_x, gyro_y, gyro_z,
    )


def build_barometer_payload(pressure_pa: float, temperature_c: float) -> bytes:
    return struct.pack(BAROMETER_FORMAT, pressure_pa, temperature_c)

def build_packet(payload_bytes: bytes, payload_type: int, device_id: int,
                 flags: int, sequence: int, timestamp_us: int) -> bytes:

    length = len(payload_bytes)

    header = struct.pack(
        HEADER_FORMAT,
        PACKET_MAGIC,
        PACKET_VERSION,
        payload_type,
        device_id,
        flags,
        sequence,
        length,
        0,            # reserved
        timestamp_us,
    )

    if flags & FLAG_NO_CRC:
        crc = 0x00000000
    else:
        crc = zlib.crc32(header + payload_bytes) & 0xFFFFFFFF

    return header + payload_bytes + struct.pack("<I", crc)


def parse_header(data: bytes) -> dict:
    magic, version, payload_type, device_id, flags, \
    sequence, length, reserved, timestamp_us = struct.unpack_from(HEADER_FORMAT, data, 0)

    return {
        "magic":        magic,
        "version":      version,
        "payload_type": payload_type,
        "device_id":    device_id,
        "flags":        flags,
        "sequence":     sequence,
        "length":       length,
        "timestamp_us": timestamp_us,
    }


def validate_packet(data: bytes, verbose=True) -> bool:

    if len(data) < HEADER_SIZE:
        if verbose: print(f"[packet] Too small ({len(data)} bytes)")
        return False

    h = parse_header(data)

    if h["magic"] != PACKET_MAGIC:
        if verbose: print(f"[packet] Bad magic 0x{h['magic']:08X}")
        return False

    if h["version"] != PACKET_VERSION:
        if verbose: print(f"[packet] Bad version {h['version']}")
        return False

    expected = HEADER_SIZE + h["length"] + 4  # 4 = sizeof(uint32_t) crc
    if expected != len(data):
        if verbose: print(f"[packet] Length mismatch (expected {expected}, got {len(data)})")
        return False

    if not (h["flags"] & FLAG_NO_CRC):
        computed = zlib.crc32(data[:-4]) & 0xFFFFFFFF
        received = struct.unpack_from("<I", data, len(data) - 4)[0]
        if computed != received:
            if verbose: print(f"[packet] CRC mismatch (computed 0x{computed:08X}, got 0x{received:08X})")
            return False

    return True
