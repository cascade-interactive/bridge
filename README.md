![C++](https://img.shields.io/badge/C%2B%2B-17-00599C?style=flat&logo=cplusplus&logoColor=white) ![clangd](https://img.shields.io/badge/clangd-LSP-5C5C5C?style=flat&logo=llvm&logoColor=white) ![Style](https://img.shields.io/badge/style-Google%20C%2B%2B-4285F4?style=flat) ![PlatformIO](https://img.shields.io/badge/PlatformIO-ESP32-FF7F00?style=flat&logo=platformio&logoColor=white) [![License: MIT](https://img.shields.io/badge/license-MIT-22C55E?style=flat)](LICENSE)

# Real-Time HIL Simulation Bridge

This is a small Hardware-in-the-Loop setup I built to test embedded flight-control code on a real ESP32 while feeding it simulated flight data from a desktop program.

The basic idea is:

```text
simulated vehicle state -> desktop bridge -> UART -> ESP32 -> telemetry back to desktop
```

The simulation side can be Unreal Engine 5, but I also included a Python mock simulator so I can test the firmware and bridge without launching UE5 every time.

Right now this project is mostly about the communication layer: getting data from a simulator, packing it into a binary protocol, sending it to the ESP32, parsing it safely, and recovering when the serial stream gets corrupted.

It is not a full flight controller yet. It is the HIL bridge I would use before testing actual control logic on physical hardware.

---

## Why I Made This

I wanted a way to test embedded control logic without immediately putting it on a real drone or rocket and hoping nothing stupid happens.

Pure simulation is useful, but it skips a lot of the annoying embedded problems:

- UART timing
- packet framing
- bad bytes
- stale data
- parser desync
- limited bandwidth
- firmware-side message handling

So this project puts a real microcontroller in the loop.

The ESP32 receives simulated sensor/state data as binary packets, runs whatever logic I put on it, and sends telemetry back to the host.

The point is not that the simulation is perfect. The point is that the firmware has to deal with a real hardware interface.

---

## Architecture

```text
+-------------------+        UDP        +----------------------+       UART        +----------------+
| UE5 / Python Sim  |  -------------->  | C++ Desktop Bridge   |  ------------->  | ESP32 Firmware |
|                   |                   |                      |                  |                |
| state vectors     |                   | packetizes data      |                  | parses packets |
| mock physics      |                   | writes serial        |                  | runs handlers  |
+-------------------+                   +----------------------+                  +----------------+
                                                   ^                                      |
                                                   |                                      |
                                                   +------------- telemetry --------------+
```

There are three pieces:

### Simulation

The sim produces vehicle-state data and sends it over UDP.

I have a Python test simulator in `/tests` that sends mock data at about 100 Hz. That is mainly there so I can test packet flow quickly without opening Unreal Engine.

### Desktop Bridge

The bridge is a C++17 program.

It listens for UDP packets, converts them into my binary packet format, and writes them to the ESP32 over UART at 115200 baud.

It also reads telemetry coming back from the ESP32.

### ESP32 Firmware

The ESP32 code is a PlatformIO project.

It reads the UART stream, searches for valid packets, checks the CRC, and dispatches the payload based on packet type.

---

## Packet Format

I used a simple binary protocol instead of JSON because I wanted something closer to what I would actually use on a small embedded target.

Each packet looks like this:

```text
| 0xAA | Length | Sequence | Type | Payload | CRC-32 |
|------|--------|----------|------|---------|--------|
| 1 B  | 1 B    | 2 B      | 1 B  | N B     | 4 B    |
```

### Fields

| Field | Description |
|---|---|
| `0xAA` | Sync byte. Used to find the start of a packet. |
| `Length` | Number of bytes in the payload. |
| `Sequence` | Packet counter. Used to notice skipped or stale packets. |
| `Type` | Message type. Used to decide which handler should process the payload. |
| `Payload` | The actual message data. |
| `CRC-32` | Checksum for rejecting corrupted packets. |

The protocol definitions live in `/shared` so the desktop bridge and ESP32 use the same packet layout.

---

## Serial Framing

This was the main thing I cared about in this project.

If you just read fixed-size structs from UART, one dropped byte can ruin the entire stream. After that, every read is shifted and everything looks like garbage.

So the ESP32 parser works as a small framing state machine.

It does not assume the stream is aligned. It keeps scanning until it finds the `0xAA` sync byte, then reads the length, packet header, payload, and CRC.

If the CRC fails or the length does not make sense, it throws away that candidate packet and starts searching again.

That means the firmware can recover from:

- dropped bytes
- random garbage in the serial stream
- bad packet lengths
- CRC failures
- stale packets
- stream desync

The goal is not to make UART magical. The goal is to fail cleanly and resync instead of silently feeding garbage into the controller.

---

## Sequence Numbers

Every packet has a sequence counter.

This lets the ESP32 tell when packets were skipped or arrived out of order. That matters because the sim can generate data faster than the UART link can move it.

For now, the sequence number is mostly used for diagnostics and stale-packet detection. Later it can be used to decide whether the controller should ignore old state updates.

---

## Repository Layout

```text
/desktop
  C++ UDP-to-UART bridge.

/esp/controller
  PlatformIO firmware for the ESP32.

/shared
  Shared packet definitions, packed structs, message types, and CRC logic.

/tests
  Python mock simulator for generating UDP test data.
```

---

## Build and Run

### 1. Build the Desktop Bridge

Requires CMake and a C++17 compiler.

```bash
cd desktop
mkdir build
cd build
cmake ..
make
./hil_bridge
```

### 2. Flash the ESP32

Requires PlatformIO.

```bash
cd esp/controller
pio run --target upload
```

### 3. Start the Mock Simulator

```bash
cd tests
python sim_mock.py
```

---

## What Works Right Now

Current focus:

- UDP input from the simulator
- C++ desktop bridge
- UART output to the ESP32
- shared binary packet definitions
- CRC-32 validation
- packet sequence counters
- ESP32 stream parser
- basic fault recovery
- Python mock simulator for testing without UE5

---

## What I Still Want To Add

Things I want to improve next:

- cleaner telemetry logging on the desktop side
- better timing measurements
- packet drop statistics
- a real UE5 vehicle state source instead of only the Python mock
- more realistic control-loop examples on the ESP32
- plotting tools for comparing sim state and firmware output
- higher baud rate testing
- maybe a small replay system for debugging bad runs

---

## Known Issues / TODO

This is the stuff I still need to clean up. Some of it is basic repo polish, and some of it is actual protocol/runtime hardening.

- Move hard-coded IPs, ports, COM port, and baud rate into a config file or CLI flags
- Finish the outbound queue/ring buffer path and define what happens when it overflows
- Add reconnect/error handling for serial disconnects and failed writes
- Add latency, packet rate, dropped packet, and CRC failure counters
- Add integration tests using UDP loopback and a mock or virtual serial device
- Document the packet format, byte order, payload layout, and versioning rules more formally
- Make payload registration easier for other robotics use cases, like UGV sensors or actuator commands
- Add basic build/run instructions for Windows and Linux
- Clean up cross-platform serial and socket behavior
- Add a simple architecture diagram showing simulator, bridge, ESP32, and visualization flow

### Protocol Hardening

The binary protocol works for the current bridge, but it needs more explicit rules before I keep building on top of it.

Things I want to lock down:

- packet byte order
- packing/alignment expectations
- max payload sizes
- protocol versioning
- compatibility rules between PC-side tools and firmware
- how new payload types should be added without breaking old firmware

I also want tests that build packets in Python and validate them in C++, and the reverse. That should catch schema drift early instead of finding out later that the desktop bridge and ESP32 disagree about the same packet.

### Runtime Reliability

The current runtime path is good enough for testing the bridge, but I want better visibility into what is happening under load.

Things to add:

- malformed packet counters
- serial resync counters
- dropped message counters
- bridge-to-ESP32 latency measurements
- packet rate logging
- CRC failure logging

If packet rates go higher, I may also replace the current tight polling approach with a clearer event loop or threaded read/write model. I do not want the bridge logic to turn into a pile of timing bugs once the simulator gets more realistic.


---

## Notes

This repo is not meant to pretend the ESP32 is flying a real vehicle yet.

The useful part is the bridge: simulated state goes into real firmware, and the firmware has to survive an actual serial stream with real embedded constraints.

That is the piece I wanted before building more serious control logic on top of it.
