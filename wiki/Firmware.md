# Firmware Migration Guide: From ROS1 rosserial to ROS2 COBS Protocol

This guide explains how to adapt the Mowgli STM32 firmware to communicate with the ROS2 stack using the COBS (Consistent Overhead Byte Stuffing) wire protocol instead of the ROS1 rosserial bridge.

## Overview

**What's Changing:**

| Aspect | ROS1 (rosserial) | ROS2 (COBS) |
|--------|-----------------|-----------|
| Protocol | rosserial-over-serial | Binary COBS framing |
| Serialization | ROS message serialization | Custom C structs + CRC-16 |
| Baud rate | 57600 | 115200 (configurable) |
| Handshake | rosserial negotiation | Simple heartbeat |
| Transport | USB serial (CDC) | USB serial (CDC) |
| Error detection | None | CRC-16 CCITT |
| Real-time loop | Arbitrary | ≥ 100 Hz (sensors + heartbeat) |

**Benefits:**

- **Minimal overhead:** COBS adds <1% overhead vs. ROS serialization's variable overhead
- **Deterministic latency:** Binary format, no variable-length serialization
- **Robust:** CRC-16 catches transmission errors
- **Simple:** No ROS message format dependencies on firmware side
- **Portable:** Works with any serial port, no ROS middleware required

> **Note on this guide:** The code below is a generic, pseudocode-level
> migration walkthrough and does **not** match the symbol names or packet
> layout of the live firmware (which uses COBS handler callbacks such as
> `on_cmd_vel` / `motors_handler`, a `pkt_cmd_vel_t` carrying `vx`/`wz`, and a
> 2-wheel encoder/status path — not a 4-wheel `LlStatus` / `handle_cmd_vel`
> scheme). For the authoritative motor-control behaviour, read the
> [Drive Motor Control](#drive-motor-control--per-wheel-velocity-pi) section
> below, then the actual source in
> `firmware/stm32/ros_usbnode/src/ros/ros_custom/cpp_main.cpp`.

## Drive Motor Control — Per-Wheel Velocity PI

The host sends a single `CMD_VEL` packet (`vx`, `wz`); the **firmware** owns the
diff-drive split and the per-wheel velocity loop. A host-side velocity loop over
the USB round-trip was tried and abandoned — it was fragile across the USB
dead-time — so the loop lives in firmware where local encoder feedback is fast.

**Source:** `firmware/stm32/ros_usbnode/src/ros/ros_custom/cpp_main.cpp`
(`on_cmd_vel`, `motors_handler`, `init_ROS`) and the vendored PX4 PID core in
`firmware/stm32/ros_usbnode/include/pid.hpp`.

- **Inverse kinematics:** `on_cmd_vel` converts (`vx`, `wz`) to per-wheel target
  speeds (`vx ± wz·WHEEL_BASE/2`), clamps them to `±MAX_MPS`, and hands them to
  `motors_handler`.
- **Loop:** `motors_handler` runs at `MOTORS_NBT_TIME_MS` = 20 ms (50 Hz). Per
  wheel it derives the actual speed from the signed cumulative encoder count
  (`left/right_ticks_signed`, 300 ticks/m) over the 20 ms period.
- **Controller:** a vendored PX4 `PID` (BSD-3, header-only `pid.hpp`,
  derivative-on-measurement, NaN/inf guards) runs as a **PI** loop: P =
  `WHEEL_PI_KP` (30), I = `WHEEL_PI_KI` (5000), **D = 0**, integral clamp ±100,
  output clamp ±255 PWM.
- **Feedforward:** the PI trim is added on top of an open-loop feedforward
  `target_mps × PWM_PER_MPS` (300). The integrator bridges the brushed-DC
  static-friction deadband (~PWM 40) on sub-deadband commands, where open-loop
  alone leaves the motor buzzing without moving.
- **Anti-windup (direction-aware conditional integration):** the integrator is
  frozen only in the direction that would worsen an already-railed ±255 output;
  it is always allowed to unwind out of saturation (keyed on the error sign, not
  just the saturation bit).
- **Resets / safety:** the integrator is reset on target sign-flip, stop-to-go,
  and hard-stop (emergency or cmd_vel watchdog); a stopped wheel is forced to PWM
  0 to kill residual hum. Set `USE_WHEEL_PI 0` to fall back to open-loop
  forwarding for bring-up.

Field-validated on the robot: linear speed tracked ~0.85–1.03 of commanded.

## Files Overview

### Firmware Directory Structure

```
firmware/  (from Mowgli board repository)
├── src/
│   ├── main.c                      # MCU initialization, main loop
│   ├── hardware/
│   │   ├── motor.c                 # ESC/motor control
│   │   ├── imu.c                   # IMU (accel, gyro)
│   │   ├── encoder.c               # Wheel encoder feedback
│   │   ├── power.c                 # Battery voltage/current monitoring
│   │   ├── serial.c                # USB CDC initialization & I/O
│   │   └── [other drivers]
│   │
│   └── protocol/
│       ├── cobs.c / cobs.h         # COBS encode/decode (NEW)
│       ├── crc16.c / crc16.h       # CRC-16 checksum (NEW)
│       ├── ll_datatypes.h          # Packet structure definitions (NEW)
│       └── packet_handler.c        # Packet assembly/dispatch (NEW)
│
├── include/
│   └── [public headers]
│
└── Makefile / CMakeLists.txt       # Build configuration
```

## Integration Steps

### Step 1: Copy New Protocol Files

From the ROS2 package, copy these files from `mowgli_hardware/include/mowgli_hardware/`:

```bash
# In firmware directory:
mkdir -p src/protocol

# Copy from ROS2 package:
cp /path/to/mowgli_hardware/include/mowgli_hardware/ll_datatypes.hpp \
   firmware/include/ll_datatypes.h

cp /path/to/mowgli_hardware/src/cobs.cpp \
   firmware/src/protocol/cobs.c

cp /path/to/mowgli_hardware/src/crc16.cpp \
   firmware/src/protocol/crc16.c

cp /path/to/mowgli_hardware/src/packet_handler.cpp \
   firmware/src/protocol/packet_handler.c
```

**Note:** C++ files will need minor conversion to C:

- Remove `std::` namespaces, use plain C types
- Replace `<vector>` with fixed-size arrays or circular buffers
- Keep the algorithm unchanged (COBS and CRC-16 are language-agnostic)

### Step 2: Update Serial Initialization

**Before (rosserial):**
```c
void usb_serial_init(void) {
    // Initialize USB CDC for rosserial
    // rosserial expects handshake, topic subscriptions, etc.
}
```

**After (COBS):**
```c
void usb_serial_init(void) {
    // Simple USB CDC initialization (no special handshake)
    usb_cdc_init(115200);  // 115200 baud

    // No rosserial negotiation needed
}
```

### Step 3: Update Main Loop

The main loop must:
1. Read and decode incoming COBS packets
2. Dispatch packets to handlers
3. Collect sensor data
4. Send periodic status packets
5. Send heartbeat to monitor watchdog on Pi side

**Example main loop (pseudocode):**

```c
#define LOOP_FREQ_HZ        100     // Main loop frequency
#define HEARTBEAT_DIV       25      // Send heartbeat every 25 loops (4 Hz)
#define STATUS_DIV          1       // Send status every 1 loop (100 Hz)
#define IMU_DIV             1       // Send IMU every 1 loop (100 Hz)

volatile uint32_t loop_counter = 0;

void main_loop(void) {
    while (1) {
        // ─────────────────────────────────────────────────────────────
        // 1. Read and process incoming serial data
        // ─────────────────────────────────────────────────────────────
        uint8_t rx_buf[256];
        int rx_len = usb_cdc_read_nb(rx_buf, sizeof(rx_buf));

        for (int i = 0; i < rx_len; i++) {
            packet_handler_feed(rx_buf[i]);
        }

        // packet_handler internally calls dispatch_packet() when
        // a complete, valid packet is received.

        // ─────────────────────────────────────────────────────────────
        // 2. Update sensors & state (every loop iteration, 100 Hz)
        // ─────────────────────────────────────────────────────────────
        encoder_update();           // Poll encoders (count ticks)
        imu_update();               // Poll IMU (accel, gyro)
        battery_update();           // Poll battery voltage/current
        check_emergency_buttons();  // Poll stop/lift sensors

        // ─────────────────────────────────────────────────────────────
        // 3. Send periodic packets to Pi
        // ─────────────────────────────────────────────────────────────

        if (loop_counter % HEARTBEAT_DIV == 0) {
            // Heartbeat: 4 Hz (from Pi: emergency flags, release request)
            // This is incoming only; no outgoing heartbeat needed from firmware.
            // Pi sends heartbeat to assert watchdog on firmware.
        }

        if (loop_counter % STATUS_DIV == 0) {
            // Status packet: encoder ticks, emergency state, power, etc.
            send_status_packet();
        }

        if (loop_counter % IMU_DIV == 0) {
            // IMU packet: accel, gyro (if different from status)
            send_imu_packet();
        }

        // ─────────────────────────────────────────────────────────────
        // 4. Motor control (in real-time, no delay)
        // ─────────────────────────────────────────────────────────────
        motor_apply_velocity_command();  // Use latest cmd_vel from packet

        // ─────────────────────────────────────────────────────────────
        // 5. Wait for next loop iteration (maintain ~100 Hz)
        // ─────────────────────────────────────────────────────────────
        systick_wait_until_next_tick(LOOP_FREQ_HZ);

        loop_counter++;
    }
}
```

### Step 4: Implement Packet Handlers

Each incoming packet type triggers a handler function:

```c
// Packet dispatch (called by packet_handler when complete packet received)
void dispatch_packet(const uint8_t *data, size_t len) {
    if (len < 1) return;

    uint8_t type = data[0];

    switch (type) {
        case PACKET_ID_LL_HEARTBEAT:
            handle_heartbeat(data, len);
            break;

        case PACKET_ID_LL_HIGH_LEVEL_STATE:
            handle_high_level_state(data, len);
            break;

        case PACKET_ID_LL_CMD_VEL:
            handle_cmd_vel(data, len);
            break;

        default:
            // Unknown packet type, ignore
            break;
    }
}

// Handler: Heartbeat (Pi → firmware, asserts watchdog)
void handle_heartbeat(const uint8_t *data, size_t len) {
    if (len < sizeof(LlHeartbeat)) return;

    LlHeartbeat pkt;
    memcpy(&pkt, data, sizeof(LlHeartbeat));

    // Reset firmware watchdog (Pi is alive)
    watchdog_reset();

    // Check emergency control bits
    if (pkt.emergency_requested) {
        set_emergency_stop_active(true);
    }

    if (pkt.emergency_release_requested) {
        set_emergency_stop_active(false);
    }
}

// Handler: High-level mode (Pi → firmware, informational)
void handle_high_level_state(const uint8_t *data, size_t len) {
    if (len < sizeof(LlHighLevelState)) return;

    LlHighLevelState pkt;
    memcpy(&pkt, data, sizeof(LlHighLevelState));

    // Pi tells us the current high-level mode (idle, mowing, docking, recording)
    // Useful for sound notifications, LED feedback, etc.
    uint8_t current_mode = pkt.current_mode;
    uint8_t gps_quality = pkt.gps_quality;

    // Example: Play different sound for different modes
    // sound_play_mode_notification(current_mode);

    // Store for telemetry/logging
    firmware_state.current_mode = current_mode;
    firmware_state.gps_quality = gps_quality;
}

// Handler: Velocity command (Pi → firmware, motor control)
void handle_cmd_vel(const uint8_t *data, size_t len) {
    if (len < sizeof(LlCmdVel)) return;

    LlCmdVel pkt;
    memcpy(&pkt, data, sizeof(LlCmdVel));

    // Store velocity command for motor update (next real-time iteration)
    motor_set_target_velocity(pkt.linear_x, pkt.angular_z);
}
```

### Step 5: Implement Packet Senders

Functions to encode and send status, IMU, and UI event packets:

```c
// Send Status packet (100 Hz)
void send_status_packet(void) {
    LlStatus pkt = {0};
    pkt.type = PACKET_ID_LL_STATUS;

    // Encoder counts (reset each send for delta calculation)
    pkt.wheel_ticks.fl_tick = encoder_get_fl_ticks_and_clear();
    pkt.wheel_ticks.fr_tick = encoder_get_fr_ticks_and_clear();
    pkt.wheel_ticks.rl_tick = encoder_get_rl_ticks_and_clear();
    pkt.wheel_ticks.rr_tick = encoder_get_rr_ticks_and_clear();

    // Status bitmask
    pkt.status_bitmask = 0;
    if (firmware_initialized) {
        pkt.status_bitmask |= STATUS_BIT_INITIALIZED;
    }
    if (raspberry_pi_powered) {
        pkt.status_bitmask |= STATUS_BIT_RASPI_POWER;
    }
    if (battery_charging) {
        pkt.status_bitmask |= STATUS_BIT_CHARGING;
    }
    if (rain_sensor_detected) {
        pkt.status_bitmask |= STATUS_BIT_RAIN;
    }
    if (sound_module_available) {
        pkt.status_bitmask |= STATUS_BIT_SOUND_AVAIL;
    }
    if (sound_module_playing) {
        pkt.status_bitmask |= STATUS_BIT_SOUND_BUSY;
    }
    if (ui_board_available) {
        pkt.status_bitmask |= STATUS_BIT_UI_AVAIL;
    }

    // Emergency bitmask
    pkt.emergency_bitmask = 0;
    if (emergency_stop_latched) {
        pkt.emergency_bitmask |= EMERGENCY_BIT_LATCH;
    }
    if (stop_button_pressed) {
        pkt.emergency_bitmask |= EMERGENCY_BIT_STOP;
    }
    if (lift_detected) {
        pkt.emergency_bitmask |= EMERGENCY_BIT_LIFT;
    }

    // Power monitoring
    pkt.v_charge = battery_get_charge_voltage();      // mV → float V
    pkt.v_system = battery_get_system_voltage();      // mV → float V
    pkt.charging_current = battery_get_current();     // mA → float A

    // Send packet
    send_packet((uint8_t *)&pkt, sizeof(LlStatus));
}

// Send IMU packet (100 Hz, or bundled with Status)
void send_imu_packet(void) {
    LlImu pkt = {0};
    pkt.type = PACKET_ID_LL_IMU;

    // Read accelerometer (m/s²)
    imu_get_acceleration(pkt.acceleration_mss);

    // Read gyroscope (rad/s)
    imu_get_angular_velocity(pkt.gyro_rads);

    send_packet((uint8_t *)&pkt, sizeof(LlImu));
}

// Send UI event packet (on-demand: button press, release)
void send_ui_event_packet(uint8_t button_id, uint32_t press_duration_ms) {
    LlUiEvent pkt = {0};
    pkt.type = PACKET_ID_LL_UI_EVENT;
    pkt.button_id = button_id;
    pkt.press_duration = press_duration_ms;

    send_packet((uint8_t *)&pkt, sizeof(LlUiEvent));
}

// Low-level: encode, add CRC, COBS frame, send
void send_packet(const uint8_t *data, size_t len) {
    // Calculate CRC-16 over all bytes (including type byte)
    uint16_t crc = crc16_ccitt(data, len);

    // Create buffer: data + CRC
    uint8_t packet_with_crc[256];
    memcpy(packet_with_crc, data, len);
    packet_with_crc[len]     = (crc >> 0) & 0xFF;   // CRC low byte
    packet_with_crc[len + 1] = (crc >> 8) & 0xFF;   // CRC high byte

    // COBS encode
    uint8_t encoded[256];
    size_t encoded_len = cobs_encode(packet_with_crc, len + 2, encoded);

    // Frame with delimiters and send
    usb_cdc_write_byte(0x00);                        // Start frame
    usb_cdc_write_buf(encoded, encoded_len);         // Encoded payload
    usb_cdc_write_byte(0x00);                        // End frame
}
```

## Packet Structure Reference

### ll_datatypes.h (Header File Structure)

```c
#ifndef LL_DATATYPES_H
#define LL_DATATYPES_H

#include <stdint.h>

// ─────────────────────────────────────────────────────────────
// Packet type IDs
// ─────────────────────────────────────────────────────────────

#define PACKET_ID_LL_HEARTBEAT          0x00
#define PACKET_ID_LL_HIGH_LEVEL_STATE   0x01
#define PACKET_ID_LL_CMD_VEL            0x02
#define PACKET_ID_LL_STATUS             0x10
#define PACKET_ID_LL_IMU                0x11
#define PACKET_ID_LL_UI_EVENT           0x12

// ─────────────────────────────────────────────────────────────
// Status bitmasks
// ─────────────────────────────────────────────────────────────

#define STATUS_BIT_INITIALIZED      (1u << 0)
#define STATUS_BIT_RASPI_POWER      (1u << 1)
#define STATUS_BIT_CHARGING         (1u << 2)
#define STATUS_BIT_RAIN             (1u << 3)
#define STATUS_BIT_SOUND_AVAIL      (1u << 4)
#define STATUS_BIT_SOUND_BUSY       (1u << 5)
#define STATUS_BIT_UI_AVAIL         (1u << 6)

// ─────────────────────────────────────────────────────────────
// Emergency bitmasks
// ─────────────────────────────────────────────────────────────

#define EMERGENCY_BIT_LATCH         (1u << 0)
#define EMERGENCY_BIT_STOP          (1u << 1)
#define EMERGENCY_BIT_LIFT          (1u << 2)

// ─────────────────────────────────────────────────────────────
// Packet structures (all little-endian)
// ─────────────────────────────────────────────────────────────

// Heartbeat: Pi → Firmware (4 Hz)
typedef struct {
    uint8_t type;                           // PACKET_ID_LL_HEARTBEAT
    uint8_t emergency_requested;            // 0 = normal, 1 = emergency stop
    uint8_t emergency_release_requested;    // 0 = no change, 1 = release latch
    uint16_t crc16;                         // Appended by encoder
} LlHeartbeat;

// High-level state: Pi → Firmware (2 Hz)
typedef struct {
    uint8_t type;                           // PACKET_ID_LL_HIGH_LEVEL_STATE
    uint8_t current_mode;                   // 0=idle, 1=mowing, 2=docking, 3=recording
    uint8_t gps_quality;                    // 0=no fix, 1=RTK float, 2=RTK fixed
    uint16_t crc16;
} LlHighLevelState;

// Velocity command: Pi → Firmware (on-demand, ~10 Hz from Nav2)
typedef struct {
    uint8_t type;                           // PACKET_ID_LL_CMD_VEL
    float linear_x;                         // m/s (positive = forward)
    float angular_z;                        // rad/s (positive = left turn)
    uint16_t crc16;
} LlCmdVel;

// Status: Firmware → Pi (100 Hz)
typedef struct {
    uint8_t type;                           // PACKET_ID_LL_STATUS
    uint32_t fl_tick;                       // Encoder ticks (front-left, unused)
    uint32_t fr_tick;                       // Encoder ticks (front-right, unused)
    uint32_t rl_tick;                       // Encoder ticks (rear-left, active)
    uint32_t rr_tick;                       // Encoder ticks (rear-right, active)
    uint8_t status_bitmask;                 // See STATUS_BIT_*
    uint8_t emergency_bitmask;              // See EMERGENCY_BIT_*
    float v_charge;                         // Charging port voltage (V)
    float v_system;                         // Battery voltage (V)
    float charging_current;                 // Charging current (A)
    uint16_t crc16;
} LlStatus;

// IMU: Firmware → Pi (100 Hz)
typedef struct {
    uint8_t type;                           // PACKET_ID_LL_IMU
    float acceleration_mss[3];              // accel_x, accel_y, accel_z (m/s²)
    float gyro_rads[3];                     // gyro_x, gyro_y, gyro_z (rad/s)
    uint16_t crc16;
} LlImu;

// UI Event: Firmware → Pi (on-demand)
typedef struct {
    uint8_t type;                           // PACKET_ID_LL_UI_EVENT
    uint8_t button_id;                      // Button identifier
    uint32_t press_duration;                // Press duration (ms)
    uint16_t crc16;
} LlUiEvent;

#endif  // LL_DATATYPES_H
```

## Testing & Validation

### Loopback Test

Before integrating with the ROS2 stack, test the protocol locally on the STM32:

```c
// Test: Send packet, read it back
void test_cobs_loopback(void) {
    // Create a test packet
    uint8_t test_data[10] = {PACKET_ID_LL_HEARTBEAT, 0x01, 0x00, ...};

    // Encode
    uint8_t encoded[256];
    size_t encoded_len = cobs_encode(test_data, sizeof(test_data), encoded);

    // Verify: encoded length should be ~(original + some overhead)
    printf("Original: %d, Encoded: %d\n", sizeof(test_data), encoded_len);

    // Decode
    uint8_t decoded[256];
    size_t decoded_len = cobs_decode(encoded, encoded_len, decoded);

    // Verify: decoded matches original
    assert(decoded_len == sizeof(test_data));
    assert(memcmp(decoded, test_data, sizeof(test_data)) == 0);

    printf("COBS loopback test: PASS\n");
}

// Test: CRC calculation
void test_crc16(void) {
    uint8_t data[] = {PACKET_ID_LL_STATUS, 0x01, 0x02, 0x03};
    uint16_t crc = crc16_ccitt(data, sizeof(data));

    // Known CRC value (pre-computed)
    uint16_t expected = 0xABCD;  // Example

    assert(crc == expected);
    printf("CRC-16 test: PASS\n");
}
```

### Hardware Test with ROS2 Stack

Once integrated:

```bash
# 1. Bring up the stack
ros2 launch mowgli_bringup mowgli.launch.py serial_port:=/dev/mowgli

# 2. Monitor incoming packets
ros2 topic echo /hardware_bridge/status

# 3. Send a velocity command
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}"

# 4. Verify motor response (should move forward slowly)

# 5. Debug serial with raw bytes
timeout 5 cat /dev/mowgli | xxd
```

### Common Issues

**Issue 1: Packets not received**

- Check baud rate (115200)
- Verify USB driver installed on Pi
- Test with loopback echo test first

**Issue 2: CRC mismatch**

- Verify CRC polynomial (0xA001 for CCITT reversed)
- Check byte order (little-endian)
- Ensure CRC calculated before COBS encoding

**Issue 3: COBS framing errors**

- Verify start/end delimiters (0x00)
- Check that no 0x00 bytes appear in encoded payload
- Test COBS encode/decode in isolation

**Issue 4: Motors not responding to cmd_vel**

- Check emergency stop status (firmware may be in safety mode)
- Verify heartbeat is being received (watchdog not triggered)
- Check velocity limits in motor control code

## Migration Checklist

- [ ] Copy COBS, CRC-16, and protocol files to firmware
- [ ] Update main loop to read/dispatch incoming packets
- [ ] Implement packet handlers (heartbeat, cmd_vel, high_level_state)
- [ ] Implement packet senders (status, IMU, UI event)
- [ ] Update USB serial initialization (remove rosserial)
- [ ] Test COBS loopback (encode/decode)
- [ ] Test CRC-16 calculation
- [ ] Integrate with motor control system
- [ ] Test with hardware on bench (no Pi connection yet)
- [ ] Burn to STM32 and test with ROS2 stack
- [ ] Monitor /hardware_bridge/status for 100 Hz updates
- [ ] Send cmd_vel from ROS2 and verify motor response
- [ ] Test emergency stop button (firmware side)
- [ ] Test Pi watchdog (disconnect and verify motor stops)
- [ ] Document any custom extensions

## References

- **COBS Algorithm:** https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing
- **CRC-16 CCITT:** https://en.wikipedia.org/wiki/Cyclic_redundancy_check
- **STM32 USB CDC:** STM32CubeMX HAL documentation
- **ROS2 Message Types:** https://index.ros.org/doc/ros2/

---

**Once firmware is updated, proceed to [ARCHITECTURE.md](ARCHITECTURE.md) for full system integration.**
