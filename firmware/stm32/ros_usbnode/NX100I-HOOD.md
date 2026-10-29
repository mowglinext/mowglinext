# YardForce NX100i hood protocols

Reverse-engineered offline from the stock mainboard firmware ("GRM V4.03_2021") and live RAM
dumps of a running NX100i (2026-10-14). What Mowgli implements: the ultrasonic board (`src/ultrasonic_nx100i.c`, selected by
`ULTRASONIC_TYPE_NX100I_I2C` in `board.h`) and the panel (`src/panel_nx100i.c`,
`PANEL_TYPE_YARDFORCE_NX100I`: handshake, keepalive, START/HOME as UI events 4/5, START as a
virtual play button for the emergency release, `PANEL_NX100I_ShowMessageCode()` for the LCD).
Both await their first run on hardware.

Confirmed from code + live RAM unless marked (inferred).

## Ultrasonic board "RM CSB" — bit-banged I2C on PC10 (SCL) / PC11 (SDA)
- Slave address 7-bit 0x51 (stock writes 0xA2 / 0xA3). Stock bit-bangs push-pull with pull-ups,
  ~15-18 kHz (busy-loop delays, 20 units ≈ 27 µs). Use real open-drain in a replacement.
- Transaction: START, W 0xA2, W reg, repeated START, W 0xA3, read N bytes (ACK all but last), STOP.
- Bring-up: after 100 ms read reg 0x22, 26 bytes = ASCII ID, must start "RM CSB_V1"
  (live: "RM CSB_V1.00_2018(210819)A"). Then every 50 ms read reg 0x20, 4 bytes =
  d0_hi d0_lo d1_hi d1_lo, big-endian u16, units 0.1 mm (live: 0x377B/0x385F = 142.0/144.3 cm).
  Which is left/right: NOT verified (assume d0 = left like LUV1000Ri).
- Timeout: no valid read for 2000 ms -> distances 0, restart at ID read.
- Stock decision logic: enabled by config; threshold T = setting+10 (= 40 cm); per sensor ignore
  < 15 cm and > T; "near" after 2 consistent samples; obstacle flag if either near; "blocked"
  if both in 15..140 cm for 5 samples; cleared when both > 170 cm for 30 counts.
- Mainboard also forwards both distances (cm*10 as u16 BE) to the USB app (id 0x60BE).

## Panel "RM EC3" (LCD + 18 keys) — USART1 PA9 TX / PA10 RX, 115200 8N1, no DMA
- Frame: 55 AA LEN ID_HI ID_LO PAYLOAD[LEN-2] SUM, SUM = byte-sum of all preceding bytes & 0xFF.
- Handshake (mainboard -> panel, each repeated every 20 ms until answered):
  1. `55 AA 02 FF FF FF`  -> panel echoes id FFFF
  2. `55 AA 02 FF FE FE`  -> echoes FFFE
  3. `55 AA 05 FF FD 0A 99 21 ..` (payload = board cfg; NX100i `0A 99 21`) -> FFFD reply accepted iff payload[0]==0x0A
  4. `55 AA 02 FF FB FB`  -> FFFB reply with 26-byte ID starting "RM" (live "RM EC3_V1.00_2020(221121)B") -> connected
- Keepalive every 200 ms: `55 AA 05 50 84 <mower_state> <0x20000386> <0x20000382> SUM`
  (live payload 00 FF 01, sum D8); panel answers `55 AA 03 50 84 04 DA`. 2.5 s silence -> re-handshake.
- Panel -> mainboard requests (id 0x50xx, reply echoes id): 5001 -> 21-byte version string;
  5002 -> 10-byte serial; 5003 + 4 B clock -> reply `50 04 00`; **5005 = START**, **5006 = HOME**
  (echoed with len 0); 5030 -> `50 81 <code>` status/message; 5018 -> 5 B settings; ~110 settings
  get/set sub-ids (0x07-0x3d, 0x54-0x79, 0xa0-0xcd) not decoded.
- LCD: message CODE only (`55 AA 03 50 81 <code> SUM`), chosen from mower state/flags;
  codes seen 0,1,4,5,6,7,8,9,11,12,13 (13 = ultrasonic/trapped), 0x34, plus error numbers.
  Code -> text table must be read off the LCD on the bench. Whether unsolicited 5081 is accepted: unverified.
- Keys: the panel runs its own menu UI; the mainboard only sees semantic commands (START/HOME/settings).
- Other ids on the same UART: 0x00xx / 0x40xx from a second logical device (version "V123.0.12",
  keepalive `55 AA 09 00 84 ...`) — unknown, not needed.
- Note: USART1 is initialised unconditionally at boot (115200). The "BRR=0" seen on 2026-10-13 was
  read with the hood disconnected and is contradicted by code/RAM — re-check.

## Not on the hood
- PE10/PE12/PE13 are plain rising-edge counters (EXTI), no timer capture: not ultrasonic echo.
  PE13 goes high on the dock. Purpose unknown (perimeter coils / dock signal?).
- No cover-open switch found; "Please Close Cover" exists only for the USB app.

## Stock code addresses for bench cross-checks
 USART1 TX ring 0x20006018, RX ring 0x20006118,
link state 0x200081e8[0] (4 = connected), panel ID 0x200007e4, CSB struct 0x20000618
(+2 state, +0x20/+0x28 doubles cm), obstacle flags 0x20000618[0..1].
