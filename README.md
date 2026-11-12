# QuackPack-DuckGPS

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![ClusterDuck Protocol](https://img.shields.io/badge/ClusterDuck-Protocol-yellow.svg)](https://github.com/ClusterDuck-Protocol/ClusterDuck-Protocol)
[![Build example](https://github.com/Project-Owl/QuackPack-DuckGPS/actions/workflows/build.yml/badge.svg)](https://github.com/Project-Owl/QuackPack-DuckGPS/actions/workflows/build.yml)

## Overview

This QuackPack adds GPS position and time to a Duck. It reads NMEA from a GPS module on an ESP32 hardware UART with [TinyGPS++](https://github.com/mikalhart/TinyGPSPlus), without blocking the Duck's `loop()`. It also:

- detects the module's baud rate;
- configures u-blox receivers (NEO-6M, NEO-7M, NEO-M8N and similar);
- builds the GPS payload for the ClusterDuck Protocol `gps` topic.

Logging goes through the ClusterDuck Protocol logger.

## What is a QuackPack?

A **QuackPack** is a modular library that extends Duck firmware with application-specific functionality. QuackPacks are **not standalone projects**: they are included in a Duck firmware project and configured from its sketch.

## Hardware

### Supported boards

- **Primary**: LilyGO T-Beam v1.2 (SX1262, AXP2101 PMU)
- **Also**: T-Beam v1.0/v1.1, and any ESP32 board with a GPS module on a hardware UART

### Wiring

`begin()` uses the board's `CDPCFG_GPS_RX` / `CDPCFG_GPS_TX` from the CDP board headers. Without them, it uses the T-Beam pins.

| Signal                           | T-Beam GPIO | Notes                                 |
|----------------------------------|-------------|---------------------------------------|
| `CDPCFG_GPS_RX` (ESP32 receives) | 34          | wired to the GPS module's TX          |
| `CDPCFG_GPS_TX` (ESP32 sends)    | 12          | wired to the GPS module's RX          |
| GPS power                        | AXP2101 ALDO3 | T-Beam v1.2: must be on, 3.3 V      |

### T-Beam v1.2: power the GPS first (AXP2101 ALDO3)

On the T-Beam v1.2 the GPS module runs from the AXP2101 PMU's ALDO3 rail. The library does not manage power. Switch the rail on **before** `dgps.begin()`, or baud detection will hear nothing:

```cpp
#include <XPowersLib.h>
XPowersAXP2101 pmu;

Wire.begin(21, 22);
if (pmu.begin(Wire, AXP2101_SLAVE_ADDRESS, 21, 22)) {
    pmu.setALDO3Voltage(3300);
    pmu.enableALDO3();
}
```

Cutting ALDO3 before deep sleep powers the GPS down. Call `dgps.end()` before sleep, then turn the rail back on and call `begin()` again after wake.

## Installation

### Requirements

- [PlatformIO](https://platformio.org/)
- A Duck firmware project based on [ClusterDuck Protocol](https://github.com/ClusterDuck-Protocol/ClusterDuck-Protocol) 5.x (the example is built against CDP 5.5)

### Pinning it in a project (DMS, CDP or your own)

Add the QuackPack to your project's `lib_deps`. Pin it by **commit SHA** and name the tag in a comment, the same way the project pins CDP. A tag can be moved; a SHA cannot. Name the dependency (`QuackPack-DuckGPS = ...`) so PlatformIO treats every reference as one package.

```ini
lib_deps =
    ; your project owns the CDP pin
    ClusterDuck-Protocol = https://github.com/Project-Owl/ClusterDuck-Protocol.git#0f47693450c948f6a3d9426a6077456560b9c838
    ; QuackPack-DuckGPS v1.1.0
    QuackPack-DuckGPS = https://github.com/Project-Owl/QuackPack-DuckGPS.git#<v1.1.0 commit SHA>
```

`git rev-list -n 1 v1.1.0` prints the commit of a tag.

The library depends only on `mikalhart/TinyGPSPlus`, which PlatformIO installs automatically. If your project already pins it (DMS uses `mikalhart/TinyGPSPlus@1.0.2`), that version is used.

**It has no ClusterDuck-Protocol dependency on purpose.** A git-URL dependency in a library's `library.json` is re-resolved on every `pio pkg install`, and it replaces the CDP version the project pinned. Version 1.0.0 did this: a project pinning CDP 5.5 ended up building CDP 5.0.1. Each project pins CDP itself.

With PlatformIO's default library finder (`chain`), only sketches that `#include <DuckGPS.h>` compile the library, so other ducks in the same project (for example PapaDucks) do not grow.

## Quick start

```cpp
#include <CDP.h>
#include <arduino-timer.h>
#include <DuckGPS.h>

MamaDuck duck("MAMAGPS1");
DuckGPS dgps;                       // Serial1; nothing happens until begin()
auto timer = timer_create_default();
int32_t gpsCounter = 0;

bool sendGps(void*) {
    char payload[DuckGPS::JSON_MAX_LEN];
    if (dgps.toJson(payload, sizeof(payload), gpsCounter) > 0 &&
        duck.sendData(topics::gps, std::string(payload)) == DUCK_ERR_NONE) {
        gpsCounter++;
    }
    return true;
}

void setup() {
    // power the GPS here (see above), then:
    dgps.begin();                   // board pins, automatic baud, u-blox setup
    duck.setupWithDefaults();
    timer.every(60000, sendGps);
}

void loop() {
    dgps.update();                  // never blocks
    timer.tick();
    duck.run();
}
```

The full sketch is in [examples/BasicDuckFirmware](examples/BasicDuckFirmware/src/main.cpp).

## API

Include `<DuckGPS.h>`; the class is `DuckGPS`. All times are UTC.

| Method | Description |
|--------|-------------|
| `DuckGPS(HardwareSerial& port = Serial1)` | Creates the handler. Does not touch the hardware. |
| `bool begin(int rx = DUCKGPS_DEFAULT_RX, int tx = DUCKGPS_DEFAULT_TX, uint32_t baud = DuckGPS::AUTO_BAUD, bool configureUblox = true)` | Opens the UART and detects and configures the module (see below). Returns false if nothing answered; the UART stays open. |
| `void end()` | Closes the UART (for example before deep sleep). |
| `bool update()` | Parses the bytes waiting in the UART and returns. Call it on every `loop()` pass. True if a new sentence with a fix arrived. |
| `bool readData(unsigned long ms = 5000)` | Blocking: parses for `ms` milliseconds. True if a new fix arrived. Prefer `update()`. |
| `bool hasFix(uint32_t maxAgeMs = 5000)` | A position fix arrived within `maxAgeMs`. |
| `bool timeValid(uint32_t maxAgeMs = 5000)` | A plausible UTC date and time arrived within `maxAgeMs`. A receiver often knows the time before it has a fix. |
| `double lat()`, `double lng()` | Last position, decimal degrees. |
| `double altitudeM()` | Altitude above mean sea level, **metres**. |
| `double altitudeFt()` | Altitude above mean sea level, feet. |
| `double speedKmph()` | Ground speed, km/h. |
| `uint32_t satellites()` | Satellites used in the fix. |
| `uint32_t hdop()` | HDOP in **hundredths** (120 means 1.2). |
| `std::time_t epoch()` | Unix time of the last sentence, or 0 when the date or time is invalid. Computed without `mktime()` or `TZ`. |
| `std::string ISO8601()` | `YYYY-MM-DDThh:mm:ssZ`, or `""` when `epoch()` is 0. |
| `std::string geoJsonPoint()` | `{"type": "Point", "coordinates": [lng,lat]}` |
| `size_t toJson(char* out, size_t size, int32_t counter = -1, uint32_t maxAgeMs = 5000)` | Writes the GPS payload (below). Returns its length, or 0 if it did not fit. |
| `uint32_t baud()`, `bool isUblox()` | What `begin()` found. |
| `bool sendUbx(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len, uint16_t ackTimeoutMs)` | Sends any UBX message; waits for its ACK unless `ackTimeoutMs` is 0. |
| `TinyGPSPlus& parser()` | The underlying TinyGPS++ parser, for anything not wrapped here (course, custom NMEA fields, counters). |

Constants: `DuckGPS::AUTO_BAUD` (0), `DuckGPS::DEFAULT_MAX_AGE_MS` (5000), `DuckGPS::JSON_MAX_LEN` (192, always enough for `toJson()`), `DUCKGPS_VERSION` ("1.1.0").

### Payload (`toJson`)

With a position:

```json
{"c":7,"lat":48.11730,"lng":11.51667,"alt":545.4,"sats":8,"hdop":0.9,"t":"2026-11-04T12:35:19Z"}
```

Without a position:

```json
{"c":7,"error":"No GPS fix","sats":0,"chars":420,"t":"2026-11-04T12:35:19Z"}
```

- `c` is the counter argument; it is left out when the counter is negative.
- `lat` and `lng` have 5 decimals.
- `alt` is in metres, the unit the DMS expects.
- `hdop` is the real HDOP.
- `"stale":true` is added when the last fix is older than `maxAgeMs`.
- `t` only appears when `timeValid(maxAgeMs)`.
- `alt` and `hdop` only appear when the receiver reported them.
- `chars` counts the bytes received from the module. 0 means a wiring or power problem.
- The longest possible payload is about 150 bytes, inside CDP's 228-byte limit.

### What `begin()` does

1. Opens the UART with a 2 KB receive buffer.
2. With `AUTO_BAUD`, probes 9600, 115200, 38400, 57600 and 19200 baud with a UBX CFG-RATE poll. A u-blox answers at its configured rate; this is the same probe as the DMS `getUbloxAck()`. A module that only sends NMEA is recognised from a sentence with a valid checksum. If nothing answers, the UART stays at `DUCKGPS_FALLBACK_BAUD` (9600).
3. When a u-blox answered and `configureUblox` is true, it configures the receiver:
   - 1 Hz updates, with GGA, GSA and RMC output (GLL, GSV and VTG off);
   - GPS + SBAS + Galileo + QZSS + GLONASS;
   - AssistNow Autonomous and interference detection.

   Each message waits up to 1 s for its ACK at 9600 baud, and 0.3 s at higher rates. Configuration stops after two unanswered messages.

Typical blocking time in `setup()`:

- about 0.2-0.7 s for a u-blox at 9600 baud;
- about 1.2 s at 115200;
- about 3.4 s when no module answers.

`begin(rx, tx, 9600, false)` opens the UART and returns at once.

### Compile-time options

Set these with `-D` in `build_flags`.

| Define | Default | Meaning |
|--------|---------|---------|
| `DUCKGPS_DEFAULT_RX` / `DUCKGPS_DEFAULT_TX` | `CDPCFG_GPS_RX` / `CDPCFG_GPS_TX`, else 34 / 12 | Default pins for `begin()` |
| `DUCKGPS_FALLBACK_BAUD` | 9600 | Baud rate when detection hears nothing |
| `DUCKGPS_RX_BUFFER_SIZE` | 2048 | UART receive buffer, bytes |
| `DUCKGPS_MIN_VALID_YEAR` / `DUCKGPS_MAX_VALID_YEAR` | 2024 / 2079 | Dates outside this range count as invalid. u-blox modules report 1980-01-06 before they know the time; TinyGPS++ reads that as 2080. |

## Migrating from 1.0.x (and the `phoenix-2026` tag)

The tag `phoenix-2026` marks the copy that phoenix-zoo vendors (temp-phoenix, a6699b9). From that copy:

- `DuckGPS dgps(34, 12);` plus `dgps.setup();` becomes `DuckGPS dgps;` plus `dgps.begin(34, 12);` (or just `dgps.begin()` on a CDP board). The constructor no longer opens the UART.
- `altitude(DuckGPS::AltitudeUnit::feet)` becomes `altitudeFt()`, or better `altitudeM()`. Note that a payload "alt" in feet becomes metres if you switch.
- `speed(DuckGPS::SpeedUnit::kmph)` becomes `speedKmph()`.
- `localHour()`, `hour()`, `minute()`, `second()` are gone. Use `epoch()` / `ISO8601()`, or `parser().time`.
- `setBaudrate()` and `sendMsgWithAck()` are gone; `sendUbx()` sends any UBX message.
- `readData(ms)` still exists and returns bool. It now parses correctly, but `update()` from `loop()` is the intended use.
- `epoch()` now returns 0 for an invalid date or time, and `ISO8601()` returns `""`.
- Adafruit uBlox and Adafruit BusIO are no longer dependencies.

## Building the example

```bash
pio run            # builds examples/BasicDuckFirmware for the T-Beam
pio run -t upload
pio device monitor
```

`platformio.ini` uses the same platform as the DMS builder (espressif32 7.0.1) and CDP 5.5 pinned by commit. GitHub Actions runs `pio run` on every push and pull request.

## Compatibility

- **ClusterDuck Protocol**: 5.x (built and tested against Project-Owl CDP 5.5, 0f476934)
- **PlatformIO**: 6.1+
- **Platform**: espressif32 7.0.1 (Arduino core 2.0.17)
- **Development environment**: PlatformIO only (Arduino IDE not supported)

## Contributing

Contributions are welcome! Please read [CONTRIBUTING.md](CONTRIBUTING.md) for details on our code of conduct and the process for submitting pull requests.

## Changelog

### 1.1.0

- `begin()` opens the UART (no longer the constructor), detects the baud rate and configures u-blox modules with short ACK waits.
- `update()` for `loop()`, plus `hasFix()` and `timeValid()`.
- Fixed NMEA parsing: main never parsed anything, and temp-phoenix fed -1 bytes to TinyGPS++.
- `epoch()` and `ISO8601()` validate the time and compute UTC without `mktime()`.
- Altitude getters are explicit about units; `toJson()` builds the GPS payload with altitude in metres.
- No CDP dependency in `library.json`, owner-qualified dependencies, and Adafruit uBlox dropped.
- Example and CI build against CDP 5.5.

### 1.0.0

- Initial release

## License

This project is licensed under the Apache 2.0 License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- Built on the [ClusterDuck Protocol](https://github.com/ClusterDuck-Protocol/ClusterDuck-Protocol)
- Created by [OWL Integrations](https://www.owlintegrations.com/)

## Resources

- [OWL Integrations](https://www.owlintegrations.com/)
- [OWL DMS](https://www.owlintegrations.com/) - Device Management System for mesh networking
- [ClusterDuck Protocol](https://github.com/ClusterDuck-Protocol/ClusterDuck-Protocol)
- [Discord Community](https://discord.gg/clusterduck)
- **DMS Support**: support@owlintegrations.com

## Support

For questions and support:
- Join the [ClusterDuck Protocol Discord](https://discord.gg/clusterduck)
- Open an [Issue](https://github.com/Project-Owl/QuackPack-DuckGPS/issues)

## Authors

- **Brenton Poke** - *Initial work* - [@BrentonPoke](https://github.com/brentonpoke)
