# BasicDuckFirmware

A MamaDuck (device ID `MAMAGPS1`) that reports its GPS position on the `gps` topic.

- `setup()` switches on the T-Beam v1.2 GPS rail (AXP2101 ALDO3). It then calls `dgps.begin()` before the radio starts; `begin()` detects the module's baud rate and configures a u-blox receiver.
- `loop()` calls `dgps.update()` on every pass. This parses whatever the GPS has sent and never blocks, so the duck keeps relaying LoRa traffic.
- Every 60 s a timer sends `dgps.toJson()` with a running counter, with or without a fix.
- Every 10 s it prints the fix and UTC time on the serial console.

## Build and flash

From the repository root:

```bash
pio run -t upload
pio device monitor
```

The root `platformio.ini` builds this sketch for `ttgo-t-beam` with espressif32 7.0.1 and Project-Owl CDP 5.5 pinned by commit. It uses this checkout of the QuackPack (`symlink://.`).

## Using it in your own project

Copy `src/main.cpp`. In your own `platformio.ini`, pin `QuackPack-DuckGPS` by commit next to your CDP pin. The main [README](../../README.md#installation) explains how.
