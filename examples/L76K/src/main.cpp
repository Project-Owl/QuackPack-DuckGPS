#include <Arduino.h>
#include <Wire.h>
#include <XPowersLib.h>
#include <DuckGPS.h>
#include <algorithm>

// Standalone Supreme L76K diagnostic example. No radio/SD dependency or demo
// policy. Compare these pins/rails with the exact purchased board revision.
XPowersAXP2101 pmu;
DuckGPS gnss;
DuckGPSFix latest;
bool changed = false;
bool running = false;

void onFix(const DuckGPSFix& fix, void*) {
    // Demonstrates capture, not persistent logging. A real logger must enqueue
    // EVERY snapshot and expose queue overflow rather than keep only latest.
    latest = fix;
    changed = true;
}

void setup() {
    Serial.begin(115200); // Do not wait for a USB host.
    Wire1.begin(42, 41);
    Wire1.setTimeOut(20);
    if (!pmu.begin(Wire1, AXP2101_SLAVE_ADDRESS, 42, 41)) {
        Serial.println("AXP2101 missing; check board revision/power before continuing");
        return;
    }
    // Manufacturer Supreme example: ALDO4 supplies GNSS (classic T-Beam uses ALDO3).
    pmu.setALDO4Voltage(3300);
    pmu.enableALDO4();
    pinMode(7, OUTPUT); // L76K wake/enable.
    digitalWrite(7, HIGH);
    DuckGPS::Options options;
    options.receiver = DuckGPS::Receiver::L76K;
    options.rxPin = 9; options.txPin = 8; // Host UART, per manufacturer example.
    options.baud = 9600;
    options.powerSettleMs = 1500; // elapsed time handled by update(), not delay().
    options.constellations = 0; // Preserve factory GPS+BeiDou; compare mode 7 in field testing.
    gnss.onFix(onFix);
    running = gnss.start(options);
}

void loop() {
    if (!running) return;
    gnss.update(); // Application may service mesh/buttons/storage each pass.
    static char output[160];
    static size_t length = 0, sent = 0;
    if (changed && length == sent) {
        changed = false;
        const int n = snprintf(output, sizeof(output), "fix=%lu valid=%u utc=%s lat=%.6f lon=%.6f hdop=%s%.2f\n",
            static_cast<unsigned long>(latest.sequence), latest.positionValid,
            latest.timeValid ? latest.utc : "unknown", latest.latitude, latest.longitude,
            latest.hasHdop ? "" : "unknown/", latest.hdop);
        length = n > 0 && size_t(n) < sizeof(output) ? size_t(n) : 0;
        sent = 0;
    }
    if (Serial && length > sent) {
        const int available = Serial.availableForWrite();
        const size_t count = available > 0 ? std::min(size_t(available), length-sent) : 0;
        if (count) sent += Serial.write(reinterpret_cast<const uint8_t*>(output+sent), count);
    }
}
