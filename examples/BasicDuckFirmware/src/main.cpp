/**
 * @file main.cpp
 * @brief MamaDuck that reports its GPS position with QuackPack-DuckGPS.
 *
 * loop() drains the GPS UART with dgps.update(), which never blocks, so the
 * duck keeps relaying LoRa traffic while the GPS works on a fix. Every
 * GPS_INTERVAL_MS the GPS payload (DuckGPS::toJson) goes out on topics::gps.
 *
 * On a T-Beam with an AXP2101 PMU (v1.2) the GPS runs from the ALDO3 rail,
 * which has to be switched on before dgps.begin().
 */

#include <Arduino.h>
#include <string>
#include <CDP.h>
#include <arduino-timer.h>
#include <DuckGPS.h>

#if defined(ARDUINO_T_Beam)
#include <Wire.h>
#include <XPowersLib.h>
XPowersAXP2101 pmu;
#endif

MamaDuck duck("MAMAGPS1"); // Device ID, exactly 8 bytes and unique in the mesh
DuckGPS dgps;              // Serial1; pins and baud are chosen in begin()
auto timer = timer_create_default();

const unsigned long GPS_INTERVAL_MS = 60000;
const unsigned long STATUS_INTERVAL_MS = 10000;
int32_t gpsCounter = 0;
bool setupOK = false;

/**
 * @brief Powers the GPS on a T-Beam v1.2: ALDO3 at 3.3 V. Boards without an
 * AXP2101 skip this; their GPS is powered some other way.
 */
void enableGpsRail() {
#if defined(ARDUINO_T_Beam)
    Wire.begin(21, 22);
    if (!pmu.begin(Wire, AXP2101_SLAVE_ADDRESS, 21, 22)) {
        Serial.println("[GPS] No AXP2101 found; assuming the GPS is already powered");
        return;
    }
    pmu.setALDO3Voltage(3300);
    pmu.enableALDO3();
#endif
}

/**
 * @brief Timer callback: sends the GPS payload, with or without a fix.
 */
bool sendGps(void*) {
    char payload[DuckGPS::JSON_MAX_LEN];
    if (dgps.toJson(payload, sizeof(payload), gpsCounter) == 0) {
        return true;
    }
    if (duck.sendData(topics::gps, std::string(payload)) == DUCK_ERR_NONE) {
        Serial.printf("[GPS] Sent %s\n", payload);
        gpsCounter++;
    } else {
        Serial.println("[GPS] Failed to send the GPS packet");
    }
    return true; // keep the timer running
}

/**
 * @brief Timer callback: prints what the GPS knows right now.
 */
bool printStatus(void*) {
    if (dgps.hasFix()) {
        Serial.printf("[GPS] %.5f, %.5f, %.1f m, %lu sats, HDOP %.1f\n", dgps.lat(), dgps.lng(),
                      dgps.altitudeM(), static_cast<unsigned long>(dgps.satellites()),
                      dgps.hdop() / 100.0);
    } else {
        Serial.printf("[GPS] No fix yet (%lu bytes received)\n",
                      static_cast<unsigned long>(dgps.parser().charsProcessed()));
    }
    if (dgps.timeValid()) {
        Serial.printf("[GPS] UTC %s (epoch %ld)\n", dgps.ISO8601().c_str(), static_cast<long>(dgps.epoch()));
    }
    return true;
}

void setup() {
    Serial.begin(115200);

    // GPS first: begin() blocks briefly while it detects the module, and the
    // radio is not receiving yet.
    enableGpsRail();
    if (!dgps.begin()) {
        Serial.println("[GPS] No GPS answered; check the power rail and wiring");
    }

    if (duck.setupWithDefaults() != DUCK_ERR_NONE) {
        Serial.println("[MAMA] Failed to set up the MamaDuck");
        return;
    }

    timer.every(GPS_INTERVAL_MS, sendGps);
    timer.every(STATUS_INTERVAL_MS, printStatus);
    setupOK = true;
    Serial.println("[MAMA] Setup OK");
}

void loop() {
    if (!setupOK) {
        return;
    }
    dgps.update();
    timer.tick();
    duck.run();
}
