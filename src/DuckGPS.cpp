/**
 * @file DuckGPS.cpp
 * @brief GPS handling class implementation for ClusterDuck Protocol
 * @author Brenton Poke
 * @license Apache 2.0
 */

#include <DuckGPS.h>
#include "utils/DuckLogger.h"
#include <cstdarg>
#include <cstdio>
#include <limits>

namespace {

// UBX framing and the message ids this library uses.
constexpr uint8_t UBX_SYNC_1 = 0xB5;
constexpr uint8_t UBX_SYNC_2 = 0x62;
constexpr uint8_t UBX_CLASS_ACK = 0x05;
constexpr uint8_t UBX_ACK_NAK = 0x00;
constexpr uint8_t UBX_ACK_ACK = 0x01;
constexpr uint8_t UBX_CLASS_CFG = 0x06;
constexpr uint8_t UBX_CFG_MSG = 0x01;
constexpr uint8_t UBX_CFG_RATE = 0x08;
constexpr uint8_t UBX_CFG_NAVX5 = 0x23;
constexpr uint8_t UBX_CFG_ITFM = 0x39;
constexpr uint8_t UBX_CFG_GNSS = 0x3E;

// CFG-MSG in its 3-byte form sets the rate of one NMEA sentence on the port the
// message arrives on (UART1 here): {class, id, rate}.
constexpr uint8_t CFG_MSG_GLL_OFF[] = {0xF0, 0x01, 0x00};
constexpr uint8_t CFG_MSG_GSV_OFF[] = {0xF0, 0x03, 0x00};
constexpr uint8_t CFG_MSG_VTG_OFF[] = {0xF0, 0x05, 0x00};
constexpr uint8_t CFG_MSG_GGA_ON[] = {0xF0, 0x00, 0x01};
constexpr uint8_t CFG_MSG_GSA_ON[] = {0xF0, 0x02, 0x01};
constexpr uint8_t CFG_MSG_RMC_ON[] = {0xF0, 0x04, 0x01};

// CFG-RATE: 1000 ms measurement rate (1 Hz), 1 measurement per solution, GPS time.
constexpr uint8_t CFG_RATE_1HZ[] = {0xE8, 0x03, 0x01, 0x00, 0x01, 0x00};

// CFG-NAVX5 version 0: 3..16 satellites, minimum C/N0 6 dBHz, AssistNow
// Autonomous on.
constexpr uint8_t CFG_NAVX5[] = {
    0x00, 0x00,             // msgVer
    0x1B, 0x00,             // mask1: apply minMax, minCno and aop settings
    0x00, 0x00, 0x00, 0x00, // mask2
    0x00, 0x00,             // reserved
    0x03,                   // minSVs
    0x10,                   // maxSVs
    0x06,                   // minCNO
    0x00,                   // reserved
    0x00,                   // iniFix3D off
    0x00, 0x00,             // reserved
    0x00,                   // ackAiding off
    0x00, 0x00,             // reserved
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, // reserved
    0x00,                   // reserved
    0x01,                   // aopCfg: AssistNow Autonomous on
    0x00, 0x00,             // reserved
    0x00, 0x00,             // reserved
    0x00, 0x00, 0x00, 0x00, // reserved
    0x00, 0x00, 0x00,       // reserved
    0x01,                   // useAdr (not applied, mask2 is 0)
};

// CFG-ITFM: jamming and interference detection on, with u-blox's recommended
// thresholds; antenna type unknown, auxiliary band scan on (u-blox 8 only).
constexpr uint8_t CFG_ITFM[] = {0x3F, 0x10, 0xB1, 0x56, 0x1E, 0x03, 0x00, 0x01};

// CFG-GNSS for u-blox 8: GPS, SBAS, Galileo, QZSS and GLONASS. Older modules
// answer it with NAK, which is harmless.
constexpr uint8_t CFG_GNSS[] = {
    0x00,                                           // msgVer
    0x00,                                           // numTrkChHw (read only)
    0xFF,                                           // numTrkChUse: all available
    0x05,                                           // numConfigBlocks
    // gnssId, resTrkCh, maxTrkCh, reserved1, flags (4 bytes)
    0x00, 0x08, 0x10, 0x00, 0x01, 0x00, 0x01, 0x01, // GPS
    0x01, 0x01, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01, // SBAS
    0x02, 0x04, 0x08, 0x00, 0x01, 0x00, 0x01, 0x01, // Galileo
    0x05, 0x00, 0x03, 0x00, 0x01, 0x00, 0x01, 0x01, // QZSS
    0x06, 0x08, 0x0E, 0x00, 0x01, 0x00, 0x01, 0x01  // GLONASS
};

/**
 * Incremental UBX frame parser. feed() returns true when a frame with a valid
 * checksum is complete; the first bytes of its payload are kept.
 */
class UbxParser {
public:
    uint8_t msgClass = 0;
    uint8_t msgId = 0;
    uint16_t length = 0;
    uint8_t payload[8] = {};

    /// True between frames. Bytes that arrive while idle (except the first sync
    /// byte) are NMEA.
    bool idle() const { return state == State::sync1; }

    bool feed(uint8_t c) {
        switch (state) {
        case State::sync1:
            if (c == UBX_SYNC_1) state = State::sync2;
            return false;
        case State::sync2:
            state = (c == UBX_SYNC_2) ? State::msgClass : State::sync1;
            return false;
        case State::msgClass:
            ckA = 0;
            ckB = 0;
            addChecksum(c);
            msgClass = c;
            state = State::msgId;
            return false;
        case State::msgId:
            addChecksum(c);
            msgId = c;
            state = State::length1;
            return false;
        case State::length1:
            addChecksum(c);
            length = c;
            state = State::length2;
            return false;
        case State::length2:
            addChecksum(c);
            length |= static_cast<uint16_t>(c) << 8;
            count = 0;
            state = (length == 0) ? State::checksumA : State::payload;
            return false;
        case State::payload:
            addChecksum(c);
            if (count < sizeof(payload)) payload[count] = c;
            if (++count == length) state = State::checksumA;
            return false;
        case State::checksumA:
            state = (c == ckA) ? State::checksumB : State::sync1;
            return false;
        case State::checksumB:
            state = State::sync1;
            return c == ckB;
        }
        return false;
    }

private:
    enum class State { sync1, sync2, msgClass, msgId, length1, length2, payload, checksumA, checksumB };
    State state = State::sync1;
    uint16_t count = 0;
    uint8_t ckA = 0;
    uint8_t ckB = 0;

    void addChecksum(uint8_t c) {
        ckA += c;
        ckB += ckA;
    }
};

/// Bounded JSON builder over a caller buffer; any overflow empties the output.
class JsonWriter {
public:
    JsonWriter(char* buffer, size_t bufferSize) : out(buffer), size(bufferSize) { out[0] = '\0'; }

    void add(const char* format, ...) __attribute__((format(printf, 2, 3))) {
        if (!ok) return;
        va_list args;
        va_start(args, format);
        const int n = vsnprintf(out + len, size - len, format, args);
        va_end(args);
        if (n < 0 || static_cast<size_t>(n) >= size - len) {
            ok = false;
            return;
        }
        len += static_cast<size_t>(n);
    }

    size_t finish() {
        if (ok) return len;
        out[0] = '\0';
        return 0;
    }

private:
    char* out;
    size_t size;
    size_t len = 0;
    bool ok = true;
};

/// UTC date and time as reported by the receiver.
struct UtcFields {
    int year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
};

bool isLeapYear(int year) {
    return (year % 4 == 0 && year % 100 != 0) || year % 400 == 0;
}

bool validUtc(int year, uint8_t month, uint8_t day, uint8_t hour, uint8_t minute, uint8_t second) {
    static const uint8_t daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (year < DUCKGPS_MIN_VALID_YEAR || year > DUCKGPS_MAX_VALID_YEAR) return false;
    if (month < 1 || month > 12) return false;
    const uint8_t monthDays = daysInMonth[month - 1] + ((month == 2 && isLeapYear(year)) ? 1 : 0);
    if (day < 1 || day > monthDays) return false;
    // Second 60 is a leap second, which NMEA can report.
    return hour < 24 && minute < 60 && second <= 60;
}

/// Days since 1970-01-01 for a proleptic Gregorian date (Howard Hinnant's
/// days_from_civil).
int64_t daysFromCivil(int year, unsigned month, unsigned day) {
    year -= month <= 2 ? 1 : 0;
    const int era = (year >= 0 ? year : year - 399) / 400;
    const unsigned yearOfEra = static_cast<unsigned>(year - era * 400);
    const unsigned dayOfYear = (153 * (month > 2 ? month - 3 : month + 9) + 2) / 5 + day - 1;
    const unsigned dayOfEra = yearOfEra * 365 + yearOfEra / 4 - yearOfEra / 100 + dayOfYear;
    return static_cast<int64_t>(era) * 146097 + static_cast<int64_t>(dayOfEra) - 719468;
}

bool readUtc(TinyGPSPlus& gps, UtcFields& utc) {
    if (!gps.date.isValid() || !gps.time.isValid()) return false;
    utc.year = gps.date.year();
    utc.month = gps.date.month();
    utc.day = gps.date.day();
    utc.hour = gps.time.hour();
    utc.minute = gps.time.minute();
    utc.second = gps.time.second();
    return validUtc(utc.year, utc.month, utc.day, utc.hour, utc.minute, utc.second);
}

} // namespace

DuckGPS::DuckGPS(HardwareSerial& port) : GPSSerial(port) {}

bool DuckGPS::begin(int rxPin, int txPin, uint32_t baud, bool configureUblox) {
    if (started_) end();
    ublox_ = false;

    // The RX buffer can only be resized while the UART is closed.
    GPSSerial.setRxBufferSize(DUCKGPS_RX_BUFFER_SIZE);
    baud_ = (baud == AUTO_BAUD) ? DUCKGPS_FALLBACK_BAUD : baud;
    GPSSerial.begin(baud_, SERIAL_8N1, rxPin, txPin);
    started_ = true;

    bool heard = true;
    if (baud == AUTO_BAUD) {
        heard = detectBaud();
    } else if (configureUblox) {
        // Fixed baud: one probe decides whether the u-blox configuration applies.
        bool nmeaSeen = false;
        ublox_ = probe(baud_, baud_ <= 9600 ? 1100 : 400, nmeaSeen);
        heard = ublox_ || nmeaSeen;
    } else {
        loginfo_ln("[DuckGPS] UART open at %lu baud (RX %d, TX %d)", static_cast<unsigned long>(baud_), rxPin, txPin);
        return true;
    }

    if (!heard) {
        loginfo_ln("[DuckGPS] No GPS module answered on RX %d / TX %d; listening at %lu baud",
                   rxPin, txPin, static_cast<unsigned long>(baud_));
        return false;
    }
    loginfo_ln("[DuckGPS] %s module at %lu baud", ublox_ ? "u-blox" : "NMEA", static_cast<unsigned long>(baud_));
    if (ublox_ && configureUblox) applyUbloxConfig();
    return true;
}

void DuckGPS::end() {
    if (!started_) return;
    GPSSerial.end();
    started_ = false;
}

bool DuckGPS::detectBaud() {
    // Baud rates in order of likelihood: 9600 is the u-blox and NMEA factory
    // default, 115200 is common on DMS T-Beams. A u-blox at 9600 can delay its
    // reply behind most of a second of NMEA, and a module that is not a u-blox
    // only shows itself with its 1 Hz NMEA burst, so these two get a window of
    // just over a second. Other rates need begin() with an explicit baud.
    struct Candidate {
        uint32_t baud;
        uint32_t windowMs;
    };
    static const Candidate candidates[] = {
        {9600, 1100}, {115200, 1100}, {38400, 400}, {57600, 400}, {19200, 400},
    };

    for (const Candidate& candidate : candidates) {
        bool nmeaSeen = false;
        if (probe(candidate.baud, candidate.windowMs, nmeaSeen)) {
            ublox_ = true;
            baud_ = candidate.baud;
            return true;
        }
        if (nmeaSeen) {
            baud_ = candidate.baud;
            return true;
        }
    }

    baud_ = DUCKGPS_FALLBACK_BAUD;
    GPSSerial.updateBaudRate(baud_);
    clearBuffer();
    return false;
}

bool DuckGPS::probe(uint32_t baud, uint32_t windowMs, bool& nmeaSeen) {
    logdbg_ln("[DuckGPS] Probing at %lu baud", static_cast<unsigned long>(baud));
    GPSSerial.updateBaudRate(baud);
    clearBuffer();

    // Poll CFG-RATE (empty payload). A u-blox answers with CFG-RATE and ACK-ACK.
    writeUbx(UBX_CLASS_CFG, UBX_CFG_RATE, nullptr, 0);

    UbxParser ubx;
    TinyGPSPlus nmea; // separate parser, so bytes read at a wrong baud never reach gps
    const uint32_t start = millis();
    while (uint32_t(millis() - start) < windowMs) {
        while (GPSSerial.available() > 0) {
            const int c = GPSSerial.read();
            if (c < 0) break;
            const uint8_t value = static_cast<uint8_t>(c);
            const bool wasIdle = ubx.idle();
            if (ubx.feed(value)) {
                const bool rateReply = ubx.msgClass == UBX_CLASS_CFG && ubx.msgId == UBX_CFG_RATE;
                const bool rateAck = ubx.msgClass == UBX_CLASS_ACK && ubx.length >= 2 &&
                                     ubx.payload[0] == UBX_CLASS_CFG && ubx.payload[1] == UBX_CFG_RATE;
                if (rateReply || rateAck) return true;
            } else if (wasIdle && value != UBX_SYNC_1) {
                nmea.encode(static_cast<char>(value));
            }
        }
        delay(1);
    }
    nmeaSeen = nmea.passedChecksum() > 0;
    return false;
}

void DuckGPS::applyUbloxConfig() {
    struct Step {
        uint8_t id;
        const uint8_t* payload;
        uint16_t length;
        const char* what;
    };
    // Turn the unused sentences off first: at 9600 baud a full default NMEA
    // burst takes most of a second, and ACKs queue behind it.
    static const Step steps[] = {
        {UBX_CFG_MSG, CFG_MSG_GLL_OFF, sizeof(CFG_MSG_GLL_OFF), "disable GLL"},
        {UBX_CFG_MSG, CFG_MSG_GSV_OFF, sizeof(CFG_MSG_GSV_OFF), "disable GSV"},
        {UBX_CFG_MSG, CFG_MSG_VTG_OFF, sizeof(CFG_MSG_VTG_OFF), "disable VTG"},
        {UBX_CFG_MSG, CFG_MSG_GGA_ON, sizeof(CFG_MSG_GGA_ON), "enable GGA"},
        {UBX_CFG_MSG, CFG_MSG_GSA_ON, sizeof(CFG_MSG_GSA_ON), "enable GSA"},
        {UBX_CFG_MSG, CFG_MSG_RMC_ON, sizeof(CFG_MSG_RMC_ON), "enable RMC"},
        {UBX_CFG_RATE, CFG_RATE_1HZ, sizeof(CFG_RATE_1HZ), "set the update rate"},
        {UBX_CFG_NAVX5, CFG_NAVX5, sizeof(CFG_NAVX5), "configure navigation settings"},
        {UBX_CFG_ITFM, CFG_ITFM, sizeof(CFG_ITFM), "enable jamming detection"},
        {UBX_CFG_GNSS, CFG_GNSS, sizeof(CFG_GNSS), "configure GNSS constellations"},
    };
    const uint16_t ackTimeoutMs = baud_ <= 9600 ? 1000 : 300;

    uint8_t timeouts = 0;
    for (const Step& step : steps) {
        const UbxReply reply = sendUbxWithAck(UBX_CLASS_CFG, step.id, step.payload, step.length, ackTimeoutMs);
        if (reply == UbxReply::ack) {
            timeouts = 0;
            continue;
        }
        if (reply == UbxReply::nak) {
            timeouts = 0;
            logdbg_ln("[DuckGPS] Module rejected: %s", step.what);
            continue;
        }
        logdbg_ln("[DuckGPS] No ACK to: %s", step.what);
        if (++timeouts >= 2) {
            loginfo_ln("[DuckGPS] Module stopped acknowledging; skipping the rest of the configuration");
            return;
        }
    }
    logdbg_ln("[DuckGPS] u-blox configuration done");
}

bool DuckGPS::sendUbx(uint8_t msgClass, uint8_t msgId, const uint8_t* payload, uint16_t length,
                      uint16_t ackTimeoutMs) {
    if (!started_) return false;
    if (ackTimeoutMs == 0) {
        writeUbx(msgClass, msgId, payload, length);
        return true;
    }
    return sendUbxWithAck(msgClass, msgId, payload, length, ackTimeoutMs) == UbxReply::ack;
}

DuckGPS::UbxReply DuckGPS::sendUbxWithAck(uint8_t msgClass, uint8_t msgId, const uint8_t* payload,
                                          uint16_t length, uint16_t timeoutMs) {
    writeUbx(msgClass, msgId, payload, length);

    UbxParser ubx;
    const uint32_t start = millis();
    while (uint32_t(millis() - start) < timeoutMs) {
        while (GPSSerial.available() > 0) {
            const int c = GPSSerial.read();
            if (c < 0) break;
            const uint8_t value = static_cast<uint8_t>(c);
            const bool wasIdle = ubx.idle();
            if (ubx.feed(value)) {
                if (ubx.msgClass == UBX_CLASS_ACK && ubx.length >= 2 &&
                    ubx.payload[0] == msgClass && ubx.payload[1] == msgId) {
                    if (ubx.msgId == UBX_ACK_ACK) return UbxReply::ack;
                    if (ubx.msgId == UBX_ACK_NAK) return UbxReply::nak;
                }
            } else if (wasIdle && value != UBX_SYNC_1) {
                // NMEA keeps flowing while we wait; parse it rather than drop it.
                gps.encode(static_cast<char>(value));
            }
        }
        delay(1);
    }
    return UbxReply::timeout;
}

void DuckGPS::writeUbx(uint8_t msgClass, uint8_t msgId, const uint8_t* payload, uint16_t length) {
    const uint8_t header[6] = {
        UBX_SYNC_1, UBX_SYNC_2, msgClass, msgId,
        static_cast<uint8_t>(length & 0xFF), static_cast<uint8_t>(length >> 8),
    };
    // 8-bit Fletcher checksum over class, id, length and payload.
    uint8_t ckA = 0;
    uint8_t ckB = 0;
    for (size_t i = 2; i < sizeof(header); i++) {
        ckA += header[i];
        ckB += ckA;
    }
    for (uint16_t i = 0; i < length; i++) {
        ckA += payload[i];
        ckB += ckA;
    }
    const uint8_t checksum[2] = {ckA, ckB};

    GPSSerial.write(header, sizeof(header));
    if (length > 0) GPSSerial.write(payload, length);
    GPSSerial.write(checksum, sizeof(checksum));
}

void DuckGPS::clearBuffer() {
    while (GPSSerial.available() > 0)
        GPSSerial.read();
}

bool DuckGPS::update() {
    if (!started_) return false;
    const uint32_t fixesBefore = gps.sentencesWithFix();
    // Only bytes that are actually waiting: read() returns -1 on an empty
    // buffer, and that must never reach the NMEA parser.
    while (GPSSerial.available() > 0) {
        const int c = GPSSerial.read();
        if (c < 0) break;
        gps.encode(static_cast<char>(c));
    }
    return gps.sentencesWithFix() != fixesBefore;
}

bool DuckGPS::readData(unsigned long ms) {
    if (!started_) {
        logerr_ln("[DuckGPS] readData() called before begin()");
        return false;
    }
    logdbg_ln("Reading GPS data for %lu ms...", ms);
    const uint32_t fixesBefore = gps.sentencesWithFix();
    const uint32_t start = millis();
    do {
        update();
        delay(1);
    } while (uint32_t(millis() - start) < ms);

    const bool newFix = gps.sentencesWithFix() != fixesBefore;
    if (newFix) {
        logdbg_ln("Found lock");
    } else {
        loginfo_ln("No GPS Lock");
    }
    return newFix;
}

bool DuckGPS::hasFix(uint32_t maxAgeMs) {
    return gps.location.isValid() && gps.location.age() <= maxAgeMs;
}

bool DuckGPS::timeValid(uint32_t maxAgeMs) {
    UtcFields utc;
    return readUtc(gps, utc) && gps.date.age() <= maxAgeMs && gps.time.age() <= maxAgeMs;
}

std::time_t DuckGPS::tmConvert_t(const int& YYYY, const uint8_t& MM, const uint8_t& DD,
                                 const uint8_t& hh, const uint8_t& mm, const uint8_t& ss) {
    if (!validUtc(YYYY, MM, DD, hh, mm, ss)) return 0;
    const int64_t seconds = daysFromCivil(YYYY, MM, DD) * 86400 +
                            static_cast<int64_t>(hh) * 3600 + mm * 60 + ss;
    // A 32-bit time_t ends in January 2038.
    if (seconds > static_cast<int64_t>(std::numeric_limits<std::time_t>::max())) return 0;
    return static_cast<std::time_t>(seconds);
}

std::time_t DuckGPS::epoch() {
    UtcFields utc;
    if (!readUtc(gps, utc)) return 0;
    return tmConvert_t(utc.year, utc.month, utc.day, utc.hour, utc.minute, utc.second);
}

std::string DuckGPS::ISO8601() {
    UtcFields utc;
    if (!readUtc(gps, utc) ||
        tmConvert_t(utc.year, utc.month, utc.day, utc.hour, utc.minute, utc.second) == 0) {
        return std::string();
    }
    char buffer[40]; // 21 bytes for a valid date; sized for any int year
    snprintf(buffer, sizeof(buffer), "%04d-%02u-%02uT%02u:%02u:%02uZ", utc.year,
             static_cast<unsigned>(utc.month), static_cast<unsigned>(utc.day),
             static_cast<unsigned>(utc.hour), static_cast<unsigned>(utc.minute),
             static_cast<unsigned>(utc.second));
    return buffer;
}

double DuckGPS::lat() {
    return gps.location.lat();
}

double DuckGPS::lng() {
    return gps.location.lng();
}

double DuckGPS::altitudeM() {
    return gps.altitude.meters();
}

double DuckGPS::altitudeFt() {
    return gps.altitude.feet();
}

double DuckGPS::speedKmph() {
    return gps.speed.kmph();
}

uint32_t DuckGPS::satellites() {
    return gps.satellites.value();
}

uint32_t DuckGPS::hdop() {
    return gps.hdop.value();
}

std::string DuckGPS::geoJsonPoint() {
    std::string geojsonpoint = R"({"type": "Point", "coordinates": [)";
    return geojsonpoint
            .append(std::to_string(lng()))
            .append(",")
            .append(std::to_string(lat()))
            .append("]}");
}

size_t DuckGPS::toJson(char* out, size_t outSize, int32_t counter, uint32_t maxAgeMs) {
    if (out == nullptr || outSize == 0) return 0;
    JsonWriter json(out, outSize);
    json.add("{");
    if (counter >= 0) json.add("\"c\":%ld,", static_cast<long>(counter));

    if (gps.location.isValid()) {
        json.add("\"lat\":%.5f,\"lng\":%.5f", lat(), lng());
        if (gps.altitude.isValid()) json.add(",\"alt\":%.1f", altitudeM());
        json.add(",\"sats\":%lu", static_cast<unsigned long>(satellites()));
        if (gps.hdop.isValid()) json.add(",\"hdop\":%.1f", hdop() / 100.0);
        if (!hasFix(maxAgeMs)) json.add(",\"stale\":true");
    } else {
        json.add("\"error\":\"No GPS fix\",\"sats\":%lu,\"chars\":%lu",
                 static_cast<unsigned long>(satellites()),
                 static_cast<unsigned long>(gps.charsProcessed()));
    }

    if (timeValid(maxAgeMs)) json.add(",\"t\":\"%s\"", ISO8601().c_str());
    json.add("}");
    return json.finish();
}
