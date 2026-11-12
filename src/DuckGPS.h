/**
 * @file DuckGPS.h
 * @brief GPS handling class for ClusterDuck Protocol
 *
 * Parses NMEA from a GPS module on a hardware UART with TinyGPS++. begin()
 * opens the UART, detects the baud rate and, on u-blox modules, configures
 * the receiver. update() drains the UART without blocking and belongs in
 * loop(). Position, time and the GPS payload helpers read the parsed state.
 *
 * The library has no dependency on a particular CDP version: the Duck project
 * that includes it pins ClusterDuck Protocol itself.
 *
 * @author Brenton Poke
 * @license Apache 2.0
 */

#ifndef CLUSTERDUCK_PROTOCOL_DUCKGPS_H
#define CLUSTERDUCK_PROTOCOL_DUCKGPS_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <ctime>
#include <string>
#include "include/cdpcfg.h"

#define DUCKGPS_VERSION "1.1.0"

// Default UART pins for begin(): the board's CDPCFG_GPS_RX/TX when CDP defines
// them, otherwise the T-Beam wiring (GPS TX -> GPIO34, GPS RX <- GPIO12).
#ifndef DUCKGPS_DEFAULT_RX
#ifdef CDPCFG_GPS_RX
#define DUCKGPS_DEFAULT_RX CDPCFG_GPS_RX
#else
#define DUCKGPS_DEFAULT_RX 34
#endif
#endif

#ifndef DUCKGPS_DEFAULT_TX
#ifdef CDPCFG_GPS_TX
#define DUCKGPS_DEFAULT_TX CDPCFG_GPS_TX
#else
#define DUCKGPS_DEFAULT_TX 12
#endif
#endif

// Baud rate used when automatic detection hears nothing at all (module not
// powered yet, or not connected). 9600 is the factory default of u-blox and
// most other NMEA modules.
#ifndef DUCKGPS_FALLBACK_BAUD
#define DUCKGPS_FALLBACK_BAUD 9600
#endif

// UART receive buffer. 2048 bytes holds about two seconds of NMEA at 9600 baud,
// so a loop() that stalls briefly does not lose sentences.
#ifndef DUCKGPS_RX_BUFFER_SIZE
#define DUCKGPS_RX_BUFFER_SIZE 2048
#endif

// GPS dates outside this range are treated as invalid. TinyGPS++ maps the
// two-digit NMEA year to 20YY, so the 1980-01-06 date that u-blox modules
// report before they know the time shows up as 2080 and is rejected.
#ifndef DUCKGPS_MIN_VALID_YEAR
#define DUCKGPS_MIN_VALID_YEAR 2024
#endif
#ifndef DUCKGPS_MAX_VALID_YEAR
#define DUCKGPS_MAX_VALID_YEAR 2079
#endif

class DuckGPS {
public:
    /// Pass as the baud argument of begin() to detect the module's baud rate.
    static constexpr uint32_t AUTO_BAUD = 0;
    /// Default freshness limit for hasFix(), timeValid() and toJson().
    static constexpr uint32_t DEFAULT_MAX_AGE_MS = 5000;
    /// Buffer size that always fits the output of toJson(), including the NUL.
    static constexpr size_t JSON_MAX_LEN = 192;

    /**
     * @brief Creates the GPS handler. Does not touch the hardware.
     *
     * @param port UART the GPS module is wired to (Serial1 by default)
     */
    explicit DuckGPS(HardwareSerial& port = Serial1);

    /**
     * @brief Opens the UART and prepares the GPS module. Call once from setup(),
     * after the GPS is powered (on a T-Beam with an AXP2101, enable ALDO3 first).
     *
     * With baud = AUTO_BAUD, 9600, 115200, 38400, 57600 and 19200 baud are
     * probed in turn with a UBX CFG-RATE poll, which a u-blox module answers at
     * its configured rate (the same probe as the DMS getUbloxAck()). A module
     * that only sends NMEA is detected from a sentence with a valid checksum.
     * If nothing answers, the UART stays open at DUCKGPS_FALLBACK_BAUD.
     * Detection takes about 0.2-0.7 s for a u-blox at 9600, about 1.2 s at
     * 115200, and blocks for about 3.4 s when no module answers.
     *
     * When a u-blox module answered and configureUblox is true, the receiver is
     * set to 1 Hz with GGA, GSA and RMC output, GPS + SBAS + Galileo + QZSS +
     * GLONASS, and interference detection. Each message waits for its ACK for
     * up to 1 s at 9600 baud and 0.3 s at higher rates, and configuration stops
     * after two unanswered messages.
     *
     * With an explicit baud and configureUblox = false, begin() only opens the
     * UART and returns at once.
     *
     * @param rxPin ESP32 pin that receives from the GPS (the module's TX)
     * @param txPin ESP32 pin that transmits to the GPS (the module's RX)
     * @param baud UART baud rate, or AUTO_BAUD to detect it
     * @param configureUblox send the u-blox configuration when a u-blox answers
     * @return true if a module was heard, or if detection was skipped;
     *         false if nothing answered (the UART is still open)
     */
    bool begin(int rxPin = DUCKGPS_DEFAULT_RX, int txPin = DUCKGPS_DEFAULT_TX,
               uint32_t baud = AUTO_BAUD, bool configureUblox = true);

    /**
     * @brief Closes the UART, for example before deep sleep. begin() reopens it.
     */
    void end();

    /**
     * @brief Parses every byte waiting in the UART and returns. Never blocks;
     * call it on every loop() pass.
     *
     * @return true if at least one new sentence with a position fix was parsed
     */
    bool update();

    /**
     * @brief Parses GPS data for a fixed window. This blocks for ms
     * milliseconds; prefer update() from loop().
     *
     * @param ms length of the read window in milliseconds
     * @return true if at least one new sentence with a position fix was parsed
     */
    bool readData(unsigned long ms = 5000);

    /**
     * @brief Whether the receiver reported a position fix recently.
     *
     * @param maxAgeMs maximum age of the last fix, in milliseconds
     * @return true if a fix was parsed within the last maxAgeMs milliseconds
     */
    bool hasFix(uint32_t maxAgeMs = DEFAULT_MAX_AGE_MS);

    /**
     * @brief Whether the receiver reported a plausible UTC date and time
     * recently. A receiver can know the time before it has a position fix.
     *
     * @param maxAgeMs maximum age of the last date and time, in milliseconds
     * @return true if epoch() and ISO8601() hold a fresh, valid timestamp
     */
    bool timeValid(uint32_t maxAgeMs = DEFAULT_MAX_AGE_MS);

    /// Latitude of the last fix, in decimal degrees (0 before the first fix).
    double lat();
    /// Longitude of the last fix, in decimal degrees (0 before the first fix).
    double lng();
    /// Altitude above mean sea level of the last fix, in metres.
    double altitudeM();
    /// Altitude above mean sea level of the last fix, in feet.
    double altitudeFt();
    /// Ground speed of the last fix, in km/h.
    double speedKmph();
    /// Number of satellites used in the last fix.
    uint32_t satellites();
    /// Horizontal dilution of precision, in hundredths (120 means 1.2).
    uint32_t hdop();

    /**
     * @brief UTC time reported by the receiver, as Unix epoch seconds. The
     * value is the time of the last parsed sentence; use timeValid() to check
     * that it is recent.
     *
     * @return epoch seconds, or 0 if the date or time is missing or invalid
     */
    std::time_t epoch();

    /**
     * @brief UTC time reported by the receiver in ISO 8601 form.
     *
     * @return "YYYY-MM-DDThh:mm:ssZ", or an empty string when epoch() is 0
     */
    std::string ISO8601();

    /**
     * @brief The last position as a GeoJSON Point.
     *
     * @return {"type": "Point", "coordinates": [lng,lat]}
     */
    std::string geoJsonPoint();

    /**
     * @brief Writes the GPS payload as JSON, ready for topics::gps.
     *
     * With a position: {"c":N,"lat":..,"lng":..,"alt":..,"sats":..,"hdop":..,"t":".."}
     * - lat and lng have 5 decimals; alt is in metres with 1 decimal; hdop
     *   is the real HDOP with 1 decimal.
     * - "stale":true is added when the last fix is older than maxAgeMs.
     * Without a position: {"c":N,"error":"No GPS fix","sats":..,"chars":..,"t":".."}
     * - chars is the number of bytes received from the GPS, for diagnostics.
     * In both forms "c" is left out when counter is negative, and "t" (ISO 8601
     * UTC) only appears when timeValid(maxAgeMs). alt and hdop only appear when
     * the receiver reported them. The longest possible output is about 150
     * bytes, well inside CDP's 228-byte payload limit.
     *
     * @param out destination buffer; JSON_MAX_LEN bytes always suffice
     * @param outSize size of out in bytes
     * @param counter value for the "c" field, or -1 to leave it out
     * @param maxAgeMs freshness limit for the fix and the time
     * @return length of the JSON (without the NUL), or 0 if it did not fit
     */
    size_t toJson(char* out, size_t outSize, int32_t counter = -1,
                  uint32_t maxAgeMs = DEFAULT_MAX_AGE_MS);

    /// UART baud rate in use (0 before begin()).
    uint32_t baud() const { return baud_; }
    /// Whether begin() got a UBX reply, so the module is a u-blox.
    bool isUblox() const { return ublox_; }

    /**
     * @brief Sends a UBX message and optionally waits for its ACK. NMEA that
     * arrives while waiting is still parsed.
     *
     * @param msgClass UBX message class (0x06 for CFG)
     * @param msgId UBX message id
     * @param payload message payload (may be nullptr when length is 0)
     * @param length payload length in bytes
     * @param ackTimeoutMs how long to wait for ACK-ACK; 0 sends without waiting
     * @return true if the module acknowledged the message (or, with
     *         ackTimeoutMs = 0, if it was written); false on NAK or timeout
     */
    bool sendUbx(uint8_t msgClass, uint8_t msgId, const uint8_t* payload,
                 uint16_t length, uint16_t ackTimeoutMs);

    /// The underlying TinyGPS++ parser, for data this class does not wrap.
    TinyGPSPlus& parser() { return gps; }

protected:
    /**
     * @brief Converts a UTC calendar date and time to Unix epoch seconds,
     * without mktime() or the TZ setting.
     *
     * @return epoch seconds, or 0 if any field is out of range
     */
    static std::time_t tmConvert_t(const int& YYYY, const uint8_t& MM, const uint8_t& DD,
                                   const uint8_t& hh, const uint8_t& mm, const uint8_t& ss);

private:
    enum class UbxReply { ack, nak, timeout };

    TinyGPSPlus gps;
    HardwareSerial& GPSSerial;
    uint32_t baud_ = 0;
    bool started_ = false;
    bool ublox_ = false;

    bool detectBaud();
    bool probe(uint32_t baud, uint32_t windowMs, bool& nmeaSeen);
    void applyUbloxConfig();
    void writeUbx(uint8_t msgClass, uint8_t msgId, const uint8_t* payload, uint16_t length);
    UbxReply sendUbxWithAck(uint8_t msgClass, uint8_t msgId, const uint8_t* payload,
                            uint16_t length, uint16_t timeoutMs);
    void clearBuffer();
};

#endif //CLUSTERDUCK_PROTOCOL_DUCKGPS_H
