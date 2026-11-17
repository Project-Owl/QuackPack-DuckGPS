#pragma once

#include <cstddef>
#include <cstdint>
#include <ctime>

#ifndef DUCKGPS_MIN_VALID_YEAR
#define DUCKGPS_MIN_VALID_YEAR 2024
#endif
#ifndef DUCKGPS_MAX_VALID_YEAR
#define DUCKGPS_MAX_VALID_YEAR 2079
#endif

// One immutable GNSS solution. Missing fields are explicitly marked, never
// borrowed from a previous epoch. HDOP is dimensionless, not an error in metres.
struct DuckGPSFix {
    uint32_t sequence = 0;
    uint32_t receivedAtMs = 0;
    uint32_t timeOfDayMs = 0;
    std::time_t epoch = 0;
    uint16_t millisecond = 0;
    bool timeValid = false;
    bool positionValid = false;
    bool hasGga = false;
    bool hasRmc = false;
    bool hasAltitude = false;
    bool hasHdop = false;
    bool hasSatellites = false;
    bool hasSpeed = false;
    uint8_t quality = 0;
    uint16_t satellites = 0;
    double latitude = 0;
    double longitude = 0;
    double altitudeM = 0; // GGA mean sea level, not ellipsoid altitude.
    double hdop = 0;
    double speedKmph = 0;
    char utc[25] = {}; // YYYY-MM-DDThh:mm:ss.sssZ, only when timeValid.
};

// Bounded, allocation-free assembler for GGA/RMC (any two-character talker).
// Pair by UTC time-of-day; emit once when both arrive, on the next epoch, or
// after joinWindowMs without a partner. A GGA-only solution has unknown date.
class DuckGPSFixStream {
public:
    using Callback = void (*)(const DuckGPSFix&, void*);
    void onFix(Callback callback, void* context = nullptr);
    void reset(); // Keeps callback registration.
    bool feed(char byte, uint32_t nowMs);
    bool tick(uint32_t nowMs);
    bool latest(DuckGPSFix& out) const;
    void setJoinWindow(uint32_t ms) { joinWindowMs_ = ms; }
    uint32_t emitted() const { return sequence_; }
    uint32_t rejected() const { return rejected_; }
    uint32_t validSentences() const { return validSentences_; }

private:
    bool sentence(uint32_t nowMs);
    bool emit();
    bool duplicate(uint32_t tod, std::time_t epoch, uint32_t nowMs) const;
    Callback callback_ = nullptr;
    void* context_ = nullptr;
    char line_[160] = {};
    size_t length_ = 0;
    bool collecting_ = false;
    bool pending_ = false;
    bool haveLatest_ = false;
    bool ggaValid_ = false;
    bool rmcValid_ = false;
    uint32_t joinWindowMs_ = 250;
    uint32_t sequence_ = 0;
    uint32_t rejected_ = 0;
    uint32_t validSentences_ = 0;
    DuckGPSFix pendingFix_;
    DuckGPSFix latest_;
    struct Seen { uint32_t tod = 0, at = 0; std::time_t epoch = 0; bool valid = false; };
    Seen seen_[16] = {};
    size_t nextSeen_ = 0;
};
