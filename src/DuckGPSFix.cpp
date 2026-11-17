#include "DuckGPSFix.h"
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>

namespace {
int hex(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}
bool number(const char* text, double& out) {
    if (!text || !*text) return false;
    // NMEA decimal notation only: reject spaces, exponents, NaN and infinity.
    const char* p = text;
    if (*p == '-') ++p;
    bool digit = false, dot = false;
    for (; *p; ++p) {
        if (*p >= '0' && *p <= '9') digit = true;
        else if (*p == '.' && !dot) dot = true;
        else return false;
    }
    if (!digit) return false;
    out = std::strtod(text, nullptr);
    return std::isfinite(out);
}
bool integer(const char* text, unsigned max, unsigned& out) {
    if (!text || !*text) return false;
    out = 0;
    for (const char* p = text; *p; ++p) {
        if (*p < '0' || *p > '9') return false;
        const unsigned d = unsigned(*p - '0');
        if (out > max / 10 || (out == max / 10 && d > max % 10)) return false;
        out = out * 10 + d;
    }
    return true;
}
bool clockMs(const char* text, uint32_t& out) {
    if (std::strlen(text) < 6) return false;
    for (size_t i = 0; i < 6; ++i) if (text[i] < '0' || text[i] > '9') return false;
    const unsigned h = (text[0]-'0')*10 + text[1]-'0';
    const unsigned m = (text[2]-'0')*10 + text[3]-'0';
    const unsigned s = (text[4]-'0')*10 + text[5]-'0';
    if (h > 23 || m > 59 || s > 59) return false; // Leap-second epoch is unknown.
    unsigned frac = 0, count = 0;
    if (text[6]) {
        if (text[6] != '.' || !text[7]) return false;
        for (const char* p = text + 7; *p; ++p) {
            if (*p < '0' || *p > '9') return false;
            if (count < 3) frac = frac * 10 + unsigned(*p - '0');
            ++count;
        }
    }
    while (count < 3) { frac *= 10; ++count; }
    out = ((h * 60 + m) * 60 + s) * 1000 + frac;
    return true;
}
bool coordinate(const char* text, const char* dir, bool latitude, double& out) {
    double raw;
    const char* dot = std::strchr(text, '.');
    const size_t digits = dot ? size_t(dot - text) : std::strlen(text);
    if (digits != (latitude ? 4u : 5u)) return false;
    if (!number(text, raw) || raw < 0 || std::strlen(dir) != 1) return false;
    const char positive = latitude ? 'N' : 'E', negative = latitude ? 'S' : 'W';
    if (*dir != positive && *dir != negative) return false;
    const double degrees = std::floor(raw / 100), minutes = raw - degrees * 100;
    out = degrees + minutes / 60;
    if (minutes >= 60 || out > (latitude ? 90 : 180)) return false;
    if (*dir == negative) out = -out;
    return true;
}
bool dateEpoch(const char* date, uint32_t tod, DuckGPSFix& fix) {
    if (std::strlen(date) != 6) return false;
    for (size_t i = 0; i < 6; ++i) if (date[i] < '0' || date[i] > '9') return false;
    const unsigned day = (date[0]-'0')*10 + date[1]-'0';
    const unsigned month = (date[2]-'0')*10 + date[3]-'0';
    int year = 2000 + (date[4]-'0')*10 + date[5]-'0';
    if (year < DUCKGPS_MIN_VALID_YEAR || year > DUCKGPS_MAX_VALID_YEAR || month < 1 || month > 12) return false;
    const bool leap = year % 4 == 0 && (year % 100 != 0 || year % 400 == 0);
    const unsigned days[] = {31,28,31,30,31,30,31,31,30,31,30,31};
    if (day < 1 || day > days[month-1] + unsigned(month == 2 && leap)) return false;
    const int originalYear = year;
    year -= month <= 2;
    const int era = year / 400;
    const unsigned yoe = unsigned(year - era * 400);
    const unsigned doy = (153 * (month > 2 ? month - 3 : month + 9) + 2) / 5 + day - 1;
    const int64_t nDays = int64_t(era) * 146097 + yoe * 365 + yoe / 4 - yoe / 100 + doy - 719468;
    const int64_t epoch = nDays * 86400 + tod / 1000;
    if (epoch > int64_t(std::numeric_limits<std::time_t>::max())) return false;
    fix.epoch = std::time_t(epoch);
    fix.millisecond = uint16_t(tod % 1000);
    fix.timeValid = true;
    std::snprintf(fix.utc, sizeof(fix.utc), "%04d-%02u-%02uT%02u:%02u:%02u.%03uZ",
        originalYear, month, day, unsigned(tod/3600000), unsigned(tod/60000%60),
        unsigned(tod/1000%60), unsigned(tod%1000));
    return true;
}
}

void DuckGPSFixStream::onFix(Callback callback, void* context) {
    callback_ = callback; context_ = context;
}
void DuckGPSFixStream::reset() {
    const auto cb = callback_; void* ctx = context_; const auto window = joinWindowMs_;
    *this = DuckGPSFixStream{};
    callback_ = cb; context_ = ctx; joinWindowMs_ = window;
}
bool DuckGPSFixStream::latest(DuckGPSFix& out) const {
    if (!haveLatest_) return false;
    out = latest_; return true;
}
bool DuckGPSFixStream::feed(char byte, uint32_t nowMs) {
    if (byte == '$') { collecting_ = true; length_ = 0; return false; }
    if (!collecting_) return false;
    if (byte == '\r' || byte == '\n') {
        collecting_ = false; line_[length_] = 0;
        return sentence(nowMs);
    }
    if (byte < 32 || byte > 126 || length_ == sizeof(line_) - 1) {
        collecting_ = false; ++rejected_; return false;
    }
    line_[length_++] = byte;
    return false;
}
bool DuckGPSFixStream::duplicate(uint32_t tod, std::time_t epoch, uint32_t nowMs) const {
    for (const auto& seen : seen_) {
        if (seen.valid && seen.tod == tod && uint32_t(nowMs-seen.at) < 60000 &&
            (!seen.epoch || !epoch || seen.epoch == epoch)) return true;
    }
    return false;
}
bool DuckGPSFixStream::emit() {
    if (!pending_) return false;
    pendingFix_.positionValid = (pendingFix_.hasGga || pendingFix_.hasRmc) &&
        (!pendingFix_.hasGga || ggaValid_) && (!pendingFix_.hasRmc || rmcValid_);
    pendingFix_.sequence = ++sequence_;
    latest_ = pendingFix_; haveLatest_ = true; pending_ = false;
    seen_[nextSeen_] = {latest_.timeOfDayMs, latest_.receivedAtMs, latest_.epoch, true};
    nextSeen_ = (nextSeen_ + 1) % 16;
    if (callback_) callback_(latest_, context_);
    return true;
}
bool DuckGPSFixStream::tick(uint32_t nowMs) {
    return pending_ && uint32_t(nowMs-pendingFix_.receivedAtMs) >= joinWindowMs_ ? emit() : false;
}
bool DuckGPSFixStream::sentence(uint32_t nowMs) {
    char* star = std::strchr(line_, '*');
    if (!star || std::strlen(star) != 3 || hex(star[1]) < 0 || hex(star[2]) < 0) { ++rejected_; return false; }
    uint8_t checksum = 0;
    for (char* p = line_; p < star; ++p) checksum ^= uint8_t(*p);
    if (checksum != ((hex(star[1]) << 4) | hex(star[2]))) { ++rejected_; return false; }
    ++validSentences_;
    *star = 0;
    char* fields[24] = {line_}; size_t count = 1;
    for (char* p = line_; *p; ++p) if (*p == ',') {
        *p = 0;
        if (count == 24) { ++rejected_; return false; }
        fields[count++] = p + 1;
    }
    if (std::strlen(fields[0]) != 5) return false;
    const bool gga = std::strcmp(fields[0] + 2, "GGA") == 0;
    const bool rmc = std::strcmp(fields[0] + 2, "RMC") == 0;
    if ((!gga && !rmc) || count < (gga ? 11u : 10u)) return false;
    uint32_t tod;
    if (!clockMs(fields[1], tod)) { ++rejected_; return false; }
    DuckGPSFix date;
    if (rmc) dateEpoch(fields[9], tod, date);
    if (duplicate(tod, date.epoch, nowMs)) return false;
    bool emitted = false;
    if (pending_ && (pendingFix_.timeOfDayMs != tod ||
        (date.timeValid && pendingFix_.timeValid && date.epoch != pendingFix_.epoch))) emitted = emit();
    if (!pending_) {
        pendingFix_ = {}; pendingFix_.receivedAtMs = nowMs; pendingFix_.timeOfDayMs = tod;
        pendingFix_.millisecond = uint16_t(tod % 1000);
        pending_ = true; ggaValid_ = rmcValid_ = false;
    }
    double lat = 0, lon = 0;
    if (gga && !pendingFix_.hasGga) {
        pendingFix_.hasGga = true;
        unsigned quality = 0, satellites = 0;
        const bool qualityOK = integer(fields[6], 8, quality);
        pendingFix_.quality = uint8_t(quality);
        ggaValid_ = qualityOK && quality > 0 && coordinate(fields[2], fields[3], true, lat) &&
            coordinate(fields[4], fields[5], false, lon);
        if (ggaValid_) { pendingFix_.latitude = lat; pendingFix_.longitude = lon; }
        pendingFix_.hasSatellites = integer(fields[7], 999, satellites);
        pendingFix_.satellites = uint16_t(satellites);
        pendingFix_.hasHdop = number(fields[8], pendingFix_.hdop) && pendingFix_.hdop >= 0;
        pendingFix_.hasAltitude = std::strcmp(fields[10], "M") == 0 && number(fields[9], pendingFix_.altitudeM);
    } else if (rmc && !pendingFix_.hasRmc) {
        pendingFix_.hasRmc = true;
        rmcValid_ = std::strcmp(fields[2], "A") == 0 && coordinate(fields[3], fields[4], true, lat) &&
            coordinate(fields[5], fields[6], false, lon);
        if (rmcValid_ && !pendingFix_.hasGga) { pendingFix_.latitude = lat; pendingFix_.longitude = lon; }
        pendingFix_.hasSpeed = number(fields[7], pendingFix_.speedKmph) && pendingFix_.speedKmph >= 0;
        pendingFix_.speedKmph *= 1.852;
        if (date.timeValid) {
            pendingFix_.epoch = date.epoch; pendingFix_.millisecond = date.millisecond;
            pendingFix_.timeValid = true; std::memcpy(pendingFix_.utc, date.utc, sizeof(date.utc));
        }
    }
    if (pendingFix_.hasGga && pendingFix_.hasRmc) emitted = emit() || emitted;
    return emitted;
}
