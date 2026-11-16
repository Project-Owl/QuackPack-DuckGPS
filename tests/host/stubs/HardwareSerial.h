#pragma once
#include <Arduino.h>
#include <deque>
#include <functional>
#include <string>
#include <vector>
class HardwareSerial {
public:
    std::deque<uint8_t> input;
    std::vector<uint8_t> output;
    std::vector<uint32_t> baudHistory;
    std::function<void(uint32_t)> onBaud;
    std::function<void(const uint8_t*, size_t)> onWrite;
    int txCapacity = 128, rxPin = -1, txPin = -1;
    bool opened = false;
    size_t setRxBufferSize(size_t n) { return n; }
    void begin(uint32_t baud, int, int rx, int tx) { opened = true; rxPin=rx; txPin=tx; updateBaudRate(baud); }
    void end() { opened = false; input.clear(); }
    void updateBaudRate(uint32_t baud) { baudHistory.push_back(baud); if (onBaud) onBaud(baud); }
    int available() { return int(input.size()); }
    int availableForWrite() { return txCapacity; }
    int read() { if (input.empty()) return -1; const int c=input.front(); input.pop_front(); return c; }
    size_t write(const uint8_t* p, size_t n) { output.insert(output.end(),p,p+n); if(onWrite) onWrite(p,n); return n; }
    void inject(const std::string& s) { input.insert(input.end(),s.begin(),s.end()); }
};
extern HardwareSerial Serial1;
