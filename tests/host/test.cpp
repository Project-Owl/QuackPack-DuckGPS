#include <DuckGPS.h>
#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <random>
#include <vector>

uint32_t fakeMillis = 0;
unsigned delayCalls = 0;
HardwareSerial Serial1;

std::string nmea(const std::string& body) {
    uint8_t checksum = 0;
    for (char c : body) checksum ^= uint8_t(c);
    char tail[8]; snprintf(tail,sizeof(tail),"*%02X\r\n",unsigned(checksum));
    return "$" + body + tail;
}
std::string gga(const std::string& t="120000.000", const std::string& quality="1",
                const std::string& lat="3348.123456", const std::string& hdop="0.85") {
    return nmea("GNGGA,"+t+","+lat+",N,11730.654321,W,"+quality+",12,"+hdop+",123.4,M,-30.0,M,,");
}
std::string rmc(const std::string& t="120000.000", const std::string& valid="A",
                const std::string& date="161126") {
    return nmea("GNRMC,"+t+","+valid+",3348.123456,N,11730.654321,W,1.5,0.0,"+date+",,,A");
}
void collect(const DuckGPSFix& fix, void* ctx) { static_cast<std::vector<DuckGPSFix>*>(ctx)->push_back(fix); }
void feed(DuckGPSFixStream& s, const std::string& bytes, uint32_t now) { for(char c:bytes) s.feed(c,now); }

void streamTests() {
    DuckGPSFixStream s; std::vector<DuckGPSFix> fixes; s.onFix(collect,&fixes);
    feed(s,gga()+rmc(),100);
    assert(fixes.size()==1);
    const auto first=fixes.back();
    assert(first.positionValid && first.timeValid && first.hasGga && first.hasRmc);
    assert(std::string(first.utc)=="2026-11-16T12:00:00.000Z");
    assert(std::abs(first.latitude - (33+48.123456/60))<1e-10);
    assert(first.longitude<0 && first.hasHdop && first.hdop==0.85);
    assert(first.hasAltitude && first.altitudeM==123.4 && first.satellites==12);
    feed(s,rmc()+gga(),150); assert(fixes.size()==1); // copies across talkers/paths
    feed(s,rmc("120001.250")+gga("120001.250"),1100);
    assert(fixes.size()==2 && fixes.back().millisecond==250);
    // Several complete epochs in one UART-like batch, no loss.
    feed(s,gga("120002.000")+rmc("120002.000")+gga("120003.000")+rmc("120003.000"),2000);
    assert(fixes.size()==4 && fixes.back().sequence==4);
    // Missing GGA is not filled with last epoch's altitude/HDOP/satellites.
    feed(s,rmc("120004.000"),3000); s.tick(3249); assert(fixes.size()==4);
    s.tick(3250); assert(fixes.size()==5 && !fixes.back().hasHdop && !fixes.back().hasAltitude);
    feed(s,gga("120004.000"),3300); assert(fixes.size()==5); // late partner
    feed(s,gga("120005.000"),4000); s.tick(4250);
    assert(fixes.size()==6 && fixes.back().positionValid && !fixes.back().timeValid);
    // No fix invalidates current position; quality still logged, no false 0,0 fix.
    feed(s,gga("120006.000","0")+rmc("120006.000","V"),5000);
    assert(fixes.size()==7 && !fixes.back().positionValid && fixes.back().timeValid);
    // Malformed coordinate and non-decimal uncertainty do not become valid.
    feed(s,gga("120007.000","1","9961.00","nan")+rmc("120007.000"),6000);
    assert(!fixes.back().positionValid && !fixes.back().hasHdop);
    feed(s,rmc("120008.000","A","310226"),7000); s.tick(7250);
    assert(!fixes.back().timeValid && fixes.back().positionValid);
    // Invalid checksum, oversized/truncated sentence, binary noise recover.
    auto count=fixes.size();
    auto bad=gga("120009.000"); bad[bad.size()-4]='X';
    feed(s,bad+"$"+std::string(300,'x')+"\r\n$truncated",8000);
    feed(s,gga("120010.000")+rmc("120010.000"),8100);
    assert(fixes.size()==count+1 && s.rejected()>=2);
    // UTC midnight carries the RMC's own date, never a previous date.
    feed(s,gga("235959.000")+rmc("235959.000","A","161126"),9000);
    const auto epoch=fixes.back().epoch;
    feed(s,rmc("000000.000","A","171126")+gga("000000.000"),10000);
    assert(fixes.back().epoch==epoch+1);
    // Elapsed-time wrap is safe for join timeout.
    s.reset(); fixes.clear(); feed(s,rmc(),UINT32_MAX-100); s.tick(200);
    assert(fixes.size()==1);
    // Structurally valid but unsupported time is rejected, not relabelled.
    feed(s,gga("246000.000")+rmc("246000.000"),500); assert(fixes.size()==1);
    feed(s,gga("120001.000","1","1")+rmc("120001.000"),600);
    assert(!fixes.back().positionValid);
    // Bounded parser under arbitrary malformed UART input.
    std::mt19937 random(76);
    for(unsigned i=0;i<100000;++i) s.feed(char(random()%256),i);
    s.reset(); fixes.clear(); feed(s,gga()+rmc(),100);
    assert(fixes.size()==1 && fixes.back().sequence==1);
}

void startupTests() {
    HardwareSerial uart; DuckGPS gps(uart); DuckGPS::Options opt;
    opt.receiver=DuckGPS::Receiver::L76K; opt.rxPin=9; opt.txPin=8; opt.constellations=7;
    opt.powerSettleMs=1000;
    fakeMillis=0; delayCalls=0;
    assert(gps.start(opt)); assert(fakeMillis==0 && !uart.opened);
    fakeMillis=999; gps.update(); assert(!uart.opened);
    fakeMillis=1000; gps.update(); assert(uart.opened && uart.rxPin==9 && uart.txPin==8);
    assert(uart.output.empty()); // passive detection: no UBX, PCAS or PMTK probe
    uart.inject(gga()+rmc()); gps.update();
    assert(gps.status().receiverHeard && !gps.status().commandsSent);
    fakeMillis+=100; gps.update(); fakeMillis+=100; gps.update();
    assert(gps.status().commandsSent);
    std::string commands(uart.output.begin(),uart.output.end());
    assert(commands==nmea("PCAS02,1000")+nmea("PCAS03,1,0,0,0,1,0,0,0,0,0,,,0,0")+nmea("PCAS04,7"));
    uart.inject(gga("120001.000")+rmc("120001.000")); gps.update();
    fakeMillis+=1000; uart.inject(gga("120002.000")+rmc("120002.000")); gps.update();
    assert(gps.status().state==DuckGPS::State::Ready && gps.status().oneHzObserved && gps.hasFix());
    assert(delayCalls==0);
    char json[DuckGPS::JSON_MAX_LEN]; assert(gps.toJson(json,sizeof(json),123)>0);
    assert(std::strstr(json,"33.802058") && std::strstr(json,"12:00:02.000Z"));
    // Fresh time-only RMC with V status cannot make old position current.
    fakeMillis+=1000; uart.inject(rmc("120003.000","V")); gps.update();
    fakeMillis+=250; gps.update(); assert(!gps.hasFix());
    gps.toJson(json,sizeof(json)); assert(std::strstr(json,"No GPS fix") && !std::strstr(json,"lat"));
    // A stale valid snapshot retains original time in JSON, including absent new date.
    uart.inject(gga("120004.000")+rmc("120004.000")); gps.update(); fakeMillis+=6000;
    gps.toJson(json,sizeof(json)); assert(std::strstr(json,"stale") && std::strstr(json,"12:00:04.000Z"));
    char tiny[3]; assert(gps.toJson(tiny,sizeof(tiny))==0 && tiny[0]==0);
    gps.end(); assert(!uart.opened && gps.status().state==DuckGPS::State::Idle);
    // Read-only path never transmits or waits, and callback sees burst epochs.
    opt.receiver=DuckGPS::Receiver::NmeaOnly; opt.powerSettleMs=0; opt.txPin=-1;
    opt.maxBytesPerUpdate=64; uart.output.clear();
    std::vector<DuckGPSFix> fixes; gps.onFix(collect,&fixes);
    assert(gps.start(opt)); gps.update();
    uart.inject(gga()+rmc()+gga("120001.000")+rmc("120001.000"));
    auto queued=uart.input.size(); gps.update(); assert(queued-uart.input.size()==64);
    while(uart.available()) gps.update();
    assert(fixes.size()==2 && uart.output.empty() && delayCalls==0);
    // Late receiver wakes after transport timeout; no periodic reset needed.
    gps.start(opt); gps.update(); fakeMillis+=1500; gps.update();
    assert(gps.status().state==DuckGPS::State::TimedOut);
    uart.inject(gga()+rmc()); while(uart.available()) gps.update();
    assert(gps.status().state==DuckGPS::State::Ready);
    // TX backpressure cannot block update. It times out rather than waiting.
    opt.receiver=DuckGPS::Receiver::L76K; opt.txPin=8; opt.maxBytesPerUpdate=512;
    uart.txCapacity=0; gps.start(opt); gps.update(); uart.inject(gga()+rmc()); gps.update();
    for(unsigned i=0;i<100;++i) { fakeMillis+=50; gps.update(); }
    assert(!gps.status().commandsSent && delayCalls==0);
    // Invalid options are rejected without opening the port.
    opt.constellations=8; assert(!gps.start(opt)); assert(!uart.opened);
    assert(gps.status().state==DuckGPS::State::InvalidOptions);
    // Existing fixed-baud read-only API still compiles and captures snapshots.
    uart.txCapacity=128; assert(gps.begin(34,12,9600,false));
    uart.inject(gga()+rmc()); gps.update(); assert(gps.hasFix());
    assert(gps.baud()==9600 && !gps.isUblox());
    // Passive auto scan changes baud on elapsed time, without probe commands.
    uart.output.clear(); opt={}; opt.baud=0; opt.powerSettleMs=0;
    gps.start(opt); gps.update(); fakeMillis+=1500; gps.update();
    assert(gps.baud()==115200 && uart.output.empty());
    uart.inject(gga()+rmc()); gps.update(); assert(gps.status().receiverHeard);
    assert(delayCalls==0);
    // Existing UBX auto-detection + ACK setup: actual legacy path, fake UART.
    uart.onWrite=[&](const uint8_t*, size_t n) {
        if(n!=2 || uart.output.size()<8) return;
        // The legacy writer completes its frame with the two checksum bytes.
        size_t start=uart.output.size()-8;
        while(start && !(uart.output[start]==0xB5 && uart.output[start+1]==0x62)) --start;
        const uint8_t cls=uart.output[start+2], id=uart.output[start+3];
        uint8_t ack[]={0xB5,0x62,0x05,0x01,0x02,0x00,cls,id,0,0};
        for(size_t i=2;i<8;++i){ack[8]+=ack[i];ack[9]+=ack[8];}
        uart.input.insert(uart.input.end(),ack,ack+sizeof(ack));
    };
    uart.output.clear(); assert(gps.begin(34,12)); assert(gps.isUblox());
    uart.inject(gga()+rmc()); gps.update(); assert(gps.hasFix());
}

int main() {
    streamTests(); startupTests();
    std::cout << "PASS: fix assembly, malformed streams, async startup, JSON, legacy API and bounded service\n";
}
