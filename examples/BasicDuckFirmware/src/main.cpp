#include <Arduino.h>
#include <iostream>
#include <Ducks/MamaDuck.h>
#include <DuckGPS.h>

MamaDuck<DuckWifiNone,DuckLoRa> duck("MAMAGPS1");
DuckGPS dgps(34, 12); // RX, TX pins

void setup() {
    Wire.begin(21, 22);
    Serial.begin(115200);

    // Initialize the Duck

    duck.setupWithDefaults();; // or setupMamaDuck() / setupPapaDuck()
    // Initialize and configure your QuackPack
    dgps.setup();

    Serial.println("Duck ready!");
}

void loop() {
    dgps.readData(10000);
    // std::cout << "Latitude: " << dgps.lat() << ", Longitude: " << dgps.lng() << '\n';
    // std::cout << "Altitude: " << dgps.altitude(DuckGPS::AltitudeUnit::meter) << " meters" << '\n';
    // std::cout << "Speed: " << dgps.speed(DuckGPS::SpeedUnit::kmph) << " km/h" << '\n';
    // std::cout << "Time: " << dgps.epoch() << " epoch seconds" << '\n';
    // std::cout << "ISO 8601 Time: " << dgps.ISO8601() << '\n';
    // const auto geojson = dgps.geoJsonPoint();
    // std::cout << "GeoJSON Point: " << geojson << '\n';
    Serial.print("Local time: ");
    Serial.print(dgps.localHour(DuckGPS::zone::Central, false)); //central time zone, no daylight savings in costa rica
    Serial.print(":");
    Serial.print(dgps.minute());
    Serial.print(":");
    Serial.println(dgps.second());
    Serial.println("UTC hour: " + dgps.hour());
    std::cout << "ISO 8601 Time: " << dgps.ISO8601() << '\n';
    std::cout << "Satellites: " << dgps.satellites() << '\n';

    // duck.sendData(topics::gps, geojson);

    delay(5000); // Sleep for 60 seconds


}
