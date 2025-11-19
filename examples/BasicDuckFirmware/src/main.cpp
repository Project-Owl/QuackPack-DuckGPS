#include <Arduino.h>
#include <iostream>
#include <Ducks/MamaDuck.h>
#include <DuckGPS.h>
#include <utils/DuckUtils.h>

MamaDuck<DuckWifiNone,DuckLoRa> duck("MAMAGPS1");
DuckGPS dgps(34, 12); // RX, TX pins

void setup() {
    Wire.begin(21, 22);
    Serial.begin(115200);

    // Initialize the Duck

    duck.setupWithDefaults();; // or setupMamaDuck() / setupPapaDuck()
    // Initialize and configure your QuackPack
    dgps.setup();
    dgps.setBaudrate(115200);

    Serial.println("Duck ready!");
}

void loop() {
    // Run Duck mesh networking
    duck.run();

    dgps.readData(10000);

    std::cout << "Latitude: " << dgps.lat() << ", Longitude: " << dgps.lng() << '\n';
    std::cout << "Altitude: " << dgps.altitude(DuckGPS::AltitudeUnit::meter) << " meters" << '\n';
    std::cout << "Satellites: " << dgps.satellites() << '\n';
    std::cout << "Speed: " << dgps.speed(DuckGPS::SpeedUnit::kmph) << " km/h" << '\n';
    std::cout << "Time: " << dgps.epoch() << " epoch seconds" << '\n';
    std::cout << "GeoJSON Point: " << dgps.geoJsonPoint() << '\n';

    delay(5000); // Sleep for 60 seconds


}