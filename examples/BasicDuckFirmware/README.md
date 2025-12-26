
## Configuration

### Method 1: Programmatic Configuration (Recommended)

Configure directly in your `setup()`:

```cpp
void setup() {
    duck.begin();
    duck.setDeviceId("DUCKGPS1");
    duck.setupWithDefaults();
}
```

## API Reference

### Main Class: `DuckGPS`

#### Constructor
```cpp
DuckGPS(RXPin, TXPin);
```

Based on the `DuckGPS` QuackPack template, here are the methods and their documentation:

## Core Methods

### Constructor
```cpp
DuckGPS(uint8_t rxPin, uint8_t txPin);
```
Initializes the GPS module with specified RX and TX pins for serial communication.

**Parameters:**
- `rxPin`: GPIO pin for receiving GPS data
- `txPin`: GPIO pin for transmitting to GPS module

### Initialization
```cpp
void setup();
```
Initializes the GPS module and configures the ublox chip for operation.

### Data Acquisition
```cpp
void readData(unsigned long timeout_ms);
```
Reads and parses GPS data from the module.

**Parameters:**
- `timeout_ms`: Maximum time in milliseconds to wait for GPS data

### Position Methods
```cpp
double lat();
double lng();
```
Returns the current latitude and longitude in decimal degrees.

```cpp
double altitude(DuckGPS::AltitudeUnit unit);
```
Returns altitude in the specified unit.

**Parameters:**
- `unit`: `DuckGPS::AltitudeUnit::meter` or `DuckGPS::AltitudeUnit::feet`

### Status Methods
```cpp
uint8_t satellites();
```
Returns the number of satellites currently in view/used.

```cpp
double speed(DuckGPS::SpeedUnit unit);
```
Returns current speed in the specified unit.

**Parameters:**
- `unit`: `DuckGPS::SpeedUnit::kmph`, `DuckGPS::SpeedUnit::mph`, or `DuckGPS::SpeedUnit::knots`

### Time Methods
```cpp
unsigned long epoch();
```
Returns current GPS time as Unix epoch timestamp (seconds since Jan 1, 1970).

### Format Methods
```cpp
String geoJsonPoint();
```
Returns current position formatted as a GeoJSON Point string for easy integration with mapping systems.
