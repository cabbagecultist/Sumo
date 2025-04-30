# VL53L4CD library for Arduino
[www.pololu.com](https://www.pololu.com/)

## Summary

This is a library for the Arduino IDE that helps interface with ST's [VL53L4CD time-of-flight distance sensor](https://www.pololu.com/product/3692). The library makes it simple to configure the sensor and read range data from it via I&sup2;C.

## Supported platforms

This library is designed to work with the Arduino IDE versions 1.6.x or later; we have not tested it with earlier versions.  This library should support any Arduino-compatible board, including the [Pololu A-Star controllers](https://www.pololu.com/category/149/a-star-programmable-controllers).

## Getting started

### Hardware

A [VL53L4CD carrier](https://www.pololu.com/product/3692) can be purchased from Pololu's website.  Before continuing, careful reading of the [product page](https://www.pololu.com/product/3692) as well as the VL53L4CD datasheet is recommended.

Make the following connections between the Arduino and the VL53L4CD board:

#### 5V Arduino boards

(including Arduino Uno, Leonardo, Mega; Pololu A-Star 32U4)

    Arduino   VL53L4CD board
    -------   --------------
         5V - VIN
        GND - GND
        SDA - SDA
        SCL - SCL

#### 3.3V Arduino boards

    Arduino   VL53L4CD board
    -------   --------------
        3V3 - VIN
        GND - GND
        SDA - SDA
        SCL - SCL

### Software

If you are using version 1.6.2 or later of the [Arduino software (IDE)](http://www.arduino.cc/en/Main/Software), you can use the Library Manager to install this library:

1. In the Arduino IDE, open the "Sketch" menu, select "Include Library", then "Manage Libraries...".
2. Search for "VL53L4CD".
3. Click the VL53L4CD entry in the list.
4. Click "Install".

If this does not work, you can manually install the library:

1. Download the [latest release archive from GitHub](https://github.com/pololu/vl53l4cd-arduino/releases) and decompress it.
2. Rename the folder "vl53l4cd-arduino-master" to "VL53L4CD".
3. Move the "VL53L4CD" folder into the "libraries" directory inside your Arduino sketchbook directory.  You can view your sketchbook location by opening the "File" menu and selecting "Preferences" in the Arduino IDE.  If there is not already a "libraries" folder in that location, you should make the folder yourself.
4. After installing the library, restart the Arduino IDE.

## Examples

Several example sketches are available that show how to use the library. You can access them from the Arduino IDE by opening the "File" menu, selecting "Examples", and then selecting "VL53L4CD". If you cannot find these examples, the library was probably installed incorrectly and you should retry the installation instructions above.

## ST's VL53L4CD ULD API and this library

Most of the functionality of this library is based on the [VL53L4CD Ultra Lite Driver (ULD) API](https://www.st.com/en/embedded-software/stsw-img026.html) provided by ST (STSW-IMG026), and some of the explanatory comments in the code are quoted or paraphrased from the API source code, API user manual (UM2931), and the VL53L4CD datasheet. For more explanation about the library code and how it was derived from the API, see the comments in the source files.

 Compared to ST's ULD API, this library is intended to provide a slightly more streamlined interface with potentially smaller storage and memory footprints. However, it does not currently implement some of the more advanced functionality available in the API, and it has less robust error checking. For advanced applications, especially when storage and memory are less of an issue, consider using the VL53L4CD ULD API directly. The [VL53L4CD Arduino library by STM32duino](https://github.com/stm32duino/VL53L4CD) is an alternative library that uses the ULD API.

## Library reference

* `RangingData ranging_data`<br>
  This struct contains information about the last ranging measurement. Its members are:
  * `uint16_t range_mm`<br>
    Range reading from the last measurement, in millimeters. (This reading can also be obtained as the return value of `read()`.)
  * `uint8_t range_status`<br>
    Status of the last measurement; see the API user manual UM2931 for descriptions of the possible statuses. A status of 0 means there were no problems with the measurement.
  * `uint8_t number_of_spad`<br>
    The number of SPADs (single photon avalanche diodes) enabled for the last measurement. More SPADs will be activated for sensing distant and less reflective targets.
  * `uint16_t signal_rate_kcps`<br>
    Quantity of photons measured during the last measurement, in units of kilo counts per second.
  * `uint16_t ambient_rate_kcps`<br>
    Ambient rate of the last measurement, in units of kilo counts per second.
  * `uint16_t signal_per_spad_kcps`<br>
    Signal rate divided by number of SPADs.
  * `uint16_t ambient_per_spad_kcps`<br>
    Ambient rate divided by number of SPADs.
  * `uint16_t sigma_mm`<br>
    An estimate of the noise (standard deviation) in the last range measurement.

* `uint8_t last_status`<br>
  The status of the last I&sup2;C write transmission. See the [`Wire.endTransmission()` documentation](http://arduino.cc/en/Reference/WireEndTransmission) for return values.

* `VL53L4CD()`<br>
  Constructor.

* `void setBus(TwoWire * bus)`<br>
  Configures this object to use the specified I&sup2;C bus. `bus` should be a pointer to a `TwoWire` object; the default bus is `Wire`, which is typically the first or only I&sup2;C bus on an Arduino. If your Arduino has more than one I&sup2;C bus and you have the VL53L4CD connected to the second bus, which is typically called `Wire1`, you can call `sensor.setBus(&Wire1);`.

* `TwoWire * getBus()`<br>
  Returns a pointer to the I&sup2;C bus this object is using.

* `void setAddress(uint8_t new_addr)`<br>
  Changes the I&sup2;C target device address of the VL53L4CD to the given value (7-bit).

* `uint8_t getAddress()`<br>
  Returns the current I&sup2;C address.

* `bool init(bool io_2v8 = true, bool fast_mode_plus = false)`<br>
  Initializes and configures the sensor. The return value is a boolean indicating whether the initialization completed successfully.
  * If the optional argument `io_2v8` is true, the sensor is configured for 2V8 mode (2.8 V I/O); if false, the sensor is left in 1V8 mode. The default if not specified is true (2V8 mode). (Pololu's carrier board shifts the I&sup2;C lines to 2.8 V.)
  * If the optional argument `fast_mode_plus` is true, the sensor is configured for I&sup2;C Fast-mode Plus (up to 1 MHz); if false, the sensor is left configured for Fast-mode (up to 400 KHz). The default if not specified is false (Fast-mode, up to 400 kHz).

* `void writeReg(uint16_t reg, uint8_t value)`<br>
  Writes an 8-bit sensor register with the given value.

  Register address constants are defined by the `regAddr` enumeration type in VL53L4CD.h.<br>
  Example use: `sensor.writeReg(VL53L4CD::SYSTEM_START, 0x80);`

* `void writeReg16Bit(uint16_t reg, uint16_t value)`<br>
  Writes a 16-bit sensor register with the given value.

* `void writeReg32Bit(uint16_t reg, uint32_t value)`<br>
  Writes a 32-bit sensor register with the given value.

* `uint8_t readReg(uint16_t reg)`<br>
  Reads an 8-bit sensor register and returns the value read.

* `uint16_t readReg16Bit(uint16_t reg)`<br>
  Reads a 16-bit sensor register and returns the value read.

* `uint32_t readReg32Bit(uint16_t reg)`<br>
  Reads a 32-bit sensor register and returns the value read.

* `bool setRangeTiming(uint8_t timing_budget_ms, uint32_t inter_measurement_ms)`<br>
  Sets the measurement timing budget and inter-measurement period to the given values in milliseconds. See the API user manual UM2931 for more information on range and timing limits. The return value is a boolean indicating whether the requested budget was valid.
  * The timing budget is the time allowed for one range measurement; a longer timing budget allows for more accurate measurements. Values from 10 ms to 200 ms are allowed.
  * The inter-measurement period is the time between the start of two consecutive measurements. This can be set to 0 to make the sensor take readings as often as the timing budget allows (continuous mode), or it can be set to a value greater than the timing budget (up to a maximum of 5000 ms) for less frequent readings (autonomous low power mode).

* `uint8_t getTimingBudget()`<br>
  Returns the current measurement timing budget in milliseconds.

* `uint32_t getInterMeasurement()`<br>
  Returns the current inter-measurement period in milliseconds.

* `void startContinuous()`<br>
  Starts continuous ranging measurements.

* `void stopContinuous()`<br>
  Stops continuous ranging measurements.

* `uint16_t read(bool blocking = true)`<br>
  After continuous ranging measurements have been started, calling this function returns a range reading in millimeters and updates the `ranging_data` struct with details about the last measurement. If the optional argument `blocking` is true (the default if not specified), this function will wait until data from a new measurement is available before returning.

  If you do not want this function to block, you can use the `dataReady()` function to check if new data is available before calling `read(false)`. Calling `read(false)` before new data is available results in undefined behavior.

* `uint16_t readRangeContinuousMillimeters(bool blocking = true)`<br>
  Alias of `read()` for convenience.

* `bool dataReady()`<br>
  Returns a boolean indicating whether data from a new measurement is available from the sensor.

* `void clearInterrupt()`<br>
  Clears any active interrupt from the sensor.

* `void setTimeout(uint16_t timeout)`<br>
  Sets a timeout period in milliseconds after which read operations will abort if the sensor is not ready. A value of 0 disables the timeout.

* `uint16_t getTimeout()`<br>
  Returns the current timeout period setting.

* `bool timeoutOccurred()`<br>
  Indicates whether a read timeout has occurred since the last call to `timeoutOccurred()`.

## Version history

* 1.0.0 (2024-08-20): Original release.
