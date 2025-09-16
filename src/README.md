# MPU6050 and ESP32 Connections

This document outlines the hardware connections required to interface the MPU6050 accelerometer and gyroscope module with an ESP32 microcontroller.

## I2C Communication

The project utilizes the I2C communication protocol to read data from the MPU6050. The default I2C pins on the ESP32 are used.

### Required Connections:

| MPU6050 Pin | ESP32 Pin | Purpose          |
|-------------|-----------|------------------|
| VCC         | 3.3V      | Power Supply     |
| GND         | GND       | Ground           |
| SCL         | GPIO 22   | I2C Clock Line   |
| SDA         | GPIO 21   | I2C Data Line    |

**Note:** Ensure that the voltage supplied to the MPU6050 is 3.3V, as the ESP32 operates at this voltage level.

## Button

A push button can be connected for user input. 

### Connection:

| Component     | Connection            | ESP32 Pin |
|---------------|-----------------------|-----------|
| Push Button   | One side to GND       |           |
|               | Other side to GPIO 15 | GPIO 15   |

### Resistor:

The ESP32's internal pull-up resistor should be enabled for the button pin. This is done in the code by setting the pin mode to `INPUT_PULLUP`. This means no external pull-up or pull-down resistor is required.