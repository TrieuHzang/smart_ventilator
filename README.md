# Temperature-Based Smart Fan System

## Overview
This project is an Arduino-based embedded system that automatically controls a fan according to temperature conditions. It is designed to improve comfort and energy efficiency by adjusting fan operation based on environmental data.

## Features
- Automatic fan control based on temperature
- Real-time sensor data reading
- WiFi-based monitoring or remote control
- Embedded control logic for responsive operation

## Hardware / Platform
- Arduino / ESP-based controller
- Temperature sensor
- Fan control module
- WiFi communication module
- Power supply for embedded operation

## Software / Tools
- Embedded C / C++
- Arduino IDE
- Sensor data processing
- Functional testing and debugging

## How It Works
The system continuously reads temperature data from the sensor and compares it with predefined thresholds. When the temperature exceeds a set level, the fan is activated automatically. When the temperature returns to a normal range, the fan is turned off or adjusted according to the control logic.

## Repository Structure
- `smart_ventilator.ino`: main application logic
- `README.md`: project description

## My Role
- Developed the embedded control logic for automatic fan operation
- Processed temperature sensor data
- Implemented threshold-based fan control behavior
- Tested and debugged the system

## Future Improvements
- Add display for real-time temperature monitoring
- Support multiple fan speed levels
- Integrate WiFi for remote monitoring and control
