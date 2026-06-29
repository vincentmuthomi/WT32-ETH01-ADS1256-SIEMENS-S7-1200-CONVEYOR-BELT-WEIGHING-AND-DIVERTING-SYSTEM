
# Conveyor Belt Dynamic Weighing System

Project by: Vincent Muthomi - Mechatronics YR - 2026 Engineering Graduate JKUAT
Contact: vincentmuthomi698@gmail.com

This project implements an interface between IoT and industrial automation for a conveyor-belt-based dynamic weighing system. It combines an ESP32-based WT32-ETH01 controller, dual load cells, an ADS1256 ADC, and a Siemens PLC running ladder logic in TIA Portal. The result is a system that can measure moving objects, publish the detected weight data over Modbus TCP, and coordinate downstream actions such as solenoid valve actuation.

## System Overview

The embedded side of the project reads two load cells through the ADS1256 analog-to-digital converter. The ESP32 firmware performs the following tasks in a continuous loop:

1. Samples each load cell through the ADS1256 MUX.
2. Rejects sudden spikes with outlier damping.
3. Smooths readings using a rolling average.
4. Auto-tares the cells during startup and can also accept manual tare and calibration commands over Serial.
5. Publishes processed weight values through Modbus TCP for the PLC to consume.

The PLC side monitors the conveyor sensors, calculates timing and position relationships, and uses the incoming weight information to trigger outputs. The PLC logic also computes speed, object span, and delay windows so that actuators fire at the correct moment as items move past the sensors and weighing zone.

## Additional Hardware

To build the full working conveyor installation, the control electronics must be paired with the actual plant equipment. This includes a network switch for the industrial Ethernet connection, pneumatic cylinders for the mechanical actuation points, a compressed air supply or compressor for cylinder operation, and relays/contactors for switching and isolation. The actual conveyor belt, photoelectric sensors, and load cells, and the motor are part of the physical system but are not included in the electronics/software repository. These devices complete the physical automation layer that the PLC and ESP32 logic control.

## How the Embedded Weighing Works

The Arduino/ESP32 code is centered around two load cells defined in [LoadCellState.h](LoadCellState.h). Each cell has its own calibration factor, tare offset, averaging buffer, and outlier window. In [Weight_Measurements.ino](Weight_Measurements.ino), the ADS1256 is initialized with a high sample rate and the code repeatedly reads the selected MUX channel, discarding initial samples after each channel switch to reduce residue from the previous input.

Once raw counts are collected, the firmware applies:

- outlier clamping to prevent abrupt jumps from polluting the signal,
- rolling averaging to stabilize the measurement,
- auto-tare logic to establish a zero reference,
- weight conversion using the stored calibration factor.

The current weights from both cells are then combined and forwarded to the communication layer. [LoadCell_circular_buffer.ino](LoadCell_circular_buffer.ino) keeps a history of transmitted readings, which is useful for logging or future analysis.

## Modbus and Network Interface

[WT32_MODBUS.ino](WT32_MODBUS.ino) provides the network interface. It starts Ethernet, falls back to Wi-Fi availability checks, and exposes Modbus holding registers. The project writes processed weights into holding registers so the PLC can access them in real time. Two operating modes are supported:

- normal mode, where live load-cell weights are transmitted,
- test mode, where a synthetic weight value can be injected for commissioning and debugging.

The serial command interface is also important for commissioning. It allows manual tare, calibration factor updates, and switching between operation modes without reflashing the firmware.

## PLC Role in the Conveyor System

The PLC program shown in the TIA Portal screenshots acts as the timing and orchestration layer. It reads the Modbus data and combines it with photoelectric sensor inputs to determine:

- when an object has entered or exited the weighing section,
- how long the object occupies the sensing span,
- the belt speed based on sensor timing,
- when to start and stop solenoid valve outputs,
- how to align actuation with the object's location.

The ladder logic uses counters, timers, move instructions, and arithmetic blocks to convert raw sensor events into practical conveyor decisions. In other words, the ESP32 measures the load, while the PLC decides when to act on that load in the industrial process.

## PLC Integration Main [OB1]

The MAIN [OB1] block is the top-level sweep that ties the entire conveyor system together. It sits above the lower-level blocks and executes the ladder logic progression shown in the PDF from left to right and network to network. In this project, OB1 is the PLC coordinator that turns raw signals from the conveyor, the weight data from the ESP32, and the timing rules from the buffer data into a single control flow.

The progression inside OB1 is roughly as follows:

1. Establish the Modbus request cycle and connect the PLC client to the ESP32 data source.
2. Latch photoelectric sensor events so object entry and exit are captured reliably.
3. Count sensor activity and measure the time between photoelectric sensors to estimate object span and belt timing.
4. Calculate belt speed from the sensor timing and transfer the result into the shared buffer structure.
5. Convert the received weight and threshold values into the formats needed by the ladder logic.
6. Compute the delay to each solenoid valve using belt speed and the physical distance stored in the buffer DB.
7. Start and stop the solenoid timers so the valve actuation happens when the object reaches the target position.
8. Handle reset conditions after the object leaves the sensing zone, then prepare the next cycle.
9. Activate the structured context function block [Block_2] and advance the sample index used for its internal processing.

This means OB1 is not doing the weighing itself. Instead, it orchestrates the full sequence around the weighing system: it pulls the latest data, evaluates conveyor motion, decides when the object is in the correct position, and triggers output actions at the right time. The ESP32 side provides the measured weight, while OB1 uses that data together with the photoelectric sensors to complete the industrial conveyor logic.

All other associated blocks shown in the pdfs are databases and buffers that integrate cleanly into the Main [OB1] logic

## Intended Use

This project is best understood as a hybrid IoT and automation demo for dynamic weighing on a conveyor. It demonstrates how a low-cost embedded controller can collect sensor data, publish it over a standard industrial protocol, and integrate with a PLC that handles deterministic machine logic. The architecture is suitable for weighing, sorting, threshold-based actuation, and conveyor monitoring applications.

## Project Files

- [LoadCellState.h](LoadCellState.h): shared state and configuration for load cells, buffers, and communication flags.
- [Weight_Measurements.ino](Weight_Measurements.ino): ADC reading, filtering, tare, calibration, and weight conversion.
- [LoadCell_circular_buffer.ino](LoadCell_circular_buffer.ino): circular history buffer for readings.
- [WT32_MODBUS.ino](WT32_MODBUS.ino): Ethernet, Modbus register handling, and output publishing.
- [Transmit_Weight_through_ModBus.ino](Transmit_Weight_through_ModBus.ino): main loop tying together sensing, filtering, and Modbus transmission.

Together, these components form a conveyor weighing interface that bridges industrial PLC control with IoT-style sensor acquisition and data exchange.

