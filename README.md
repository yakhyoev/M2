# M2 — Remote Fiber Port Testing Robot

M2 (Machine2) is an experimental robotic tool that positions and inserts a fiber-optic test connector into selected ports in a fiber distribution hub (FDH). It combines a three-axis mechanism, camera-based alignment, and remote control to reduce repeated manual connections during last-mile fiber testing.

The project grew out of hands-on fiber construction and splicing work. Its goal is to let one technician control connections at the FDH while working elsewhere in the network, reducing trips between the cabinet and the terminal being tested.

**Status:** Working prototype with field trials; development is ongoing. This repository contains the Arduino and Raspberry Pi code used to develop and test the system. The software is specific to the prototype hardware and requires configuration and calibration before use.

[Project background and development updates](https://www.lightsteplab.com/rd/m2)

## The problem

Testing a fiber distribution network often involves repeatedly connecting a test instrument to different ports at the FDH while checking fibers at downstream terminals. With one technician, this can mean frequent travel back to the cabinet. With two technicians, one may spend much of the test process moving the test connection between ports.

M2 explores whether a compact, remotely operated mechanism can handle that repetitive connection task.

## How it works

1. The operator selects a port remotely.
2. The X/Y gantry moves the toolhead to the port's calibrated position.
3. A toolhead camera captures the connector area, and the Raspberry Pi estimates the alignment error.
4. The Arduino receives correction commands and adjusts the toolhead position.
5. The Z-axis plunger inserts the test connector. The attached optical instrument performs the measurement or supplies the test signal.
6. The connector retracts before moving to another port.

M2 handles the physical connection. Optical measurements are provided by an external instrument, such as an optical time-domain reflectometer (OTDR), optical loss test set (OLTS), or light source.

## Hardware

| Component | Role |
| --- | --- |
| Three-axis gantry | X/Y port positioning and Z-axis connector insertion |
| Arduino Mega | Motor control, homing, and communication with the Raspberry Pi |
| Raspberry Pi Zero 2 W | Camera processing and remote-control software |
| OV5647-based compact camera | Close-up imaging for connector alignment |
| Stepper motors and drivers | Gantry and insertion mechanism motion |
| Homing sensors | Establish repeatable axis reference positions |
| Toolhead illumination | Improve visibility around the target port |
| SC test connector and patch cord | Connect the external test instrument to the selected port |

The prototype geometry has been developed around Clearfield FDH layouts with 288 or 432 SC ports. Adapting it to another cabinet requires checking mechanical clearance, port spacing, mounting, and connector insertion geometry.

## Software

The code is divided between two controllers:

- **Arduino firmware:** Controls the motion mechanism, manages homing and insertion sequences, and exchanges commands and alignment corrections with the Raspberry Pi.
- **Raspberry Pi software:** Captures camera images, uses OpenCV to locate connector features and estimate positioning errors, and supports remote commands and diagnostic output.

Development has also included a Flask-based control interface and saved images for reviewing alignment performance. The available functions depend on the code revision in the repository.

## Getting started

This repository is currently a reference implementation for the M2 prototype, rather than a complete assembly kit. Before running the code on another build:

1. Review the firmware and Python files to identify their required libraries and hardware connections.
2. Match the motor, driver, sensor, camera, and serial settings to your hardware.
3. Configure axis directions, travel limits, steps per millimeter, and the port grid coordinates.
4. Verify homing and movement with the test connector retracted.
5. Calibrate camera alignment and insertion depth using a spare connector panel before testing an installed cabinet.

Port coordinates, correction thresholds, and motion settings are tied to the physical build. Copying them without calibration can cause misalignment or connector damage.

## Field experience and current limitations

Field trials have demonstrated the basic positioning and connection concept and exposed several areas that need improvement:

- **Lighting:** Direct sunlight can overexpose the camera image and reduce connector detection reliability.
- **Alignment:** Repeated correction can overshoot; camera-to-motion calibration and correction behavior need further refinement.
- **Insertion:** Small positioning errors or mechanical variation can prevent the connector from seating completely.
- **Communication:** Remote connection latency and Raspberry Pi–Arduino communication need to be reliable throughout the operation.
- **Packaging:** The camera, Raspberry Pi, wiring, and moving toolhead must fit within the available cabinet space.

Reliable field operation remains the main development priority. Performance targets should be treated as goals until validated through repeatable testing.

## Development direction

- Improve connector detection under changing daylight conditions.
- Make alignment corrections and insertion more repeatable.
- Improve communication recovery and operational feedback.
- Reduce toolhead size and simplify moving cables.
- Evaluate a more compact controller and electronics layout.
- Document assembly, calibration, and field-test results so others can reproduce and evaluate the approach.

## Feedback and collaboration

Feedback from fiber technicians, mechanical designers, controls developers, and equipment manufacturers is welcome. Useful contributions include field use cases, alternative connector mechanisms, vision improvements, and reproducible bug reports.

For an issue, include the code revision, hardware configuration, steps to reproduce the problem, and relevant Arduino/Raspberry Pi logs or camera images.

More background is available at [LightStepLab — M2](https://www.lightsteplab.com/rd/m2).
