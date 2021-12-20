## What is Alfred?
Alfred is a self balanced two-wheels robot based around a Raspberry Pi. It can move autonomously or be remotely controlled.

## Aim of the project
This project's main goal is for me to learn (or consolidate) and apply new skills:
- ROS, OpenCV and Google Test
- C++, Rust and UML

## Hardware
Alfred is composed of the following components:
- 2 motors GA25-371 (with quadrature encoders)
- MDD3A drive
- [MPU6050](https://gitlab.com/arnixroboticslab/alfred/-/blob/master/Datasheets/MPU-6000-Datasheet1.pdf) accelerometer + gyroscope
- [HC-SR04](https://gitlab.com/arnixroboticslab/alfred/-/blob/master/Datasheets/HCSR04.pdf) ultrasonic sensor
- Raspberry Pi Camera V2
- Raspberry Pi 3B
- [TXB0104](https://gitlab.com/arnixroboticslab/alfred/-/blob/master/Datasheets/txb0104.pdf) level shifter
- UBEC with a 5V output
- 4 18650 batteries

The remote used to control Alfred is a [STM32F3 Discovery board](https://gitlab.com/arnixroboticslab/alfred/-/blob/master/Datasheets/STM32F3_Disc_manual.pdf) to which a [HC05](https://gitlab.com/arnixroboticslab/alfred/-/blob/master/Datasheets/HC05_DS.pdf) bluetooth module is connected.

## Tooling
[`ros_cross_compile`](https://github.com/ros-tooling/cross_compile) is used to compile the project (`C++¬) for the Raspberry Pi.
`Cargo` is used to compile the remote's code (`Rust`).