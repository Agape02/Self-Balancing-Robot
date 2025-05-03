# Self-Balancing-Robot
This project implements a two-wheeled self-balancing robot using the BMI270 IMU sensor and the TLE94112 multi-half-bridge motor driver. The control algorithm is based on Kalman-filtered tilt angle estimation and a PID feedback loop, enabling the robot to maintain balance dynamically in response to disturbances.

You can run this code using: 
- Arduino IDE with installed Arduino_BMI270_BMM150 and multi-half-bridge libraries.
- Connect the board to motors and power supply (6–12V recommended).
- Ensure you adapt the code if your board does not support the mentioned architectures (AVR, XMC, SAMD, etc.).
