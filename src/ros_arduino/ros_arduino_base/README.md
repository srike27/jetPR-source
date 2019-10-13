# Overview #

ros_arduino_base provide a basic interface two wheel differential mobile base.  
## On-board (the micro-controller) ##
- Velocity control is done on-board using encoders based on a differential velocity command. 
- Also, the encoder data is sent to an off-board node for processing.  

## Off-board (the PC) ##
- Off-board, the encoder data is processed to determine the pose of base.  
- In addition, the standard velocity topic (twist) is converted to left and right velocities.

## Current motor driver connections ##
- leftPWM 8
- leftDIR 22
- leftBRK 10

- rightPWM 9
- rightDIR 23
- rightBRK 11

## Current Encoder Connections ##
- leftENCA 2
- leftENCB 3

- rightENCA 19
- rightENCB 18
