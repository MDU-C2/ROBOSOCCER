# Suggested areas for hardware
This document contains initial suggestion for areas of focus for the robot. 
Please add more if you have any improvements or suggestions.

A starting point:
- Decide some initial components (not specific).
- Start with a simple block diagram of how we want the components to interact.

## Processing Unit
How should we process everything

Requirements:
- Needs to accept inputs from the sensors, motors, and communication.

Suggestion:
- STM32, ESP32. Depending on the autonomous decision making something more powerful could be necessary.
- If RTOS is necessary the board must support it.

## Communication 

We will need to implement a communication system.
The competiton uses wireless communcication and bluetooth is not allowed.

Suggestion:
- Wi-Fi
- RF modules

## Battery management/Voltage regulation

All electronics wheel need power, circutry for providing power to the different components will be necessary.

Suggestion:
- Li-Po, Li-Ion
- Protection circuitry, prevent overcharge 

## Motor control

What type of motors for the wheels and the roller/kicker.

Suggestion:
- Brushed DC motor, brushless motor.
- PID motor control can be useful
    - Will require some kind of feedback from the motors, e.g. the rpm

## Kicker/Roller

Suggestion from Micke was to use a solenoid and try to make the kicker be able to pass the ball on the ground and chip into the air.

Suggestion:
- Solenoid

## Wheels

Almost all other competitors use some form of swedish wheels. Should we use 3 or 4 wheels etc.

Pros and cons:
- 3 Wheels is lighter and would require less power
- 4 wheels make the robot stable

## Design of kicker, body, wheel house etc.

Initial 3D draft of the robot.

Suggestions:
- Try to keep it light and within the specs for the competition.
- The batteries, kicker, and motors should be easily accessible for replacement.

## Sensors

How are we going to detect the other robots

Suggestion:
- IR sensor
- Ultrasonic sensor
- IMU can be useful for autonomous decision-making

## Cooling

Maybe, idk

## Testing

A test environment will be necessary to ensure that the subsystems works as expected and that we can show that they actually work.

Requirements:
- A standardized way of calibrating the motor speed, solenoid power (if used).
- Some tools for real-time monitoring

