# Electronic Speed Controller

## Description
The Electronic Speed Controller (ESC) is necessary to control the BLDC motors used for the wheels

## Requirements
1. **Bidirectional Control**
   - The motors need to run both forwards and backwards. ESC must be bidirectional

2. **Customization**
   - Ability to configure acceleration curves and braking is necessary

3. **Communication Interface**
   - The ESCs should accept control inputs from an [NUCLEO-H723ZG](https://github.com/DVA490-474-Project-Course/SSL-Hardware-Development/wiki/Microcontroller) (e.g. PWM, UART, CAN)
   - The ESC should provide feedback which can be used to improve the control precision. (e.g. EMF)

4. **Motor Compatibility**:  
   - Compatible with brushless motors ([DF45L024048-A](https://github.com/DVA490-474-Project-Course/SSL-Hardware-Development/wiki/Motor_Wheel)).
   - Voltage and current ratings meet or exceed motor specifications.

5. **Physical Size and Weight**:  
   - Fits within the physical constraints of your robot's design.
   - Consideration of overall weight to maintain performance.

6. **Power Management**:  
   - Integrated Battery Eliminator Circuit (BEC), needed to power electronics.
   - Efficient power usage to maximize battery life.

## **[B-G431B-ESC1](https://www.st.com/en/evaluation-tools/b-g431b-esc1.html)**

| **Specification**              | **Value**                          |
|---------------------------------|------------------------------------|
| **Microcontroller**            | STM32G431CB (32-bit ARM Cortex-M4)|
| **Voltage Range**              | 8 V to 28 V                     |
| **Continuous Current**         | Up to 15 A                        |
| **Peak Current**               | 40 A                              |
| **Motor Type**                 | Brushless DC (BLDC)               |
| **Control Algorithm**          | Field-Oriented Control (FOC) & 6 step commutation      |
| **PWM Frequency**              | Configurable, up to 100 kHz        |
| **Communication Interfaces**   | UART, CAN, PWM|
| **Sensor Compatibility**       | Hall sensors, Encoder, Sensorless |
| **Dimensions**                 | 30 mm x 41 mm                     |
| **Weight**                     | Approx. 9.2 g                      |
| **Programming Options**        | ST-Link Debugger, STM32CubeIDE    |
| **Protection Features**        | Overcurrent, Overvoltage, Overtemperature |

