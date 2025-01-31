
- Linear solenoid? 
- Capacitors for power supply
- Controll the force of the kicker (use PWM to controll the capacitors discharge time)
- Some current limiting circuit

## Requirements
- Kicker must generate enough force to kick the ball at high speed
- The capacitor must be large enough to store energy for kicks but small enough to charge quickly
- Solenoid must be durable and capable to withstand frequent, rapid actuation without overheating or lose strength
- The kicker must respond quickly to control signals.
- Minimize energy loss during charging and discharging. 

## Limitations
- Heat management
- Material selection 
- Current rating
- Mechanical wear


## Other teams kicker 

| Team            | Solenoid                            | Kicker charge   | Kick speed |     |
| --------------- | ----------------------------------- | --------------- | ---------- | --- |
| The A-Team      | S-15-75-H (Magnetic Sensor Systems) |                 |            |     |
| ZJUNIict        |                                     | 4400uF 200V     |            |     |
| ER-Force        |                                     |                 |            |     |
| Ri-one          |                                     |                 |            |     |
| UBC Thunderbots |                                     |                 |            |     |
| ITAndroids      |                                     |                 |            |     |
| RoboTeam Twente | Flat solenoid (designen themself)   | 1000mikroF 200V |            |     |
| RoboFEI         |                                     | 1000mikroF 200V |            |     |
| Tigers Mannheim |                                     | 3600mikroF 240V | 8.5 m/s    |     |
| Robot jacket    |                                     | 4000mikroF 250V | 6 m/s      |     |
| KgpKubs 2017    |  Their own design with description  |                 |            |     |
| RFC Cambridge 2016| SOTUH032051.                        |.                |.           |.    |

  ## Calculations
  - The mass of the golf ball: 0.046kg
  - v=6.5m/s
  - stroke length 0.02m
  - 100V
    
  ## Kinetic Energy: 
  $$E_k = \frac{1}{2} m v^2 $$
  ## Work:
  $$W = F \cdot d = E_k$$
  ### Required force: 
  $$ F = \frac{E_k}{d} = 83.1 N $$
  ## Power Required: 
  $$ P=V \cdot I $$

  ## Capacitor Calculations: 
  - $$E = \frac{1}{2} 0.046 \cdot 6.5^2 = 0.972 J$$
  - $$E = \frac{1}{2} C V^2 J$$
  - $$C = \frac{2 \cdot E}{V^2} = 190.2uF $$

