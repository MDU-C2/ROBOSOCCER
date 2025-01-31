# Week 39

### Powertrain & Electronics

- Create a schematic for the whole system (could be made in KiCad, as a blockdiagram). Try to include as much information as possible, i.e. power estimtes, voltage estimates etc.
    - Check so that everything is compatible and if we are required to do some changes.

### Sensor & Embedded system

- Decide on which sensors we want to use if that is not done already.
    - Create initial sensor schematics, ideally it would be done using KiCad but blockdiagram is okay, try to include as much information as possible e.g. voltages and power consumption.
- Research ways to sample the data from these sensors. Initial idea is to use FreeRTOS + micro-ROS on the ESP32.
    - There is no need to program for this task, we want to find out information about efficient/easy ways this could be done.

### CAD

- If the omni-wheels design is done we could print a prototype to see how it looks.
- When we have decided our main components, such as batteries, sensors, motors. Try to modify the base model and fit these components.
- If we have time start looking at the dribbler mechanism, i.e. how to mount the motor, we would need a base design.

### Kicker

- Create a schematic for the kicker, ideally this would be a circuit but can also be a quick sketch using blockdiagrams. Try to include as much details as possible, e.g. current draw, voltage requirements. 
    - A description of how it works and how we intend to use it. E.g. we will need to charge this capacitor and then describe the trigger mechanism.

