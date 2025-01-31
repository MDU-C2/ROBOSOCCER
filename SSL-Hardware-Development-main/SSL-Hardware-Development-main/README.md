# SSL-Hardware-Development

## Overview
Welcome to the SSL Robot Project repository! This project encompasses both hardware and software components essential for building and controlling an SSL robot. Our goal is to create a robust, scalable, and maintainable robot platform.

## Repository Structure
- **3D_Models**: Contains all 3D models related to the robot design.
- **Circuits**: Circuit designs for various robot components.
- **BOM**: Bill of Materials for each component.
- **Software**: Firmware and software for motor control and other functionalities.
- **Design_Documents**: Technical specifications and design decisions.
- **Meeting_Notes**: Documentation of project meetings.
- **Misc**: Miscellaneous files and references.
- **Simulations**: Simulation scripts and models.
- **Documentation**: Comprehensive project documentation and guides.

```bash
/SSL-Robot-Project
├── /3D_Models
│   ├── /CAD_Files            # All CAD files for different robot parts
│   ├── /STL_Files            # STL exports for 3D printing
│   └── /Renderings           # Renders of the 3D models
├── /Circuits
│   ├── /Motor_Control        # Motor circuit design files
│   ├── /Kicker               # Kicker circuit design files
│   ├── /Dribbler             # Dribbler circuit design files
│   └── /Sensor               # Sensor circuit design files
├── /BOM                      # Bill of Materials for different components
├── /Software
│   ├── /Motor_Control
│   │   ├── /Firmware          # Firmware code for motor controllers
│   │   │   ├── /src           # Source code files
│   │   │   ├── /include       # Header files
│   │   │   ├── /tests         # Unit and integration tests
│   │   │   └── README.md      # Firmware-specific documentation
│   │   ├── /Software_Doc      # Documentation related to motor control software
│   │   │   ├── /API_Docs      # API documentation if applicable
│   │   │   └── /User_Guides   # User manuals or setup guides
│   │   ├── /Libraries         # Any libraries or dependencies specific to motor control
│   │   └── /Examples          # Example projects or usage scenarios
│   ├── /Kicker_Control        # Similar structure for kicker control software
│   ├── /Dribbler_Control      # Similar structure for dribbler control software
│   └── /Sensor_Control        # Similar structure for sensor control software
├── /Design_Documents          # Technical specifications, design decisions
├── /Meeting_Notes             # Notes from each project meeting
├── /Misc                      # Any other miscellaneous files
├── /Simulations               # Simulation files or scripts if applicable
├── /Documentation             # General documentation for the project
│   ├── README.md              # Overview of the project
│   ├── CONTRIBUTING.md        # Guidelines for contributing
│   ├── LICENSE.md             # Licensing information
│   └── /Wiki_Links            # Links to relevant wiki pages
└── .gitignore                 # Git ignore file
```

## Getting Started

### Prerequisites
- **Hardware**: Access to the necessary hardware components as listed in the BOM.
- **Software**: Install required software dependencies (refer to [Dependencies](Software/Software_Dependencies.md) documentation).

### Setup Instructions
1. **Clone the Repository**
    ```bash
    git clone git@github.com:DVA490-474-Project-Course/SSL-Hardware-Development.git
    cd SSL-Robot-Project
    ```

2. **Setup Software Environment**
    - Navigate to the software directory:
        ```bash
        cd Software/Motor_Control
        ```
    - Follow the instructions in `README.md` to build and deploy the firmware.

3. **Accessing Documentation**
    - Visit the [Wiki](https://github.com/DVA490-474-Project-Course/SSL-Hardware-Development/wiki) for detailed guides and API documentation.

## Contributing
We welcome contributions! Please read our [CONTRIBUTING.md](Documentation/CONTRIBUTING.md) for guidelines on how to get involved.

## License
This project is licensed under the [MIT License](Documentation/LICENSE).

## Contact
For any questions or support, please open an issue or contact the project maintainers.

Pontus Svensson - psn19003@student.mdu.se
