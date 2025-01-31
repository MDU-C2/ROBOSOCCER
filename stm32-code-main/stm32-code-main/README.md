About
-----------------------
This repository contains the executable code intended to run on the STM32H7 Nucleo-144 board.

### Built with
The stm32-code project is built with the following:

- [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.html)
- [MicroRos](https://micro.ros.org/)
- [Docker](https://www.docker.com/)

Getting started
-----------------------

### Prerequisites

For STM32CubeIDE installation see: https://www.st.com/en/development-tools/stm32cubeide.html

On Ubuntu and other Debian derivatives, Docker can be installed with:
```
sudo apt install docker
```

In order for microRos dependencies to be downloaded when building, Docker must be
configured to run with root privilige. This can be done with:
```
sudo groupadd docker
sudo usermod -aG docker $USER
```

Log in and out after completing this step in order for group membership to be evaluated.

### Installation
1. Clone the repository and initialize the microROS submodule:
```
git clone https://github.com/DVA490-474-Project-Course/stm32-code.git
cd stm32-code
git submodule update --init --recursive
```
2. Start STM32CubeIDE

3. Log in to your ST account in the IDE if you aren't already logged on.

4. Click 'File>Open Projects from file System...'

5. Click the 'Directory...' button and select the repository folder.

6. Make sure the project is selected in the checklist and click 'Finish'.

7. Generate code by clicking 'Project>Generate Code'. The project is now ready to build.

Usage
-----------------------

In order to upload the program, connect the PC to the USB port CN1  (the one located on
the side of the board which contains the smaller processor, which is the debugger) and
click Run>Run.

Building for the first time may take a few minutes since microROS is downloading its
dependencies to the project.

Roadmap
-----------------------

- [x] Develop Sensor Interface
- [ ] Develop Kicker Interface
- [ ] Develop API to communicate with STM32 on wheel motors
- [ ] Develop API to communicate with STM32 on dribbler
- [x] Implement MicroROS Publishers
- [ ] Implement MicroROS Subscribers

Design diagrams
-----------------------

Design diagrams/files can be found in the [docs](/docs) directory. Additionally
they are available on:
- [Hardware Interface](https://www.mermaidchart.com/raw/11c442f5-192c-4ac3-b61e-867a3e2ca6ea?theme=dark&version=v0.1&format=svg)

License
-----------------------

This repository contains source files from third parties which are distributed
under the license conditions of the respective distrubutors. Source files
developed by the authors of this repository are located in the Core/Src/user-code
directory and is distrubted under the MIT Liscence.

See [MIT Licence](/Core/Src/user-code/LICENSE) for the MIT license.
See [STM32IDE License](/license.pdf) for licence conditions pertaining files accossiated with STM32IDE.
See [VL6180X Driver License](/Drivers/Adafruit_VL6180X/license.txt) for licence conditions pertaining Adafruit's VL6180X driver.
See [VL53L4CD Driver License](/Drivers/VL53L4CD_ULD_Driver/license.pdf) for licence conditions pertaining ST's VL53L4CD driver.
See [WSEN_ISDS Driver License](/Drivers/WSEN_ISDS_2536030320001/license_terms_wsen_sdk.pdf) for licence conditions pertaining Würth Elektronik's WSEN_ISDS driver.

Contributors and contact
-----------------------
- Aaiza Aziz Khan: akn23018@student.mdu.se
- Mudar Ibrahim: mim20004@student.mdu.se
- Shruthi Puthiya Kunnon: spn23001@student.mdu.se
- Emil Åberg: eag24002@student.mdu.se