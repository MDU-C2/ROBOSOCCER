# Contributing to the SSL Robot Project

We’re excited that you’re interested in contributing to the SSL Robot Project! Whether you're working on hardware, software, documentation, or testing, your contributions help improve the project. Below are the guidelines to ensure that the contribution process is smooth and efficient.

## Table of Contents
- [Code of Conduct](#code-of-conduct)
- [How Can I Contribute?](#how-can-i-contribute)
  - [Reporting Bugs](#reporting-bugs)
  - [Suggesting Enhancements](#suggesting-enhancements)
  - [Contributing to Hardware](#contributing-to-hardware)
  - [Contributing to Software](#contributing-to-software)
  - [Improving Documentation](#improving-documentation)
- [Pull Request Guidelines](#pull-request-guidelines)
- [Style Guides](#style-guides)
  - [Git Commit Messages](#git-commit-messages)
  - [Code Style (Software)](#code-style-software)
  - [File Naming Conventions](#file-naming-conventions)
- [Setting Up the Project](#setting-up-the-project)

---

## Code of Conduct

Please note that this project follows a [Code of Conduct](link_to_code_of_conduct). By participating, you are expected to uphold this code. We encourage a friendly, respectful, and inclusive environment for all contributors.

---

## How Can I Contribute?

### Reporting Bugs

If you find a bug in the hardware design, circuits, or software:
1. **Search existing issues** to see if the bug has already been reported.
2. If not, **open a new issue**. Be sure to include:
   - A clear title
   - Steps to reproduce the issue
   - The environment details (OS, tool versions, robot components, etc.)
   - Screenshots or logs if applicable.

### Suggesting Enhancements

Enhancements to hardware design, new features for software, or improvements to documentation are welcome. If you have an idea:
1. **Search existing issues** to avoid duplicates.
2. If it hasn’t been discussed, **open a new issue** and explain:
   - The purpose of the enhancement
   - Potential benefits and challenges
   - Relevant files or sections impacted.

### Contributing to Hardware

To contribute to the hardware (3D models, circuits, BOM, etc.):
1. Fork the repository and clone your fork.
2. Work on a feature branch (e.g., `feature/motor-controller-update`).
3. Follow the existing structure:
   - Place 3D models in the `/3D_Models` directory.
   - Place circuit design files in `/Circuits`.
   - Ensure the **Bill of Materials (BOM)** is updated if you change components.
4. **Document your changes** in `README.md` files in relevant directories.
5. Submit a Pull Request following the [Pull Request Guidelines](#pull-request-guidelines).

### Contributing to Software

To contribute to the motor control or other software:
1. Fork the repository and clone your fork.
2. Work on a feature branch (e.g., `feature/software-motor-enhancement`).
3. Ensure you’re following the project's [Code Style Guidelines](#code-style-software).
4. Include unit or integration tests for your changes in the `/tests` folder.
5. Submit a Pull Request following the [Pull Request Guidelines](#pull-request-guidelines).

### Improving Documentation

Good documentation is critical for onboarding new contributors and maintaining clarity. To improve or update the documentation:
1. Fork the repository and clone your fork.
2. Work on a feature branch (e.g., `docs-component-info`).
3. Make sure to update or create relevant `README.md` files, wiki pages, or guides.
4. Submit a Pull Request with your changes.

**General Information**:

**Updating component information** see [Motor_Wiki](Hardware/Components/Motor_Wheel.md) for a template.
Make any changes necessary for your component. If a datasheet is available, upload it to [Design_Documents/Datasheets](../Design_Documents/Datasheets/), give it a readable name and link to it from your component wiki page.

---

## Pull Request Guidelines

Before submitting a pull request (PR):
1. Ensure that your changes adhere to the repository structure and file organization.
2. Ensure your changes are **thoroughly tested** (hardware simulations, firmware tests, etc.).
3. Include **clear commit messages** describing the changes (see [Git Commit Messages](#git-commit-messages)).
4. For software:
   - Ensure you’ve written or updated **tests**.
   - Run **linting** and **formatting checks** (e.g., with tools like `clang-format`).
5. For hardware:
   - Ensure you’ve updated all relevant **BOMs** and **documentation**.
6. If your PR relates to an issue, link it in the PR description (e.g., `Closes #23`).
7. Request a **review** from at least one other contributor.

---

## Style Guides

### Git Commit Messages

- Use the present tense ("Add feature" not "Added feature").
- Keep messages concise and clear.
- Structure:
  ```
  [Component] Brief description of change
  
  Optional extended description if needed.
  ```

**Examples:**
- `MotorControl: Improve PWM signal accuracy`
- `Docs: Update meeting notes for September 2024`

### Code Style (Software)

For motor control and other software, follow these coding standards:
1. **Indentation**: Use spaces instead of tabs (2 or 4 spaces depending on the language).
2. **Naming**:
   - Use `snake_case` for variable and function names (e.g., `calculate_speed`).
   - Use `CamelCase` for class names (e.g., `MotorController`).
3. **Comments**:
   - Provide clear comments for complex logic.
   - Use docstrings for functions to describe input/output.

If you're writing C or C++ firmware:
- Stick to **[Google C++](https://google.github.io/styleguide/cppguide.html)** standards for portability.
- Make sure to test with multiple compilers (e.g., `gcc`, `clang`).

### File Naming Conventions

Use descriptive and consistent names for files:
- Circuit files: `MotorControl_v1.kicad`, `Kicker_v2.sch`.
- 3D models: `WheelAssembly_v1.step`, `Chassis_v3.stl`.
- Software: `motor_control_firmware.c`, `pwm_control_module.h`.
- Documentation: `2024-09-21_Meeting.md`, `MotorControl_Specifications.pdf`.

---

## Setting Up the Project

### Prerequisites

- For hardware: You’ll need KiCad for circuit design, CAD software for 3D models, and access to the appropriate tools for each subsystem.
- For software: Ensure you have the necessary compilers, libraries, and development tools installed.

### Setting Up the Repository

1. **Clone the repository**:
   ```bash
   git clone https://github.com/yourusername/SSL-Robot-Project.git
   cd SSL-Robot-Project
   ```

2. **Set up the development environment** (instructions can vary based on the subproject):
   - For software:
     ```bash
     cd Software/Motor_Control
     # Follow README.md for building and running tests
     ```

3. **Simulate or run tests** for hardware or software to validate your changes.

---

Thank you for contributing! If you have any questions, feel free to reach out via issues or discussions.

---

