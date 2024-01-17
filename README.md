[![Build Robot Code](https://github.com/frc2053/Robot2024/actions/workflows/build.yml/badge.svg)](https://github.com/frc2053/Robot2024/actions/workflows/build.yml)
[![Formatting](https://github.com/frc2053/Robot2024/actions/workflows/format.yml/badge.svg)](https://github.com/frc2053/Robot2024/actions/workflows/format.yml)
[![Sanitizers](https://github.com/frc2053/Robot2024/actions/workflows/sanitizers.yml/badge.svg)](https://github.com/frc2053/Robot2024/actions/workflows/sanitizers.yml)

# FRC 2053 Southern Tier Robotics 2024 Robot Code

This repo holds our 2024 Robot. Our test drivebase is a swerve drivetrain using V3 Falcon 500's with the MK4i modules geared at the "L2 + 16t pinion" option. We have a pigeon v2 as our IMU as well all running through the CANivore. Our library uses Phoenix V6 Pro.

# How to setup

## Prerequisites (Install this stuff first)
- Python 3 (not installed from Windows store. Please install from Python website)
- Visual Studio 2022 with C++ workload
- Version [2024.1.1](https://github.com/wpilibsuite/allwpilib/releases/tag/v2024.1.1) of the WPILib installer installed.
- Choreo [v2024.1.0](https://github.com/SleipnirGroup/Choreo/releases/tag/v2024.1.0)
- Pathplanner [v2024.1.1](https://github.com/mjansen4857/pathplanner/releases/tag/v2024.1.1)
- Elastic Dashboard [v2024.0.2](https://github.com/Gold872/elastic-dashboard/releases/tag/v2024.0.2)

## Install steps: 
- Create your virtual environment
    Inside the root of the project run:
    `python -m venv ./venv`
- Activate your virtual environment
    `./venv/Scripts/Activate.ps1`
- Install the code formatter
    `pip install -r requirements.txt`
- Install the pre-commit hooks for formatting
    `pre-commit install`

## Build steps:
- To build the code, press Ctrl+Shift+P and search for "Build Robot Code". This will build the robot code for the robot, as well as simulation (desktop).

## To run:
- To run in simulation, press Ctrl+Shift+P and search for "Simulate Robot Code". This will launch the simulation.
- To run the code on the robot, press Ctrl+Shift+P and search for "Deploy Robot Code". This will search for a RoboRio over USB or the network and upload the code to the robot.