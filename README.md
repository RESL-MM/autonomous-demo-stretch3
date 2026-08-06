# Stretch3 Autonomous Microfabrication Demo

This repo consists of scripts and documentation needed for an autonomous mobile manipulator routine for use in a university microfabrication cleanroom environment.

Specifically, it contains code for the Hello Robot Stretch3 to autonomously perform a wafer loading and unloading sequence for an Oxford RIE80 Etcher in which the robot aligns with and operates the machine and navigates between predefined stations (wafer table and RIE80).

The demo combines pre-defined coarse-grained movement in a fixed environment and fine-grained error correction and adjustments using ArUco markers, a head-mounted D435 camera, and a gripper-mounted D405 camera for manipulation tasks.

## Project Status

The repo currently holds a minimum viable demo in the `autonomous_demo` directory for use in a fixed environment (The John O'Brien Nanofabrication Laboratory @ USC) and for operational testing/data collection.

## Project Documentation

- [Operation instructions](docs/autonomous_demo_instructions.md): safety, physical preparation, execution, interruption, and shutdown
- [Project overview](docs/project-report.md): project overview, specifics, considerations, and future work

## Demo Workflow

A complete demo is defined as a single iteration of the following wafer loading and unloading sequence:

(0. assume robot starts generally near and roughly perpendicular to the RIE80)
1. Align with and open the RIE80
2. Navigate to + align with Wafer Station and pick up wafer
3. Navigate to + align with machine tray and deposit wafer
4. Navigate to + align with machine and close the RIE80
5. Wait (simulate recipe being run and wafer being etched)*
6. Align with and open the RIE80
7. Navigate to + align with machine tray and withdraw wafer
8. Navigate to + align with Wafer Station and put down wafer
9. Navigate to + align with machine and close the RIE80

Iteration Complete!

> * recipe running and robot navigation to charging/docking station while waiting to be implemented

## Repository Structure

```text
autonomous-demo-stretch3/
├── README.md
├── autonomous_demo/
│   ├── main.py
│   ├── aruco_marker_info.yaml
│   ├── base_alignment.py
│   ├── station_navigation.py
│   ├── twist_and_adjust.py
│   ├── button_and_adjust.py
│   └── ...
├── docs/
│   ├── autonomous_demo_instructions.md
│   └──  project-report.md
└── misc/
    └── ...old, incomplete, or experimental scripts
```

## Main Components
| File | Responsibility |
| --- | --- |
| `main.py` | Coordinates the supporting scripts and carries out demo iterations |
| `base_alignment.py` | Aligns markers on the robot base with markers on the floor |
| `station_navigation.py` | Locates and approaches the tray or wafer station |
| `twist_and_adjust.py` | Visually aligns with and turns the machine dial |
| `button_and_adjust.py` | Visually aligns with and presses the machine button |
| `normalized_velocity_control.py` | Converts normalized servo commands into robot joint and base commands |
| `aruco_detector.py` | Detects ArUco markers and estimates their poses |
| `aruco_to_fingertips.py` | Converts finger-marker poses into estimated fingertip poses |
| `d405_helpers.py` | Configures and reads the gripper-mounted D405 camera |
| `d435_helpers.py` | Configures and reads the head-mounted D435 camera |
| `aruco_marker_info.yaml` | Defines marker dimensions and names |

## Hardware and Software

### Hardware
- Hello Robot Stretch3
- Intel RealSense D435 head camera (included with Stretch3)
- Intel RealSense D405 gripper camera (included with Stretch3)
- Custom Printed End-Effector
    - Dual purpose: allow for interaction with machine dial and button while holding the vacuum wand in place (such that it can be actuated by the Stretch3 gripper)
- ArUco markers placed at key locations for vision based error correction
    - e.g. on the sides of the machine dial for gripper servoing and the floor for perpendicular base alignment with the RIE80
- Oxford RIE80 Etcher

### Software
The demo heavily builds off and adapts code from the [Hello Robot Stretch3 repositories](https://github.com/hello-robot/stretch_body), and in particular the [visual servoing repo](https://github.com/hello-robot/stretch_visual_servoing)

## Running the Demo
Read the [operation instructions](docs/autonomous_demo_instructions.md)

After all setup and preparations have been completed, then run the following from the repo when SSH'd into or directly using the Stretch3:
```bash
cd autonomous_demo
python3 main.py
```