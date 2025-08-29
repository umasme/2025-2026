# 🚀 NASA Lunabotics 2025 — University of Miami ASME

Robust, ROS 2–based software stack and supporting assets for our lunar excavation rover.  
This repository houses **software, hardware, simulation, datasets, and documentation** to support the NASA Lunabotics 2025 competition.  

---

## 📖 Project Overview

Our rover integrates:

- **Perception** — stereo depth, object detection, terrain mapping  
- **Localization & Navigation** — SLAM, sensor fusion, path planning  
- **Autonomy** — state machines / behavior trees for excavation & hauling  
- **Control & Teleop** — motor drivers, teleop interfaces, web dashboard  
- **Simulation** — Ignition Gazebo worlds, scenarios, and URDF models  
- **Documentation** — aligned with NASA Systems Engineering (SRR, PDR, CDR, ConOps)  

---

## 📂 Repository Layout

```text
lunabotics-2025/
├─ README.md                     # Project overview (this file)
├─ LICENSE
├─ .github/workflows/            # CI/CD pipelines
├─ .devcontainer/                # VS Code Dev Containers
├─ docker/                       # Development & runtime Dockerfiles
├─ docs/                         # Documentation
│  ├─ index.md
│  ├─ ConOps/                    # SRR/PDR/CDR deliverables
│  ├─ Architectures/             # Block/state diagrams
│  ├─ MissionRules/              # Ops rules, FMEA links
│  ├─ Tutorials/                 # Step-by-step guides
│  └─ Calibration/               # Sensor/actuator calibration
├─ hardware/                     # Mechanical + electrical
│  ├─ electrical/                # Schematics, PCB, BOM
│  ├─ mechanical/                # CAD models, drawings
│  └─ configs/                   # Wiring tables, pin maps
├─ datasets/                     # Large files (via Git LFS)
│  ├─ bags/                      # ROS bag recordings
│  ├─ maps/                      # Occupancy/elevation maps
│  └─ test_scenes/               # Images, point clouds
├─ sim/                          # Simulation assets
│  ├─ worlds/                    # Ignition/Gazebo worlds
│  ├─ models/                    # URDF meshes, STL/DAE
│  └─ scenarios/                 # Scenario configs
├─ examples/                     # Cross-package & full-system demos
│  ├─ sim_demo.launch.py
│  ├─ teleop_example.md
│  └─ rviz_configs/
├─ tools/                        # Utility scripts
│  ├─ log_tools/                 # Bag→CSV, plotting
│  ├─ calibration/               # Camera/IMU calibration
│  ├─ deploy/                    # Deployment helpers
│  └─ ci/                        # CI support scripts
└─ ros2_ws/                      # ROS 2 workspace
   ├─ src/
   │  ├─ lunabotics_bringup/     # System launch & params
   │  ├─ lunabotics_description/ # URDF/Xacro and meshes
   │  ├─ lunabotics_msgs/        # Custom messages & services
   │  ├─ lunabotics_hw/          # Hardware drivers
   │  ├─ lunabotics_perception/  # Cameras, detection
   │  ├─ lunabotics_mapping/     # SLAM & maps
   │  ├─ lunabotics_localization/# State estimation
   │  ├─ lunabotics_navigation/  # Planning & control
   │  ├─ lunabotics_control/     # Low-level control loops
   │  ├─ lunabotics_mission/     # Autonomy FSM/BT
   │  ├─ lunabotics_teleop/      # Joystick / keyboard teleop
   │  ├─ lunabotics_dashboard/   # Web UI & ROS bridge
   │  └─ lunabotics_sim/         # Simulation plugins & launches
   ├─ build/                     # (ignored) colcon build output
   ├─ install/                   # (ignored) colcon install
   └─ log/                       # (ignored) colcon logs
