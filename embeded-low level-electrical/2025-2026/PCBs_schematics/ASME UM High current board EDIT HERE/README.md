
# High-Current Power Distribution Board (PDB)

## Overview
The High-Current Power Distribution Board (PDB) is designed to safely and efficiently distribute electrical power to all high-power subsystems of the rover. It serves as the central power hub, handling high transient currents while providing protection, diagnostics, and future expandability.

## Supported Loads
This board distributes power to the following subsystems:

* **4×** Drivetrain motors (BLDC)
* **1×** Excavation motor (BLDC)
* **1×** Deposition motor (DC)
* **2×** Linear actuators 

> **Note:** The Jetson Nano is powered through an external power supply with a dedicated DC-DC converter and is **not** powered directly from this board.

## Electrical Capabilities
* **Peak Current:** Supports up to **40 A** peak current during startup and transient conditions.
* **Output Configuration:**
  * Minimum of **7** high-current outputs.
  * **1** dedicated output for a DC-DC converter.

## Board Features
* **Protection:** Reverse-polarity protection on the main power input and individual fuses for each output bus. (To be added)
* **Diagnostics:**
  * Power indication LED for board power status.
  * Individual LED indicators for each output to identify blown or faulted fuses.
* **Stability:** Decoupling capacitors for voltage stability and noise reduction.
* **Mechanical:** Defined mounting holes for secure mechanical integration.
* **Current Sensing:**
  * Initial revision includes main bus current sensor (or limited sensors).
  * PCB design preserves expandability for future per-output current sensing.
* **Optional Add-ons:**
  * Onboard DC-DC buck converter.
  * Linear 3.3 V regulator (not required for current system architecture).

## Design Objectives
* **Reliability:** Consistent operation under high-current and transient conditions.
* **Efficiency:** Low-resistance power paths to minimize voltage drop.
* **Usability:** Clear visual diagnostics for fast fault identification.
* **Scalability:** Modular and expandable architecture.
* **Durability:** Robust design suitable for mobile robotic platforms.

## Safety and Protection
* **Input:** Reverse-polarity protection.
* **Output:** Fuse protection per channel.
* **Visuals:** Fault detection via LED indicators.
* **Noise:** Electrical noise suppression using decoupling capacitors.
