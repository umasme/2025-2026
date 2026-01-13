
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
* ** Add-ons:**
  * Onboard DC-DC buck converter based on AP64502Q synchronous DC-DC buck converter (5V/5A). 
  * Linear 3.3 V regulator (not required for current system architecture).

## PCB design considerations: 
* Followed the IPC-2221-compliant power plane optimization for high current power copper pours.
* Follwed the recommeded layout from the data sheets of DC DC converter. You can find through this link [https://www.diodes.com/datasheet/download/AP64502Q.pdf].
* Used the Power Analayzer Tool in Altium for optimizing the design for high current requirement removeing all bottlenecks as you see here

  

## Design Objectives
* **Reliability:** Consistent operation under high-current and transient conditions.
* **Efficiency:** Low-resistance power paths to minimize voltage drop.
* **Usability:** Clear visual diagnostics for fast fault identification.

## Safety and Protection
* **Input:** Reverse-polarity protection.
* **Output:** Fuse protection per channel.
* **Visuals:** Fault detection via LED indicators.
* **Noise:** Electrical noise suppression using decoupling capacitors.
