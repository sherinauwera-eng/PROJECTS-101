# Autonomous Vehicle: Systems Engineering Project


---

##  Table of Contents
1. [Project Overview](#project-overview)
2. [Repository Architecture](#repository-architecture)
3. [Systems Engineering & SysML Models](#sysml-models--architecture)
4. [Hardware & Software Implementation](#hardware--software-implementation)
5. [Mechanical Design & 3D Parts](#mechanical-design--3d-parts)
6. [Team Responsibilities](#team-responsibilities)
7. [Project Resources](#project-resources)
   
---

##  Project Overview
This repository documents the complete development lifecycle of an autonomous line-following vehicle prototype using a **Model-Based Systems Engineering (MBSE)** methodology. The project transitions seamlessly from high-level logical system requirements and SysML definitions to a validated physical prototype, ensuring full traceability between architectural design, simulation, and real-time C++ hardware execution.

---

##  Repository Architecture
Our repository is structured chronologically to reflect the iterative MBSE design process, preserving historical team contributions while highlighting final production deliverables:

* `/00_Project_Management/`: Contains official course lectures, reference documentation, and component datasheets.
* `/SysML_Diagrams/`: Centralized folder housing all formal system models (Requirements, Use Case, Block Definition, Internal Block, Activity, Sequence, and Package diagrams).
* `/3D_Parts/`: Contains custom 3D printing files (`.STL`, `.STEP`, `.SLDPRT`) for motor mounts, sensor brackets, and structural elements.
* `/Tinkercad_Simulation/`: Contains virtual circuit schematics, board layouts (`.brd`), and early simulation test scripts.
* `/src/`: Houses finalized, production-ready C++ firmware files (`.ino`) for autonomous driving, obstacle avoidance, and line tracking.
* `/Task_1_Initial_Model/`: Preserves baseline logical requirements and initial behavioral models.
* `/Task_2_Prototype/`: Contains intermediate hardware-specific constraints and physical design iterations.

---

##  SysML Models & Architecture
Our models adhere strictly to standard systems engineering criteria to guarantee complete functional verification:
* **Requirements Model:** Establishes parent-child hierarchical tracking from high-level system goals (`REQ-SYS-001`) down to specific functional behaviors (`REQ-FUN-002` through `004`).
* **Use Case Diagram:** Maps system boundaries, actors (`Operator`, `Environment / Track`), and core functional use cases (line tracking, obstacle detection, V-dogleg detour, and 180° black zone maneuvers).
* **Structural Models (BDD & IBD):** Defines system composition and internal power/data flows—explicitly separating 9V power lines, 5V regulation rails, PWM motor controls, and digital sensor inputs.
* **Behavioral Models:** Includes detailed activity workflows, dynamic sequence diagrams, and a robust state-machine logic tracking track recovery states.

---

##  Hardware & Software Implementation
* **Microcontroller:** Arduino Uno (R4 compatible).
* **Sensing Subsystem:** Dual TCRT5000 infrared reflectance sensors for high-contrast line tracking; dual HC-SR04 ultrasonic sensors for active proximity scanning (<12cm safety threshold).
* **Actuation Subsystem:** L293D dual H-bridge motor driver managing DC gear motors via PWM modulation.
* **Engineering Challenges & Code Iteration:** 
  * *Actuator Asymmetry:* Physical variance in our DC motors caused mechanical drift. We implemented software-level motor balancing/boosting to ensure straight-line tracking.
  * *Control Transition:* Persistent oscillations during initial PID tuning led to a strategic architecture shift. We transitioned to a robust, deterministic **discrete state-machine framework** (`src/`) that reliably handles high-speed line tracking, S-curve verification, V-dogleg obstacle detours, and 180° black-zone reversals.

---

##  Mechanical Design & 3D Parts
The physical platform features a rigid chassis integrated with custom-engineered 3D-printed components found in `/3D_Parts/`:
* **Motor Brackets:** Custom mounts securing DC gear motors to maintain wheel alignment.
* **Sensor Housings:** Precision fixtures securing the TCRT5000 IR array and HC-SR04 ultrasonic rangefinders.

---



---

##  Testing & Demonstration Media

### 1. Hardware Circuit & Wiring Schematic
<img src="https://github.com/user-attachments/assets/b8a4ca0c-efc1-4664-bd09-5e6ee7667e7f" width="450" alt="Tinkercad Circuit Setup">

> **Description:** Complete wiring schematic mapped in Tinkercad, detailing the electrical integration between the 9V power supply, Arduino Uno microcontroller, L293D motor driver, TCRT5000 infrared sensors, and dual HC-SR04 ultrasonic rangefinders.
* **Simulation:** [View Tinkercad Circuit Simulation](https://www.tinkercad.com/things/h7Yk70u5KPw-d2d5prototyping)

---

### 2. Assembled Physical Prototype
<img src="https://github.com/user-attachments/assets/94e52b9a-75ff-4f06-86a9-ffe463c1b45a" width="450" alt="Physical Robot Assembly">

> **Description:** Top-down view of the physical autonomous vehicle chassis. Highlights the modular layout featuring the laser-cut platform, custom 3D-printed motor and sensor brackets, the central breadboard circuit, and the 9V power pack.

---

### 3. Track Navigation & Line Following Test
<video src="https://github.com/user-attachments/assets/6520a15d-1d30-4454-bbc8-dd526229e5ad" width="450" controls></video>

> **Description:** Demonstration of autonomous line tracking on the official test track. The vehicle processes real-time digital feedback from the TCRT5000 IR sensor array, dynamically applying PWM speed adjustments to maintain track alignment through complex curves.

---

### 4. Obstacle Detection & Avoidance Maneuver
<video src="https://github.com/user-attachments/assets/74f175d0-a834-4d56-8ff4-bd475340f890" width="450" controls></video>

> **Description:** Execution of the V-dogleg detour strategy. When the front-mounted HC-SR04 ultrasonic sensors detect an obstacle within the <12cm safety threshold, the system interrupts line-following behavior, halts momentum, executes a 45-degree pivot detour, bypasses the obstacle, and successfully reacquires the line.

---
*Project managed via GitHub Issues for full transparency and version control.*
