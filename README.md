# RoboArm

A full‑stack **motion‑planning and control platform** for the **RoboArmM3**.
The core algorithm is a C++ implementation of **Rapidly‑exploring Random Trees Star (RRT*)**.

*️⃣  **Primary deployment target:** ***BeagleBone Black*** (ARM Cortex‑A8) running Debian 11.

*️⃣  **Backup demo:** Raspberry Pi Pico W submodule for quick wireless tests.

---

## ✨ Key Features

* **2‑D & 3‑D RRT planners** with collision checking, path re‑wiring and output CSV logging results.
* **Forward kinematics** tailored to the RoboArm M3 geometry (Provided by Alp).
* Python utilities for plotting trajectories and analysing planner performance.
* **BeagleBone workflow** – build with `make rrt_3d_ARM`, then auto‑stream the computed path to the arm over UART using a Python helper.
* **Pico W demo** (Git submodule `pico_armTest`) that runs the same planner over Wi‑Fi.
* **FPGA UART testing** (Verilog) to validate possible future direction to optimize planner.

---

## 🗂️ Repository Layout

```bash
.
├── UART_Test/                  #  UART communication tests for Future direction
│   ├── BB_Arm_tests/           #  C++ serial communication test app & Makefile
│   ├── Verilog Files/          #  uart_rx/tx/echo modules for FPGA
│   └── uart_echo_test.c        #  C tests connection with FPGA
│
├── pico_armTest/ (submodule)   # Backup wireless demo on Raspberry Pi Pico W
│   └── …                       
│
├── src/                        #  MAIN PROJECT SRC FILES
│   ├── rrt_base.*              # Common base rrt logic agnostic to dimension and DOF 
│   ├── 2d_impl/                #  2‑D planner + plotting scripts
│   └── 3d_impl/                #  3‑D planner + plotting scripts, FK library & BeagleBone helpers
│       ├── process_commands.py #   Streams path over UART to RoboArmM3
│       └── …
│
├── EC535_Final_Report.docx     # Project report
├── Graphics.pptx               # Slides containing result plots
└── Work Breakdown Structure.docx # Initial plans
```

> **Submodule notice** – clone with `--recurse-submodules` to fetch `pico_armTest`.

---

## 🚀 Quick Start

### 1 – Clone the repo

```bash
git clone --recurse-submodules https://github.com/NaomiGonz/RoboArm.git
cd RoboArm
```

### 2 – Compile & run on **BeagleBone Black**

```bash
sudo apt update && sudo apt install build-essential python3-pip -y
cd src/3d_impl
make rrt_3d_ARM          # Produces ./rrt_3d_ARM

# Generate a path 
./rrt_3d_ARM 

# Stream the path to the RoboArmM3 controller over UART (will happen automatically if you run ./rrt_3d_ARM)
python3 process_commands.py --device /dev/ttyUSB0 
```

`process_commands.py` formats each waypoint into the RoboArmM3 command protocol and writes it at 115200 baud.

### 3 – Desktop simulation + Graph viewer

For 2D implementation:
```bash
cd src/2d_impl
make && ./rrt_2d
python3 plot_results.py results/<name>.csv
```

For 3D implementation:
```bash
cd src/3d_impl
make && ./rrt_3d
python3 plot_results.py results/<name>.csv
```

### 4 – Raspberry Pi Pico W backup demo

```bash
cd pico_armTest
mkdir build && cd build
cmake .. && make
picotool load picow_http_client.uf2   # flash the board
```

The firmware connects and sends commands to RoboArmM3 through Wi‑Fi.

### 5 – FPGA UART tests (Basys3)

```text
  1. Open UART_Test/Verilog Files in Vivado 2023.1
  2. Apply Basys_3_Constraints_UART.xdc
  3. Generate bitstream & program board
  4. Host side: make and run uart_echo_test.c
```

---

## 📦 Dependencies

| Component            | Requirement                                               |
| -------------------- | --------------------------------------------------------- |
| **BeagleBone build** | GCC 10 + , Make                                           |
| **Python tools**     | Python ≥ 3.9, `numpy`, `pandas`, `pyserial`, `matplotlib` |
| **Pico firmware**    | [Pico SDK](https://github.com/raspberrypi/pico-sdk)       |
| **FPGA tests**       | Xilinx Vivado 2023.1, Basys 3 board                       |

All scripts assume Debian‑based Linux; Windows users can build inside WSL 2.

---

## 📝 Background

Developed as the final project for **EC535— Embedded Systems** at Boston University (Spring 2025)

