# RoboArm

A full‑stack **motion‑planning and control platform** for the **RoboArmM3**.  
The core algorithm is a C++ implementation of **Rapidly‑exploring Random Tree Star (RRT\*)**, integrated with forward‑kinematics‑based collision checking and a UART command pipe to the arm.

*️⃣  **Primary deployment target:** ***BeagleBone Black*** (ARM Cortex‑A8) running Debian 11.
*️⃣  **Backup demo:** Raspberry Pi Pico W submodule build of the same planner (wireless HTTP client)

---

## ✨ Key Features

* **2‑D & 3‑D RRT** planners with obstacle avoidance, path re‑wiring, and output CSV logging results
* **Forward kinematics** * **Forward kinematics** tailored to the RoboArm M3 geometry: baseline float + experimental **fixed‑point** (DAISY‑generated) variant  
* Python utilities for plotting trajectories and analysing planner performance.
* **BeagleBone workflow** – build with `make rrt_3d_ARM`, then auto‑stream the path to the arm over UART (JSON @ 115200 baud)  
* **Pico W demo** (Git submodule `pico_armTest`) that runs the same planner over Wi‑Fi.
* **FPGA UART testing** (Verilog) echo/RX/TX cores + C host app, groundwork for future hardware acceleration

---

## 🗂️ Repository Layout

```bash
.
├── README.md                  
├── documents/                  # Reports & demo video
│   ├── EC535_Final_Report.pdf
│   └── Gautel_Gonzalez_EC535_Final_Video.mp4
│
├── pico_armTest/               # Backup wireless demo on Raspberry Pi Pico W
│   └── …   
│
└── src/                        # MAIN PROJECT SOURCE
    ├── rrt_base.*              # Common base rrt logic agnostic to dimension and DOF
    │
    ├── 2d_impl/                # 2‑D planner + plotting scripts
    │   ├── Makefile
    │   ├── rrt_2d.cpp/h
    │   └── results/*.csv
    │
    ├── 3d_impl/                # 3‑D planner + plotting scripts, FK lib & BeagleBone helpers
    │   ├── main.cpp            # Local entry
    │   ├── BB_main.cpp         # BeagleBone‑specific entry (rrt_3d_ARM)
    │   ├── forward_kinematics/
    │   ├── process_commands.py # Streams path over UART to RoboArmM3
    │   └── results/*.csv
    │
    └── UART_FPGA_Test/         # Verilog UART cores + C test harness
        ├── Verilog Files/
        └── data_test.c
````

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

# Generate a path + auto‑streams over UART
./rrt_3d_ARM 

# (only stream found path to goal)
python3 process_commands.py --device /dev/ttyUSB0
```

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

1. Open `src/UART_FPGA_Test/Verilog Files` in Vivado 2023.1
2. Apply `Basys_3_Constraints_UART.xdc`
3. Generate bitstream & program board
4. Wire Beaglebone UART pins to Basys3 physcially 
5. Host side: complile data_test.c and run `./data_test` to send and recieve data from FPGA

---

## 📦 Dependencies

| Component            | Requirement                                               |
| -------------------- | --------------------------------------------------------- |
| **BeagleBone build** | GCC 10 + , `make`, gprof (optional profiling)             |                                           |
| **Python tools**     | Python ≥ 3.9, `numpy`, `pandas`, `pyserial`, `matplotlib` |
| **Pico firmware**    | [Pico SDK](https://github.com/raspberrypi/pico-sdk)       |
| **FPGA tests**       | Xilinx Vivado 2023.1, Basys 3 board                       |

All scripts assume Debian‑based Linux; Windows users can build inside WSL 2.

---

## 📖 Further Reading & Demo

* **Project Report:** [`documents/EC535_Final_Report.pdf`](documents/EC535_Final_Report.pdf)
  – design goals, methodology, profiling results, fixed‑point study, and future work
* **Demo Video:** [`documents/Gautel_Gonzalez_EC535_Final_Video.mp4`](documents/Gautel_Gonzalez_EC535_Final_Video.mp4)

Developed as the final project for **EC535— Embedded Systems** at Boston University (Spring 2025)

