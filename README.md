# Distributed Real-Time Motor Control System (STM32)

**Platform:** STM32 Nucleo-L476RG | **Framework:** CMSIS-RTOS (Keil RTX) | **Language:** Embedded C

## 📖 Project Overview

This project implements a robust, closed-loop speed control system for a DC motor, evolved through three stages of architectural complexity: from a bare-metal loop to a multi-threaded RTOS application, and finally to a distributed client-server system communicating over TCP/IP.

The core objective was to maintain precise motor velocity (RPM) using a custom **Integer-only Proportional-Integral (PI) controller**, strictly avoiding floating-point arithmetic to optimize for low-power embedded constraints.

### Key Capabilities

* **Real-Time Performance:** Deterministic task scheduling using **CMSIS-RTOS (RTX)**.
* **Distributed Architecture:** Split control logic where a **Client** handles physical I/O and a **Server** computes control signals over Ethernet (W5500).
* **Safety-Critical Design:** Implements fail-safe mechanisms to stop the motor immediately upon network disconnection or power loss.
* **Fixed-Point Arithmetic:** All control math is performed using optimized integer algorithms to avoid FPU overhead.

---

## 🛠 Hardware & Software Stack

### Hardware

* **Microcontroller:** STM32L476RG (ARM Cortex-M4) on Nucleo-64 board.
* **Motor Driver:** Infineon BTN8982 DC Motor Control Shield (Dual Half-Bridge).
* **Actuator:** Brushed DC Motor with Quadrature Encoder.
* **Networking:** WIZnet W5500 Ethernet Shield (SPI interface).

### 🔌 Pinout & Wiring Map
Defined in `peripherals.c`, the mapping for the **Infineon BTN8982 Shield** is:

| Function | MCU Pin | Arduino Header | Timer/Channel |
| :--- | :--- | :--- | :--- |
| **Bridge 1 Enable** | PA6 | D12 | GPIO |
| **Bridge 2 Enable** | PA5 | D13 | GPIO |
| **PWM Input 1** | PB4 | D5 | TIM3_CH1 |
| **PWM Input 2** | PA7 | D11 | TIM3_CH2 |
| **Encoder A** | PA8 | D7 | TIM1_CH1 |
| **Encoder B** | PA9 | D8 | TIM1_CH2 |

### Software Tools

* **IDE:** Keil µVision (MDK-ARM).
* **Config:** STM32CubeMX for low-level peripheral initialization.
* **OS:** Keil RTX (CMSIS-RTOS implementation).
* **Drivers:** STM32 HAL (Init), **Direct Register Access (Runtime)**, & WIZnet ioLibrary.

---

## 🏗 System Architecture

The software is designed with strict modularity, separating hardware abstraction, application logic, and control algorithms. The project features three distinct build targets:

### 1. The Controller (Core Logic)

A custom PI controller implemented in `controller.c`.

* **Algorithm:** Discrete-time PI control with manual tuning.
* **Anti-Windup:** Implements a **30-bit clamping mechanism** to prevent integrator overflow.
* **Math:** Uses fixed-point integer math for scaling duty cycles (0–100%) to PWM resolution (11-bit, 0–2047).

### 2. The Real-Time OS (Task Management)

Utilizes **CMSIS-RTOS** to manage concurrent threads. 

* **Signal-Based Timing:** Unlike standard `osDelay()` loops which can drift, this project uses **Virtual Timers (`osTimer`)** to trigger synchronization signals (`osSignalSet`). Threads block indefinitely (`osSignalWait`) until the timer fires, ensuring precise, drift-free sampling periods (e.g., exactly 10ms for the control loop).

Priorities differ by build target to optimize for specific roles:

**Target 1: Monolithic (Standalone)**
* **Reference Thread (Priority: AboveNormal):** Toggles the setpoint every 4s. Given higher priority to ensure precise step-response timing.
* **Controller Thread (Priority: Normal, Period: 10ms):** Samples the encoder, calculates RPM, and executes the control law.

**Targets 2 & 3: Distributed (Client/Server)**

* **System Main (Priority: AboveNormal):** I.e. `main_id` thread, that handles hardware initialization, network connection establishment, and system reset logic during recovery.

* **Client:**
* **Sampling Thread (Priority: Idle):** Timer-driven (10ms). Reads encoder data and triggers the network thread.
* **TxRx Thread (Priority: BelowNormal):** Handles blocking TCP communication. Sends velocity and waits for control data.
* **Actuation Thread (Priority: Normal):** Unblocked by the TxRx thread upon data receipt to immediately apply the new PWM duty cycle.
* **Toggle Thread (Priority: Low):** Periodically sends a command to flip the reference direction.


* **Server:**
* **Controller Thread (Priority: BelowNormal):** Calculates the PI control output when triggered by incoming network data.
* **Reference Thread (Priority: Normal):** Manages the setpoint trajectory.
* **TxRx Thread (Priority: Low):** Listens for incoming requests and dispatches tasks.

### 3. Distributed Network (Client-Server)

The system can be compiled into two separate firmware images communicating via **TCP/IP**:

* **Client MCU (The "Body"):**
* Reads physical sensors (Encoder TIM1).
* Sends velocity data to the server.
* Receives control signals and drives the motor (PWM TIM3).
* *Safety Feature:* Automatically halts motor if TCP connection times out.


* **Server MCU (The "Brain"):**
* Generates the reference trajectory.
* Computes the PI control output based on received velocity.
* Returns the actuation command to the client.

* **Low-Latency TCP:** The socket is configured with the `SF_TCP_NODELAY` flag. This disables Nagle’s algorithm, forcing immediate transmission of control packets (4-byte payloads) without buffering, ensuring real-time responsiveness.

#### Custom Application-Layer Protocol

The Client and Server exchange data using a synchronous byte-stream protocol over TCP. The interaction depends on the Request Byte sent by the Client:

**Mode A: Control Loop (Request `0x00`)**

1. **Request:** Client sends `0x00`.
2. **Ack:** Server replies with `0x00` to acknowledge readiness.
3. **Upstream (Velocity):** Client sends 4 bytes (int32 velocity) sequentially.
   * *Protocol Detail:* The Server validates receipt of **each byte** by sending back `0x01` as an ACK.
4. **Processing:** Server updates the PI controller with the reconstructed velocity.
5. **Downstream (Control):** Server sends 4 bytes (int32 control signal) sequentially.
   * *Protocol Detail:* The Client validates receipt of **each byte** by sending back `0x01` as an ACK.

**Mode B: Reference Toggle (Request `0x01`)**

1. **Request:** Client sends `0x01`.
2. **Action:** Server immediately flips the reference direction (Fire-and-forget; no data payload or ACK exchanged).

---

## 📂 File Structure

```text
Project_Root/
├── Source/
│   ├── main.c             # System entry, HAL init, and conditional target setup
│   ├── application.c      # Monolithic RTOS implementation (Target 1)
│   ├── app-client.c       # Distributed Client implementation (Target 2)
│   ├── app-server.c       # Distributed Server implementation (Target 3)
│   ├── controller.c       # PI Control algorithms, clamping and integer math
│   └── peripherals.c      # Low-level driver for PWM (TIM3), Encoder (TIM1), and GPIO
├── Include/
│   ├── application.h      # Application interface
│   ├── controller.h       # Controller interface
│   └── peripherals.h      # Hardware abstraction interface
├── ioLibrary_Driver-master/ # WIZnet W5500 drivers (socket.c, w5500.c, wizchip_conf.c)
└── project_ESFM.uvprojx   # Keil Project File

```

---

## ⚙️ Technical Implementation Details

### Low-Level Peripheral Control

* **Encoder:** Timer 1 (TIM1) configured in **Encoder Mode** to count pulses directly via hardware, minimizing CPU load.
* **PWM:** Timer 3 (TIM3) generates a 20 kHz PWM signal. The duty cycle is manipulated via the CCR registers.
* **GPIO:** Direct manipulation of PA5/PA6 for H-Bridge enabling/disabling.

### Integer-Only Control Math

To adhere to embedded best practices (and project constraints), no `float` types are used.

* **Bitwise Scaling:** The PI controller output (up to 30-bit effective range) is mapped to the PWM resolution (11-bit) using a fast bit-shift operation (`output >> 19`). This avoids expensive division instructions while mapping the full integer range to the timer's 0–2047 duty cycle.
* **Resolution:** The PWM requires an 11-bit integer (2047 max). The controller scales the 32-bit internal state to this range while maintaining precision.

### Integer-Only Algorithms

**1. Velocity Calculation (RPM)**
To avoid floating-point operations, velocity is calculated using a pre-computed ratio derived from the encoder resolution (2048 counts/rev after 4x decoding) and the sample time:
```c
// Speed = (Counts_per_ms * 60000 ms/min) / 2048 counts/rev
// Optimized Factor: 60000 / 2048 = 29.296875 ~= 1875 / 64
speed = (1875 * encoder_delta) / (64 * time_delta_ms);
```

**2. PI Controller Tuning**
The controller uses fixed integer gains manually tuned for the motor plant:

* **Proportional Gain ():** 20
* **Integral Time Constant ():** ~25ms (Represented as scaling factors  in the integrator term).
* **Saturation:** Error input is clamped to ±21,600 to prevent immediate overflow.

### Safety & Robustness

* **Clamping:** The integrator term is clamped to prevent "windup" when the motor stalls or cannot reach speed.
* **Fail-Safe:** In Distributed mode, the Client actively polls the W5500 **PHY Link Status Register (`PHYCFGR`)** and the TCP Socket Status. If the physical cable is disconnected or the socket state deviates from `SOCK_ESTABLISHED`, the system enters a hard reset: PWM is immediately zeroed and the controller state is wiped.
* **Auto-Recovery Sequence:** If a connection loss is detected (Cable disconnected or Socket error), the `resetSystem()` routine triggers immediately:
    1. **Hard Stop:** Motor PWM is forced to 0% duty cycle.
    2. **State Wipe:** The PI Controller's accumulated error (integrator) is reset to zero via `Controller_Reset()` to prevent sudden surges upon reconnection.
    3. **Restart:** The main thread is signaled to re-initialize the socket and begin the connection handshake from scratch.

### ⚡ Optimization: Hybrid Driver Model


While **STM32 HAL** is used for system initialization (Clocks, UART, SPI), the critical control path uses **Direct Register Access (DRA)** to minimize CPU overhead and latency:
* **Encoder Reading:** Reads `TIM1->CNT` directly to bypass HAL timer overhead.
* **Actuation:** Writes directly to `TIM3->CCR1` / `CCR2` registers for PWM updates.
* **GPIO:** Uses the Bit Set/Reset Register (`GPIOA->BSRR`) for H-Bridge enabling.

## ⚙️ Configuration Parameters

Key system behaviors are defined via macros in the source files. You can adjust these values to tune performance:

| File | Macro | Default | Description |
| --- | --- | --- | --- |
| `app-client.c` / `application.c` | `PERIOD_CTRL` | `10` (ms) | The sampling period for the PI Controller loop. |
| `app-client.c` / `application.c` | `PERIOD_REF` | `4000` (ms) | The interval at which the reference setpoint toggles direction. |
| `app-client.c` / `app-server.c` | `SERVER_PORT` | `2103` | The TCP port used for the Client-Server connection. |
| `controller.c` | `Kp_num` / `Kp_den` | `20` / `1` | Proportional gain (Integer ratio). |
| `controller.c` | `Ti_num` / `Ti_den` | `1` / `40` | Integral time constant scaling (Inverse of Ti). |

---

## 🚀 How to Run

### Prerequisites

1. Open `project_ESFM.uvprojx` in **Keil µVision**.
2. Ensure you have the **STM32L4xx Device Family Pack** installed.

### Build Targets

The project is configured with distinct Targets in Keil. Select the one matching your desired mode from the dropdown menu:

1. **Non-Distributed:** Runs the Controller and Plant on a single MCU.
2. **Client:** Compiles the code to act as the sensor/actuator node (requires Ethernet shield).
3. **Server:** Compiles the code to act as the remote controller node (requires Ethernet shield).

### Hardware Setup (Distributed)

1. **Client Node:** Connect Motor Shield + Ethernet Shield. Connect Motor to output.
2. **Server Node:** Connect Ethernet Shield.
3. **Network:** Connect both boards via Ethernet cable (Direct or via Switch).
4. **IP Configuration:**
* **Device IPs:** Local static IPs are configured in `main.c` / `Ethernet_Config` struct (Default: Client `192.168.0.11`, Server `192.168.0.10`).
* **Target IP (Client Code):** The Client specifically targets the Server at `192.168.0.10`. To change the destination, modify the `server_addr` array in `Source/app-client.c`.

### 🔌 Monitoring & Debugging

The application outputs system status logs via UART (printf). To view the connection sequence and real-time state:

1. Connect the Nucleo board via USB.
2. Open a Serial Terminal (e.g., **PuTTY**, **TeraTerm**).
3. Connect to the board's COM port (Settings: **115200 baud**, 8 Data bits, No Parity, 1 Stop bit).
* *Client Logs:* Displays socket connection attempts (`C > ...`) and disconnect warnings.
* *Server Logs:* Displays listening status (`S > ...`) and client connections.


### 🩺 Troubleshooting Guide

The system outputs specific logs to the Serial Terminal to indicate failure states:

| Log Message | Mode | Probable Cause | Solution |
| --- | --- | --- | --- |
| `C Ethernet cable not connected` | Client | PHY Layer down | Check Ethernet cable connection to the switch/router. |
| `C Failed to connect to server` | Client | TCP Handshake failed | Ensure Server is powered on, running, and IP matches `server_addr` in `app-client.c`. |
| `C Connection lost! System reset...` | Client | Socket Error / Timeout | The Ethernet cable was pulled or Server crashed. The system will auto-zero the motor and retry. |
| `S Failed to listen` | Server | Socket Error | W5500 initialization failed. Check SPI connections on the shield. |
| `S Connection lost! System reset...` | Server | TCP Disconnect | Client disconnected unexpectedly. Server resets controller state and listens for new connection. |


---

## 🏆 Credits

Developed for **KTH Royal Institute of Technology**.
Based on the **MF2103** Course Material.
**Libraries:** STM32Cube HAL, CMSIS-RTOS (RTX), WIZnet ioLibrary_Driver.