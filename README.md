# Smart Autonomous Waste Management Robot (AI Vision + ESP32)

An end-to-end intelligent waste detection, navigation, and robotic arm sorting system powered by **YOLOv8 Object Detection** running on GPU (Laptop / Edge PC), **ESP32-CAM** (Wireless High-Speed Video Streamer), and **ESP32 DevKit** (Wireless Motor & 5-DOF Robotic Arm Controller).

---

## 🚀 Key Features

- **Real-Time AI Vision:** Custom-trained YOLOv8 model for waste classification with RTX GPU acceleration.
- **100% Wireless Operation:** 
  - **ESP32-CAM** streams video over HTTP MJPEG (`http://<CAM_IP>:81/stream`).
  - **ESP32 DevKit** receives motion & arm control commands via high-speed UDP (`<DEVKIT_IP>:8888`).
- **3-Way Intelligent Waste Sorting:**
  - 🥬 **Bin 1 - Biodegradable:** Organic waste, food leftovers, biological matter.
  - ♻️ **Bin 2 - Recyclable:** Plastics, metals, cardboard, paper, glass bottles.
  - 🚯 **Bin 3 - Non-Recyclable:** Mixed/hazardous waste, wrappers.
- **Autonomous Navigation & Target Locking:**
  - Automated centering (deadband algorithm) and distance approach using bounding-box scaling.
  - Safety wheel halt during robotic arm pick-and-sort sequences.
- **Manual Testing & Diagnostic Modes:**
  - Standalone ML detection HUD on live camera feed.
  - Interactive motor and servo hardware diagnostics.

---

## 🛠️ System Architecture

```
                    +-----------------------+
                    |   ESP32-CAM Module    |
                    | (Streams Video @ 81)  |
                    +-----------+-----------+
                                |  HTTP Video Stream
                                v
               +---------------------------------+
               |     Laptop / AI Processing      |
               |       YOLOv8 Detection          |
               | (Visual Centering & Trajectory) |
               +----------------+----------------+
                                |  Low-Latency UDP Commands
                                v
                    +-----------------------+
                    |  ESP32 DevKit V1      |
                    | (Controls Actuators)  |
                    +-----+-----------+-----+
                          |           |
            +-------------+           +-------------+
            v                                       v
+-----------------------+               +-----------------------+
|  4x DC Motors (Chassis)               | 5-DOF Robotic Arm     |
|  via L298N / TB6612   |               | (Base, Shoulder,      |
|  (Forward/Turn/Stop)  |               |  Elbow, Wrist, Claw)  |
+-----------------------+               +-----------------------+
```

---

## 📌 Hardware Pinout & Wiring

### 1. ESP32 DevKit -> Motor Driver (L298N / TB6612FNG)
| DevKit GPIO | Motor Driver Pin | Description |
|:---|:---|:---|
| **GPIO 25** | IN1 / PWMA | Left Motors Forward |
| **GPIO 26** | IN2 / AIN2 | Left Motors Backward |
| **GPIO 32** | IN3 / PWMB | Right Motors Forward |
| **GPIO 33** | IN4 / BIN2 | Right Motors Backward |

### 2. ESP32 DevKit -> 5-DOF Robotic Arm (Hardware PWM Servos)
| DevKit GPIO | Servo Joint | Description / Motion Range |
|:---|:---|:---|
| **GPIO 13** | Base Rotation | Sweeps to Bin 1 (30°), Bin 2 (90°), Bin 3 (150°) |
| **GPIO 14** | Shoulder | Vertical arm reach (30° - 140°) |
| **GPIO 27** | Elbow | Forearm pitch & extension (40° - 150°) |
| **GPIO 12** | Wrist | Orientation pitch (0° - 180°) |
| **GPIO 4**  | Gripper / Claw | Open (30°) / Clamp-Grip (110°) |

---

## 💻 Software & Script Guide

| Script / File | Purpose |
|:---|:---|
| `robot_wireless_controller.py` | Full autonomous robot loop: tracks trash, approaches, halts, triggers arm pick & sort into designated bins. |
| `test_manual_ml_cam.py` | Manual ML tester displaying live camera stream, bounding boxes, confidence, and sorting bin HUD. Press `P` to trigger pick. |
| `test_devkit_control.py` | Standalone diagnostic tool to test chassis motors and arm servos via UDP. |
| `test_system.py` | 12-test automated unit and validation test suite verifying all modules. |
| `esp32_cam_firmware/` | Arduino sketch for ESP32-CAM (AI-Thinker) video streaming server. |
| `esp32_devkit_firmware/` | Arduino sketch for ESP32 DevKit UDP receiver, motor driver & servo control. |
| `run_smart_waste_robot.bat` | One-click launcher for full autonomous robot controller. |
| `test_manual_ml.bat` | One-click launcher for manual ML camera tester. |

---

## 🚦 Quick Start Guide

### 1. Connect Devices to Wi-Fi Hotspot
Ensure your laptop, ESP32-CAM, and ESP32 DevKit are all connected to the same Wi-Fi hotspot (e.g. mobile hotspot or router).

### 2. Flash Firmware
- Open `esp32_cam_firmware/esp32_cam_firmware.ino` in Arduino IDE. Set Board: `AI Thinker ESP32-CAM`, PSRAM: `Enabled`. Flash via COM port. Note down the IP from Serial Monitor (e.g., `192.168.137.34`).
- Open `esp32_devkit_firmware/esp32_devkit_firmware.ino` in Arduino IDE. Set Board: `DOIT ESP32 DEVKIT V1`. Flash via COM port. Note down the IP from Serial Monitor (e.g., `192.168.137.218`).

### 3. Run Manual ML Camera Test
```powershell
python test_manual_ml_cam.py
```
*(Or double-click `test_manual_ml.bat`)*

### 4. Run Full Autonomous Waste Robot
```powershell
python robot_wireless_controller.py
```
*(Or double-click `run_smart_waste_robot.bat`)*

---

## 📊 Model Training & Dataset
- **Architecture:** Ultralytics YOLOv8
- **Classes:** Biodegradable, Recyclable, Non-Recyclable waste categories
- **Evaluation:** Evaluated on validation set with high precision and sub-15ms inference latency on NVIDIA RTX GPU.