# 🤖 4WD Sumo-Style Border-Avoid & Obstacle-Pushing Robot

<div align="center">

![Robot Model](Media/model_1.jpg)

**An intelligent 4-wheel drive robot designed for sumo-style competitions with autonomous border detection and obstacle pushing capabilities**

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Arduino](https://img.shields.io/badge/Arduino-Compatible-blue.svg)](https://www.arduino.cc/)
[![Author](https://img.shields.io/badge/Author-T.M.N.P.Thennakoon-green.svg)](https://github.com/TMNPThennakoon)

</div>

---

## 📋 Overview

This project features a sophisticated 4WD autonomous robot built for sumo-style competitions. The robot intelligently navigates within a white arena bounded by a black border, detecting and avoiding the border while actively seeking and pushing obstacles toward the arena edges.

### Key Capabilities

- 🎯 **Autonomous Navigation**: Freely roams within the arena boundaries
- 🚧 **Border Detection**: Uses dual IR line sensors to detect and avoid arena borders
- 📡 **Obstacle Detection**: HC-SR04 ultrasonic sensor for obstacle detection
- 💪 **Push Mode**: Automatically switches to high-speed pushing when obstacles are detected
- 🔄 **Smart Recovery**: Randomized turn algorithm to escape corners

---

## 🎥 Demo

Watch the robot in action:

<div align="center">

*Demo video available in the `Media/` folder (Demo.MP4)*

> **Note:** The demonstration video is excluded from the repository due to GitHub's 100MB file size limit. It is available locally in the project's `Media/` directory.

</div>

---

## 📸 Robot Images

<div align="center">

| Front View | Side View | Top View |
|:----------:|:---------:|:--------:|
| ![Model 1](Media/model_1.jpg) | ![Model 2](Media/model_2.jpg) | ![Model 3 Front](Media/model_3\(front\).HEIC) |

</div>

### 🏟️ Arena Setup

<div align="center">

![Running Area](Media/Running%20Area.HEIC)

*The white arena with black border where the robot operates*

</div>

---

## ✨ Features

- ✅ **4WD Drive System**: Rugged mechanical design with four independent motors
- ✅ **Dual IR Line Sensors**: Front-left and front-right sensors for precise border detection
- ✅ **Ultrasonic Obstacle Sensor**: HC-SR04 for accurate distance measurement
- ✅ **State Machine Architecture**: Two operational modes with smooth transitions
  - `MODE_ROAM` – Free roaming inside the arena
  - `MODE_PUSH` – High-speed pushing mode when obstacles are detected
- ✅ **Intelligent Border Handling**: Different strategies for left, right, and corner encounters
- ✅ **Randomized Escape**: Helps robot escape corners and tight spaces
- ✅ **Optional Servo Scanner**: Standard servo for advanced sensor scanning (configurable)

---

## 🔧 Hardware Components

### Main Components

| Component | Model/Specification | Quantity |
|-----------|-------------------|----------|
| **Microcontroller** | Arduino (Uno/Nano compatible) | 1 |
| **Motor Driver** | Adafruit Motor Shield v1 | 1 |
| **DC Motors** | Geared DC motors for 4WD chassis | 4 |
| **Ultrasonic Sensor** | HC-SR04 | 1 |
| **IR Line Sensors** | IR reflection sensors (down-facing) | 2 |
| **Servo Motor** | Standard servo (optional) | 1 |
| **Power Supply** | Battery pack + regulated 5V | 1 |

---

## 📍 Pin Configuration

### Ultrasonic Sensor (HC-SR04)

| Function | Arduino Pin | Notes |
|----------|-------------|-------|
| TRIG | A1 | Trigger pin |
| ECHO | A0 | Echo pin |
| VCC | 5V | Power |
| GND | GND | Ground |

### IR Line Sensors (Digital)

| Sensor | Arduino Pin | Description |
|--------|-------------|-------------|
| IR Front Left | A4 | Down-facing, near left front wheel |
| IR Front Right | A5 | Down-facing, near right front wheel |

> **Note:** The code assumes:
> - **White area** → `LOW (0)`
> - **Black line** → `HIGH (1)`
> 
> If your sensors work inversely, invert the readings in the code.

### Motor Configuration (Adafruit Motor Shield v1)

| Motor # | Position | Side | Description |
|---------|----------|------|-------------|
| M1 | Front Left | Left | Front left wheel |
| M2 | Back Left | Left | Rear left wheel |
| M3 | Back Right | Right | Rear right wheel |
| M4 | Front Right | Right | Front right wheel |

**Control Groups:**
- **Left Side**: M1 + M2 (synchronized)
- **Right Side**: M3 + M4 (synchronized)

### Servo Motor

| Device | Arduino Pin | Function |
|--------|-------------|----------|
| Servo | 10 | Sensor scanning (optional) |

---

## 💻 Software & Libraries

### Required Arduino Libraries

Install these libraries via the Arduino Library Manager:

```cpp
#include <NewPing.h>    // For HC-SR04 ultrasonic sensor
#include <Servo.h>      // For servo motor control
#include <AFMotor.h>    // For Adafruit Motor Shield v1
```

### Installation

1. Open Arduino IDE
2. Go to **Sketch → Include Library → Manage Libraries**
3. Search and install:
   - **NewPing** by Tim Eckel
   - **Servo** (usually pre-installed)
   - **AFMotor** (Adafruit Motor Shield library)

---

## 🚀 Getting Started

### 1. Hardware Setup

1. Assemble the 4WD robot chassis with motors
2. Mount the Adafruit Motor Shield v1 on Arduino
3. Connect motors to motor shield (M1, M2, M3, M4)
4. Mount HC-SR04 ultrasonic sensor at the front
5. Install IR line sensors facing downward (front-left and front-right)
6. Connect servo motor (optional) to pin 10
7. Connect power supply (battery pack + 5V regulator)

### 2. Wiring Diagram

```
┌─────────────────────────────────────┐
│         Arduino + Motor Shield      │
├─────────────────────────────────────┤
│                                     │
│  A0 ←─ ECHO (HC-SR04)              │
│  A1 ── TRIG (HC-SR04)              │
│  A4 ←─ IR Front Left               │
│  A5 ←─ IR Front Right              │
│  10 ── Servo Signal                │
│                                     │
│  M1 ── Front Left Motor            │
│  M2 ── Back Left Motor             │
│  M3 ── Back Right Motor            │
│  M4 ── Front Right Motor           │
│                                     │
└─────────────────────────────────────┘
```

### 3. Software Installation

1. Clone this repository:
   ```bash
   git clone https://github.com/TMNPThennakoon/4WD-Sweep-Robot.git
   cd 4WD-Sweep-Robot
   ```

2. Open the Arduino sketch:
   ```
   Firmware/4wd-sumo-border-avoid-robot/4wd-sumo-border-avoid-robot.ino
   ```

3. Install required libraries (see above)

4. Select your board and port in Arduino IDE

5. Upload the code to your Arduino

### 4. Calibration

Before running, calibrate your sensors:

1. **IR Sensors**: Test if white = LOW and black = HIGH
   - If inverted, uncomment the inversion lines in code:
   ```cpp
   fl = !fl;
   fr = !fr;
   ```

2. **Ultrasonic Sensor**: Adjust `OBSTACLE_DISTANCE` if needed (default: 25 cm)

3. **Speed Settings**: Modify if needed:
   - `SPEED_FORWARD`: Normal forward speed (default: 90)
   - `SPEED_PUSH`: Push mode speed (default: 100)
   - `SPEED_TURN`: Turning speed (default: 100)

### 5. Running the Robot

1. Place the robot in the center of the white arena
2. Power on the robot
3. The robot will automatically start in `MODE_ROAM`
4. Watch it navigate, detect borders, and push obstacles!

---

## 🧠 How It Works

### State Machine

The robot operates using a simple but effective state machine:

```
┌─────────────┐
│ MODE_ROAM   │ ← Default state, free movement
└──────┬──────┘
       │ Obstacle detected (< 25cm)
       ▼
┌─────────────┐
│ MODE_PUSH   │ → High-speed pushing mode
└──────┬──────┘
       │ Obstacle lost or border detected
       ▼
┌─────────────┐
│ MODE_ROAM   │
└─────────────┘
```

### Border Detection Logic

1. **Left sensor on black**: Back up → Turn right
2. **Right sensor on black**: Back up → Turn left
3. **Both sensors on black**: Back up → Random turn (left or right)

### Obstacle Detection

- Ultrasonic sensor continuously measures distance
- Obstacle detected when distance < `OBSTACLE_DISTANCE` (25cm)
- Robot switches to push mode and accelerates
- Returns to roam mode when obstacle is lost

---

## 📁 Project Structure

```
4WD-Sweep-Robot/
│
├── Firmware/
│   └── 4wd-sumo-border-avoid-robot/
│       └── 4wd-sumo-border-avoid-robot.ino  # Main Arduino sketch
│
├── Media/
│   ├── Demo.MP4                              # Demonstration video
│   ├── model_1.jpg                           # Robot image 1
│   ├── model_2.jpg                           # Robot image 2
│   ├── model_3(front).HEIC                   # Front view
│   └── Running Area.HEIC                     # Arena setup
│
├── LICENSE                                   # MIT License
└── README.md                                 # This file
```

---

## ⚙️ Configuration

### Speed Constants

```cpp
const int SPEED_FORWARD = 90;      // Normal forward speed (0-255)
const int SPEED_BACK    = 90;      // Reverse speed
const int SPEED_TURN    = 100;     // Turning speed
const int SPEED_PUSH    = 100;     // Push mode speed
```

### Timing Constants

```cpp
const int BACK_TIME     = 300;     // Back up duration (ms)
const int TURN_TIME     = 350;     // Turn duration (ms)
```

### Sensor Thresholds

```cpp
const int OBSTACLE_DISTANCE = 25;  // Obstacle detection threshold (cm)
const int MAX_DISTANCE = 200;      // Maximum ultrasonic range (cm)
```

---

## 🐛 Troubleshooting

### Robot doesn't detect borders

- Check IR sensor wiring (A4 and A5)
- Verify sensor logic (white = LOW, black = HIGH)
- Adjust sensor mounting height
- Test sensors individually using Serial monitor

### Robot doesn't detect obstacles

- Verify HC-SR04 connections (A0 for ECHO, A1 for TRIG)
- Check sensor orientation (should face forward)
- Test with Serial monitor: `Serial.println(dist);`
- Adjust `OBSTACLE_DISTANCE` threshold

### Motors not working

- Verify Motor Shield is properly connected
- Check motor connections to shield (M1-M4)
- Ensure power supply provides sufficient current
- Test motors individually

### Robot gets stuck in corners

- Increase `BACK_TIME` for longer backup
- Increase `TURN_TIME` for wider turns
- Check that random turn logic is working

---

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request

---

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

Copyright (c) 2025 **T.M.N.P.Thennakoon**

---

## 👤 Author

**T.M.N.P.Thennakoon**

- GitHub: [@TMNPThennakoon](https://github.com/TMNPThennakoon)
- Project Repository: [4WD-Sweep-Robot](https://github.com/TMNPThennakoon/4WD-Sweep-Robot)

---

## 🙏 Acknowledgments

- Adafruit for the Motor Shield library
- Tim Eckel for the NewPing library
- Arduino community for continuous support and inspiration

---

## 📚 Additional Resources

- [Arduino Documentation](https://www.arduino.cc/reference/en/)
- [Adafruit Motor Shield v1 Guide](https://learn.adafruit.com/adafruit-motor-shield)
- [HC-SR04 Ultrasonic Sensor Guide](https://howtomechatronics.com/tutorials/arduino/ultrasonic-sensor-hc-sr04/)
- [IR Line Sensor Guide](https://www.instructables.com/How-to-Use-a-QRE1113-Infrared-Line-Sensor-With-Ard/)

---

<div align="center">

**⭐ Star this repository if you find it helpful!**

Made with ❤️ by T.M.N.P.Thennakoon

</div>
