
# 🤖 Robotics Challenge – BiegSam

This repository contains all the Arduino code used for our robot **BIEGSAM** in the **Robotics Challenge**. The code is split into three key `.ino` files, each corresponding to a specific section of the challenge course.

> ✅ **Plug-and-play ready** — just upload the right code for each section found in the /src directory and adjust the motor driver I2C addresses if needed (default is 0x15).

---

## 🧭 Project Structure

| File                        | Purpose                              | Section     |
|-----------------------------|--------------------------------------|-------------|
| `src/section1/section1.ino`        | Line following using QTR sensors     | Section 1   |
| `src/section2/section2.ino` | Obstacle navigation with wall-following (no IMU) | Section 2 |
| `src/section3/section3.ino`              | Final stage logic (likely advanced pathfinding or maneuvers) | Section 3 |

---

## 🔧 Hardware Requirements

- **Arduino Uno/GIGA**
- **Motoron I2C Motor Drivers** (x2)
- **IR Distance Sensors** (GP2Y0A51SK0F) (x2)
- **Ultrasonic Sensors** (1x HC-SR04)
- **QTR Line Sensors** (1 x (4x9) + 3 x (4x1))
- **Servo Motor** (for obstacle tasks)
- **Power supply**
- **Chassis + Motors + Wheels**

---

## 🛠️ Setup Instructions

### 1. Clone the Repository
Upload this code to your GitHub and clone or download it to your local computer.

```bash
git clone https://github.com/Ethan-Hocquellet/BiegSam/
```

### 2. Open with Arduino IDE
Each section has its own `.ino` file. Open the appropriate one in **Arduino IDE** when testing a section.

---

## ⚙️ Config: Motor Driver Address

Your robot uses **two Motoron I2C motor drivers**. Make sure the I2C addresses match your setup. In your code, you’ll see lines like:

```cpp
MotoronI2C mc1(0x10);  // Motor driver 1
MotoronI2C mc2(0x50);  // Motor driver 2
```
## Hint 
Refer to the I2CSetAddresses.ino script in the examples for the Motoron driver for configuring MotoronI2C motor drivers


> 📌 Change `0x10` and `0x50` to match the addresses set via jumpers or configuration.

Use an I2C scanner sketch if unsure:  
👉 [I2C Scanner Example](https://playground.arduino.cc/Main/I2cScanner/)

---

## 🚦 How to Use

### ▶ Section 1: Line Following

1. Open `section1.ino`
2. Upload to Arduino.
3. Place the robot at the start of the **black line** on Section 1.
4. Robot uses QTR sensors for PID line following.

### ▶ Section 2: Obstacle Challenge (Wall Following)

1. Open `section2.ino`
2. Upload to Arduino.
3. Robot will autonomously:
   - Handle 5 obstacles (e.g., ramp, staircase, treadmill).
   - Use IR sensors + ultrasonic for wall detection.
   - Automatically switch to wall-following and obstacle logic.
4. Obstacle types are defined in:
   ```cpp
   Obstacle obstacleOrder[] = {
       RAMP, STAIRCASE, GIANT_CAUSEWAY, TREADMILL, LUNAR_SURFACE
   };
   ```

> Includes a PID wall-following algorithm with servo actuation for obstacle response.

### ▶ Section 3: Final Challenge

1. Open `section3.ino`
2. Upload to Arduino.
3. Executes final stage logic. (Adjust behavior as required — currently includes motor drive routines and delay-based maneuvers.)

---

## 📋 Libraries Used

Install these via **Arduino Library Manager**:

- `Motoron` (for motor drivers)
- `QTRSensors` (for line following)
- `Servo`
- `Wire` (built-in)
- `WiFiUdp` (built-in)
- `WiFi`
- `ICM20948_WE`

---

## 🧪 Tips for Testing

- Keep robot lifted when uploading to avoid unintended movement.
- Use **Serial Monitor** at `115200 baud` to debug sensor readings.
- Charge batteries before full-run tests — low voltage = erratic behavior.


## 📄 License

This project is released under the MIT License.  
Feel free to build, adapt, and learn from it for future challenges.

---
