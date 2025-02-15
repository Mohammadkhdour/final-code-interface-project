# **ESP32 Maze-Solving Robot Using Right-Hand Rule Algorithm**

🚀 **An Autonomous Robot that Navigates a Maze Using IR Sensors, LiDAR, and PID Control**

## **Overview**

This project implements an autonomous robot that solves a maze using the **Right-Hand Rule Algorithm**. It is powered by an **ESP32 Dev Module**, uses **two IR sensors** for left and right wall detection, and a **LiDAR sensor** for detecting front obstacles. The robot utilizes **PID control** for precise movement and an **MPU6050 sensor** for orientation tracking.

---

## **Table of Contents**

- [How It Works](#how-it-works)
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Installation & Usage](#installation--usage)
- [Algorithm Explanation](#algorithm-explanation)
- [Code Structure](#code-structure)
- [Future Improvements](#future-improvements)
- [License](#license)

---

## **How It Works**

1. **PID Control for Straight-Line Movement:** Ensures stable forward motion.
2. **MPU6050 for Heading Correction:** Keeps the robot aligned with the path.
3. **Right-Hand Rule Algorithm:** Uses **IR sensors** to detect walls and follow the right-hand rule.
4. **LiDAR for Front Obstacle Detection:** Detects dead ends and turns accordingly.
5. **Encoders for Distance Measurement:** Helps control precise movement.

---

## **Hardware Requirements**

✔ **Microcontroller:** ESP32 Dev Module\
✔ **Motor Driver:** H-Bridge (L298N or L293D)\
✔ **Motors:** 2 DC Motors with Encoders\
✔ **MPU6050:** Gyroscope & Accelerometer for heading correction\
✔ **IR Sensors (x2):** For left and right wall detection\
✔ **LiDAR Sensor:** For detecting front obstacles\
✔ **Battery Pack:** Power supply for mobility

---

## **Software Requirements**

🔹 **Arduino IDE with ESP32 Board Support**\
🔹 **MPU6050\_light Library**\
🔹 **PID\_v1 Library**\
🔹 **ESP32 WiFi Library (if remote monitoring is needed)**

---

## **Installation & Usage**

### **1. Clone the Repository**

```bash
git clone https://github.com/yourusername/maze-solving-robot.git
cd maze-solving-robot
```

### **2. Upload Code to ESP32**

1. Open `project.ino` in **Arduino IDE**.
2. Install required libraries (`MPU6050_light`, `PID_v1`).
3. Select **ESP32 Dev Module** as the board.
4. Connect ESP32 via USB and upload the code.

### **3. Run the Robot**

- Place the robot at the maze entrance.
- Power it on.
- Watch it navigate the maze using the **Right-Hand Rule**!

---

## **Algorithm Explanation**

📌 **Right-Hand Rule Algorithm:**

> The robot keeps its right side close to the wall and follows it until it reaches the exit.

### **Decision-Making Process**

1. If **no right wall**, turn **right**.
2. If **right wall exists & front is clear**, move **forward**.
3. If **front wall detected (LiDAR)**, turn **left**.
4. If completely blocked, make a **U-turn**.

---

## **Future Improvements**

✅ **Implement FloodFill algorithm for Different Mazes**\
✅ **Enhance LiDAR Detection for Better Accuracy**\
✅ **Optimize PID Tuning for Smoother Turns**\
✅ **Add WiFi Connectivity for Remote Monitoring**

---

## **License**

📝 **MIT License**\
Feel free to use and modify this project!

---

🚀 **Contributions are welcome!**\
Star ⭐ this repository if you find it useful! 😊

---

## **Next Steps**

Let me know if you need more documentation, wiring diagrams, or troubleshooting guides! 🚀

