# DCSmartMotor

**Created by Waqar Farooq**  
Mechatronics Lab, Department of Mechanical Engineering  
Islamic University of Science & Technology, Kashmir

---

## 🎯 Description

DCSmartMotor is an easy-to-use Arduino library for controlling **DC motors with encoders** using **PID control** and angle-based movement. Inspired by Servo.h, but for DC motors!

---

## 🔧 Features

- Control up to **6 DC motors**
- Works with **single- or dual-channel motor drivers**
- **Encoder feedback** (quadrature)
- **PID control** for precise movement
- `moveToAngle()`, `currentAngle()`, `stop()` functions
- Easy `attach()` like Servo.h

---

## 📦 Example Usage

```cpp
#include <DCSmartMotor.h>

DCSmartMotor motor1;

void setup() {
    motor1.attach(9, 8, 2, 3);
    motor1.setPID(2.0, 0.5, 0.1);
    motor1.setCPR(1024);
    motor1.setStepsPerDegree(2.84);
    motor1.moveToAngle(90);
}

void loop() {
    motor1.update(); // Call regularly for PID
}
