# Quaternion-based-UKF

This project implements a **Quaternion-Based Unscented Kalman Filter (QNUKF)** in C++ for robust attitude estimation using MARG (Magnetometer, Accelerometer, Gyroscope) sensors. It leverages the mathematical structure of quaternions on the 3-sphere \( \mathbb{S}^3 \) and applies the Unscented Kalman Filter for nonlinear state estimation.

---

## Dataset Description

This project uses a **synthetic dataset**, generated internally by simulating MARG sensor measurements. The key design choice was to keep the variance **small** and **centered around a realistic mean**.
To switch between modes:

```cpp
const bool generate_data = false;  // ← set to true to generate new synthetic data
```

When `generate_data = false`, you can use your **own CSV dataset** for testing.

---

## 📥 Input CSV Format

The system expects MARG sensor data in the following format:

```csv
timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z
0,0.009213,-0.0029,-9.8073,0.0039,-0.0001,0.0546,0.2835,0.0316,0.5003
0.01,0.02971,-0.02098,-9.7967,-0.0043,-0.0062,0.0274,0.3101,0.0305,0.4811
...
```

---

For technical details, refer to the included paper in `/paper/`.
