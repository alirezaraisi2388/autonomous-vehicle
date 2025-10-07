# 🚗 Lane Detection & Motor Control with Raspberry Pi

This project is a **prototype** for detecting road lane lines and controlling the movement of a small robot or vehicle using a **Raspberry Pi**.
The goal of this initial test is to implement a simple computer vision system that identifies road lanes and calculates the steering angle to guide the vehicle automatically.
The project is written in **Python**, using **OpenCV**, **NumPy**, and **RPi.GPIO** libraries.

> ⚙️ **Note:**
> This is an **initial test version**, and it has great potential for further development and extension in future versions.

---

## 📁 Project Structure

### 🔹 Main File (`main.py`)

The main control loop of the project:

* Captures live video from the **Raspberry Pi Camera**
* Processes each frame to detect lane lines
* Calculates and stabilizes the steering angle
* Displays real-time visual output with the detected lanes
* Controls the motors via the `Motor` class for steering and movement

### 🔹 Motor Control File (`MotorModule.py`)

Contains the `Motor` class for controlling two DC motors using the **L298N driver** and Raspberry Pi GPIO pins.
This class allows forward, backward, and turning control by adjusting PWM signals and motor direction pins.

### 🔹 Image Processing File (`image_processing.py`)

Handles all computer vision tasks, including:

* Edge detection using **Canny**
* ROI (Region of Interest) masking
* Line detection with **Hough Transform**
* Averaging lane lines and calculating slope/intercept
* Estimating and stabilizing the steering angle
* Drawing visual lane overlays and steering direction lines

---

##  System Overview

1. The **camera** captures live video from the front of the vehicle.
2. Each frame is passed into the `image_processing` module to detect lane lines.
3. The system computes the **steering angle** based on the lane position.
4. The steering angle is stabilized to prevent sudden changes.
5. The new angle is sent to the `MotorModule` to control the vehicle direction.
6. The processed image, including detected lanes and heading line, is displayed in real time.

---

## ⚙️ Dependencies

Make sure these libraries are installed before running:

```bash
sudo apt install python3-opencv python3-picamera2
pip install numpy RPi.GPIO
```

## ⚙️ Hardware Setup

* **Raspberry Pi 4/5**
* **Picamera2**
* **Motor Driver (L298N or L293D)**
* **Two DC Motors with Wheels**
* **5V–12V Power Supply**


