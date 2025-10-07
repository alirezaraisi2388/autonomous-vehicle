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

## 🧠 System Overview

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

> 🧩 **Tip:**
> Run the project on **Raspberry Pi OS** with Picamera2 enabled:
>
> ```bash
> sudo raspi-config
> Interface Options → Camera → Enable
> ```

---

## 🧩 Key Files

### 1️⃣ `main.py`

Responsible for:

* Initializing motors and camera
* Capturing live frames
* Detecting lanes and calculating steering
* Sending movement commands to motors
* Displaying processed output

### 2️⃣ `MotorModule.py`

Provides functions for moving and stopping the motors:

* `move(speed, turn, t)`: moves the robot forward/backward and turns
* `stop(t)`: stops all movement for a given duration

Example:

```python
motor.move(0.6, 0, 2)   # Move forward
motor.move(-0.5, 0.2, 2) # Move backward with slight right turn
motor.stop(1)
```

### 3️⃣ `image_processing.py`

Performs all image analysis steps:

* `detect_edges(frame)` – converts to HSV, filters white, applies Canny
* `region_of_interest(edges)` – masks bottom half of the image
* `detect_line_segments()` – detects line segments via Hough Transform
* `average_slope_intercept()` – merges left/right lines
* `get_steering_angle()` – computes lane-based steering direction
* `stabilize_steering_angle()` – smooths steering changes
* `display_heading_line()` – draws the red steering line

---

## 🧭 Processing Pipeline

1. Capture frame from camera
2. Detect lane lines
3. Compute steering angle
4. Stabilize the steering angle
5. Send command to motors
6. Display processed image

---

## ▶️ Running the Project

```bash
python3 main.py
```

Press **`Ctrl + C`** or **`q`** to stop the program.
When finished, all motors stop and OpenCV windows close automatically.

---

## ⚙️ Hardware Setup

* **Raspberry Pi 4/5**
* **Picamera2**
* **Motor Driver (L298N or L293D)**
* **Two DC Motors with Wheels**
* **5V–12V Power Supply**

---

## 💡 Optimization Tips

* Adjust white color range in `detect_edges()` for different lighting:

  ```python
  lower_white = np.array([0, 0, 200])
  upper_white = np.array([180, 30, 255])
  ```
* Add Gaussian blur to reduce noise before edge detection.
* Tune `max_angle_deviation` in `stabilize_steering_angle()` to smooth steering.

---

## 🧩 Future Improvements

This project is just a **starting point** — it can be expanded with:

* PID-based steering control
* Obstacle detection (using ultrasonic or LiDAR sensors)
* Object recognition and traffic sign detection
* Integration with ROS and Gazebo simulators

---

## 👨‍💻 Author

**AlirezaRaisi**
Developer & Researcher in **Robotics and Intelligent Systems**,
focused on **Computer Vision** and **Autonomous Vehicle Control**.
