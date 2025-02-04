
# Robot Teleop Web GUI & Integrated ROS2 Nodes

This project provides a complete solution for controlling a ROS2-based robot (such as an autonomous RC car) from a web interface on a smartphone. It includes:

- A **responsive web GUI** (built with HTML/CSS/JavaScript) that sends live control commands (Twist messages) to the robot.
- ROS2 nodes for:
  - **Motor Control:** Receives `/cmd_vel` commands and controls drive/steering motors via a motor driver.
  - **IMU:** Reads data from an MPU6050 sensor over I²C and publishes IMU data.
  - **Lidar:** Integrates a RPLIDAR (or compatible) sensor and publishes LaserScan messages.

The web GUI communicates with ROS using [roslibjs](https://github.com/RobotWebTools/roslibjs) via a locally hosted copy and [rosbridge_suite](https://github.com/RobotWebTools/rosbridge_suite).

---

## Table of Contents

- [Features](#features)
- [Prerequisites](#prerequisites)
- [Project Setup and Node Creation](#project-setup-and-node-creation)
  - [ROS2 Workspace Setup](#ros2-workspace-setup)
  - [Motor Control Node](#motor-control-node)
  - [IMU Node](#imu-node)
  - [Lidar Node](#lidar-node)
- [Web GUI Setup](#web-gui-setup)
- [Usage](#usage)
- [To Do List](#to-do-list)
- [Troubleshooting](#troubleshooting)
- [License](#license)
- [Acknowledgements](#acknowledgements)

---

## Features

- **Responsive Web GUI:** Optimized for mobile devices.
- **Real-Time Control:** Commands are sent immediately upon interaction.
- **ROS2 Integration:** Nodes for motor control, IMU data, and lidar scanning.
- **Local Communication:** Uses a locally built copy of roslibjs and rosbridge for WebSocket-based communication.
- **Extensible:** Easily integrate additional sensors or controls (e.g., virtual joystick).

---

## Prerequisites

- **ROS2 Installation:** (e.g., ROS2 Humble on Ubuntu 22.04)
- **rosbridge_suite:**  
  Install and run the rosbridge server on your ROS machine:
  ```bash
  sudo apt install ros-humble-rosbridge-server
  ros2 run rosbridge_server rosbridge_websocket
  ```
- **Network Connectivity:**  
  Ensure your smartphone and ROS machine are on the same network.
- **Hardware Setup:**
  - **Motor Control:** Wiring for drive and steering motors via an L9110 motor driver.
  - **IMU:** MPU6050 sensor connected via I²C.
  - **Lidar:** RPLIDAR or SLAMTEC Lidar connected via USB (or appropriate interface).
- **Local HTTP Server:**  
  To serve the web interface (e.g., Python’s HTTP server).

---

## Project Setup and Node Creation

### ROS2 Workspace Setup

1. **Create a ROS2 Workspace (if you haven't already):**
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   ```
   
2. **Clone or Create Your ROS2 Packages:**

   - **Motor Control Node:**  
     Create a Python package for motor control.
     ```bash
     ros2 pkg create --build-type ament_python motor_control --dependencies rclpy geometry_msgs
     ```
     
   - **IMU Node:**  
     Create a Python package for the MPU6050-based IMU.
     ```bash
     ros2 pkg create --build-type ament_python mpu6050_imu --dependencies rclpy sensor_msgs
     ```
     
   - **Lidar Node:**  
     You can either use an existing package (such as [sllidar_ros2](https://github.com/Slamtec/sllidar_ros2) or [rplidar_ros](https://github.com/roboception/rplidar_ros)) or create your own. For this project, we assume you clone an existing package:
     ```bash
     cd ~/ros2_ws/src
     git clone https://github.com/Slamtec/sllidar_ros2.git
     ```

3. **Implement the Nodes:**

   ### Motor Control Node
   - **File:** `~/ros2_ws/src/motor_control/motor_control/motor_controller.py`
   - **Overview:**  
     This node subscribes to the `/cmd_vel` topic and controls the drive (back) and steering (front) motors via GPIO using PWM.
   - **Example Code:**
     ```python
     #!/usr/bin/env python3
     import rclpy
     from rclpy.node import Node
     from geometry_msgs.msg import Twist
     import RPi.GPIO as GPIO

     # Define GPIO pins for the motors (update as needed)
     BACK_MOTOR_PIN1 = 17
     BACK_MOTOR_PIN2 = 18
     FRONT_MOTOR_PIN1 = 27
     FRONT_MOTOR_PIN2 = 22

     PWM_FREQUENCY = 100

     class MotorController(Node):
         def __init__(self):
             super().__init__('motor_controller')
             GPIO.setmode(GPIO.BCM)
             GPIO.setup(BACK_MOTOR_PIN1, GPIO.OUT)
             GPIO.setup(BACK_MOTOR_PIN2, GPIO.OUT)
             GPIO.setup(FRONT_MOTOR_PIN1, GPIO.OUT)
             GPIO.setup(FRONT_MOTOR_PIN2, GPIO.OUT)

             self.back_pwm1 = GPIO.PWM(BACK_MOTOR_PIN1, PWM_FREQUENCY)
             self.back_pwm2 = GPIO.PWM(BACK_MOTOR_PIN2, PWM_FREQUENCY)
             self.front_pwm1 = GPIO.PWM(FRONT_MOTOR_PIN1, PWM_FREQUENCY)
             self.front_pwm2 = GPIO.PWM(FRONT_MOTOR_PIN2, PWM_FREQUENCY)

             self.back_pwm1.start(0)
             self.back_pwm2.start(0)
             self.front_pwm1.start(0)
             self.front_pwm2.start(0)

             self.subscription = self.create_subscription(
                 Twist,
                 '/cmd_vel',
                 self.cmd_vel_callback,
                 10
             )
             self.get_logger().info("Motor Controller Node started.")

         def cmd_vel_callback(self, msg: Twist):
             speed = msg.linear.x
             steer = msg.angular.z
             duty_speed = abs(speed) * 100  # Scale as needed
             duty_steer = abs(steer) * 100  # Scale as needed

             if speed > 0:
                 self.back_pwm1.ChangeDutyCycle(duty_speed)
                 self.back_pwm2.ChangeDutyCycle(0)
             elif speed < 0:
                 self.back_pwm1.ChangeDutyCycle(0)
                 self.back_pwm2.ChangeDutyCycle(duty_speed)
             else:
                 self.back_pwm1.ChangeDutyCycle(0)
                 self.back_pwm2.ChangeDutyCycle(0)

             if steer > 0:
                 self.front_pwm1.ChangeDutyCycle(duty_steer)
                 self.front_pwm2.ChangeDutyCycle(0)
             elif steer < 0:
                 self.front_pwm1.ChangeDutyCycle(0)
                 self.front_pwm2.ChangeDutyCycle(duty_steer)
             else:
                 self.front_pwm1.ChangeDutyCycle(0)
                 self.front_pwm2.ChangeDutyCycle(0)

         def destroy_node(self):
             self.back_pwm1.stop()
             self.back_pwm2.stop()
             self.front_pwm1.stop()
             self.front_pwm2.stop()
             GPIO.cleanup()
             super().destroy_node()

     def main(args=None):
         rclpy.init(args=args)
         node = MotorController()
         rclpy.spin(node)
         node.destroy_node()
         rclpy.shutdown()

     if __name__ == '__main__':
         main()
     ```
   - **Build the Package:**
     ```bash
     cd ~/ros2_ws
     colcon build --packages-select motor_control
     source install/setup.bash
     ```

   ### IMU Node
   - **File:** `~/ros2_ws/src/mpu6050_imu/mpu6050_imu/mpu6050_node.py`
   - **Overview:**  
     This node reads data from an MPU6050 sensor over I²C and publishes `sensor_msgs/Imu` messages.
   - **Example Code:**
     ```python
     #!/usr/bin/env python3
     import rclpy
     from rclpy.node import Node
     from sensor_msgs.msg import Imu
     import math, time
     from smbus2 import SMBus

     MPU6050_ADDR = 0x68
     PWR_MGMT_1   = 0x6B
     ACCEL_XOUT_H = 0x3B
     GYRO_XOUT_H  = 0x43

     def read_word(bus, reg):
         high = bus.read_byte_data(MPU6050_ADDR, reg)
         low = bus.read_byte_data(MPU6050_ADDR, reg+1)
         value = (high << 8) + low
         if value >= 0x8000:
             value = -((65535 - value) + 1)
         return value

     class MPU6050Node(Node):
         def __init__(self):
             super().__init__('mpu6050_node')
             self.publisher_ = self.create_publisher(Imu, '/imu/data', 10)
             self.timer = self.create_timer(0.02, self.timer_callback)
             self.bus = SMBus(1)
             self.bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0)
             self.get_logger().info("MPU6050 Node started.")

         def timer_callback(self):
             imu_msg = Imu()
             imu_msg.header.stamp = self.get_clock().now().to_msg()
             imu_msg.header.frame_id = "imu_link"

             try:
                 accel_x = read_word(self.bus, ACCEL_XOUT_H)
                 accel_y = read_word(self.bus, ACCEL_XOUT_H+2)
                 accel_z = read_word(self.bus, ACCEL_XOUT_H+4)
                 imu_msg.linear_acceleration.x = accel_x / 16384.0 * 9.81
                 imu_msg.linear_acceleration.y = accel_y / 16384.0 * 9.81
                 imu_msg.linear_acceleration.z = accel_z / 16384.0 * 9.81

                 gyro_x = read_word(self.bus, GYRO_XOUT_H)
                 gyro_y = read_word(self.bus, GYRO_XOUT_H+2)
                 gyro_z = read_word(self.bus, GYRO_XOUT_H+4)
                 imu_msg.angular_velocity.x = math.radians(gyro_x / 131.0)
                 imu_msg.angular_velocity.y = math.radians(gyro_y / 131.0)
                 imu_msg.angular_velocity.z = math.radians(gyro_z / 131.0)

                 imu_msg.orientation_covariance[0] = -1  # No orientation estimation
             except Exception as e:
                 self.get_logger().error("Error reading MPU6050: " + str(e))

             self.publisher_.publish(imu_msg)

     def main(args=None):
         rclpy.init(args=args)
         node = MPU6050Node()
         rclpy.spin(node)
         node.destroy_node()
         rclpy.shutdown()

     if __name__ == '__main__':
         main()
     ```
   - **Build the Package:**
     ```bash
     cd ~/ros2_ws
     colcon build --packages-select mpu6050_imu
     source install/setup.bash
     ```

   ### Lidar Node
   - **Option:** Use an existing package such as [sllidar_ros2](https://github.com/Slamtec/sllidar_ros2).
   - **Installation:**  
     Clone the repository into your workspace:
     ```bash
     cd ~/ros2_ws/src
     git clone https://github.com/Slamtec/sllidar_ros2.git
     ```
   - **Build the Package:**
     ```bash
     cd ~/ros2_ws
     colcon build --packages-select sllidar_ros2
     source install/setup.bash
     ```
   - **Running the Lidar Node:**  
     Use the provided launch files or run the node directly. For example:
     ```bash
     ros2 launch sllidar_ros2 view_sllidar_a1_launch.py
     ```
     (Modify parameters as needed.)

---

## Web GUI Setup

1. **Local Copy of ROSLIB:**  
   Since pre-built files for ROSLIB v2 are no longer provided by default, either:
   - **Option A:** Download a stable v1 release of `roslib.min.js` from [jsDelivr](https://cdn.jsdelivr.net/npm/roslib/build/roslib.min.js) and save it in your project folder.
   - **Option B:** Use NPM to install and build ROSLIB locally:
     ```bash
     npm init -y
     npm install roslib
     ```
     Then copy the built/minified file (`roslib.min.js`) from `node_modules/roslib/` (or from the build output folder) to your project folder.
2. **Web GUI Files:**  
   Your project folder should include:
   - `index.html` – the teleop GUI (see below for code)
   - `roslib.min.js` – the locally hosted JavaScript file

3. **Responsive Teleop GUI (index.html):**  
   Refer to the [Web GUI code section](#web-gui-code) below.

### Web GUI Code

```html
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <!-- Ensure mobile responsiveness -->
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Robot Teleop GUI</title>
  <script src="roslib.min.js"></script>
  <style>
    body {
      font-family: Arial, sans-serif;
      margin: 10px;
      text-align: center;
      background-color: #f7f7f7;
    }
    h1 { font-size: 1.8em; margin-bottom: 20px; }
    .control-container { margin: 15px 0; }
    label { font-size: 1.2em; }
    input[type=range] {
      width: 80%;
      max-width: 300px;
      margin: 10px 0;
    }
    button {
      padding: 15px;
      font-size: 1.2em;
      margin: 5px;
      width: 40%;
      max-width: 200px;
      border: none;
      border-radius: 5px;
      background-color: #4285f4;
      color: #fff;
    }
    button:hover { background-color: #357ae8; cursor: pointer; }
    button.stop { background-color: #ea4335; }
    button.stop:hover { background-color: #d93025; }
  </style>
</head>
<body>
  <h1>Robot Teleoperation</h1>
  
  <!-- Speed Control Slider -->
  <div class="control-container">
    <label for="speedSlider">Speed:</label><br>
    <input type="range" id="speedSlider" min="-1" max="1" step="0.1" value="0">
    <div id="speedValue">0</div>
  </div>
  
  <!-- Steering Buttons -->
  <div class="control-container">
    <button onclick="setSteering(0.995)">Turn Left</button>
    <button onclick="setSteering(0)">Straight</button>
    <button onclick="setSteering(-0.995)">Turn Right</button>
  </div>
  
  <!-- Stop Button -->
  <div class="control-container">
    <button class="stop" onclick="stopRobot()">Stop</button>
  </div>
  
  <script>
    // Connect to rosbridge – update with your robot's IP address
    var ros = new ROSLIB.Ros({
      url: 'ws://192.168.219.100:9090'
    });

    ros.on('connection', function() { console.log('Connected to rosbridge!'); });
    ros.on('error', function(error) { console.log('Error connecting to rosbridge: ', error); });
    ros.on('close', function() { console.log('Connection closed.'); });

    // Define the /cmd_vel topic
    var cmdVel = new ROSLIB.Topic({
      ros: ros,
      name: '/cmd_vel',
      messageType: 'geometry_msgs/Twist'
    });

    var currentSpeed = 0.0;
    var currentAngular = 0.0;

    function publishCmd() {
      var twist = new ROSLIB.Message({
        linear: { x: currentSpeed, y: 0.0, z: 0.0 },
        angular: { x: 0.0, y: 0.0, z: currentAngular }
      });
      cmdVel.publish(twist);
      console.log('Published command:', twist);
    }

    // Update speed in real time
    var speedSlider = document.getElementById('speedSlider');
    var speedValue = document.getElementById('speedValue');

    speedSlider.addEventListener('input', function() {
      currentSpeed = parseFloat(this.value);
      speedValue.textContent = currentSpeed.toFixed(2);
      publishCmd();
    });

    // Set steering and publish immediately
    function setSteering(value) {
      currentAngular = value;
      console.log('Steering set to:', currentAngular);
      publishCmd();
    }

    // Stop the robot immediately
    function stopRobot() {
      currentSpeed = 0;
      currentAngular = 0;
      speedSlider.value = 0;
      speedValue.textContent = "0";
      publishCmd();
    }
  </script>
</body>
</html>
```

---

## Usage

1. **Start ROS2 Nodes:**

   - **Motor Control Node:**
     ```bash
     ros2 run motor_control motor_controller
     ```
   - **IMU Node:**
     ```bash
     ros2 run mpu6050_imu mpu6050_node
     ```
   - **Lidar Node:**  
     Either run the provided launch file (for example):
     ```bash
     ros2 launch sllidar_ros2 view_sllidar_a1_launch.py
     ```
     or run the node directly if applicable.

2. **Start the rosbridge Server:**
   ```bash
   ros2 run rosbridge_server rosbridge_websocket
   ```

3. **Serve the Web GUI:**
   In the folder containing `index.html` and `roslib.min.js`:
   ```bash
   python3 -m http.server 8000
   ```

4. **Access the GUI:**
   Open your smartphone or computer browser and navigate to:
   ```
   http://<YOUR_ROBOT_IP>:8000/index.html
   ```
   Replace `<YOUR_ROBOT_IP>` with the IP address of the machine serving the web page.

5. **Control the Robot:**
   - Adjust the speed slider for forward/backward motion.
   - Tap steering buttons to change direction.
   - Tap the stop button to halt the robot.
   
   Verify the commands are received by checking:
   ```bash
   ros2 topic echo /cmd_vel
   ```

---

## To Do List

- [ ] **Integrate Virtual Joystick:**  
  Research and integrate a virtual joystick library (e.g., [nipplejs](https://github.com/yoannmoinet/nipplejs)) for more intuitive control.
- [ ] **Enhance UI/UX:**  
  Improve the layout and styling of the web interface for better usability and visual appeal.
- [ ] **Feedback Integration:**  
  Display real-time feedback from the robot (e.g., battery level, sensor readings) on the GUI.
- [ ] **Error Handling & Alerts:**  
  Implement better error handling and visual alerts for connection issues or command failures.
- [ ] **Mobile Testing:**  
  Test the interface extensively on different mobile devices and screen sizes.
- [ ] **Documentation:**  
  Expand the documentation and add inline code comments for future maintenance and collaboration.
- [ ] **Node Integration:**  
  Further integrate and test the motor control, IMU, and lidar nodes within a complete autonomous system.

---

## Troubleshooting

- **WebSocket Connection Issues:**  
  If the browser console logs errors (e.g., `ROSLIB is not defined` or connection errors), verify:
  - `roslib.min.js` is present in your project folder.
  - The WebSocket URL in your `index.html` is correct.
- **No Command Response:**  
  Ensure:
  - The rosbridge server is running.
  - The motor control node is correctly wired and receiving `/cmd_vel` commands.
- **Hardware Issues:**  
  Verify that the motors, IMU, and lidar are properly connected and powered.

---

## License

This project is released under the [BSD License](LICENSE) (or choose another license as appropriate).

---

## Acknowledgements

- [Robot Web Tools](http://robotwebtools.org/) for roslibjs and rosbridge_suite.
- The ROS community for continuous support and documentation.
```

