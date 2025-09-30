# TLR Workspace (Traffic Light Recognition)

This project utilizes `ros2_v4l2_camera` for camera interfacing and `ultralytics_ros` for object detection with YOLO in a ROS2 environment.

## 1. Installation

First, clone the repository:
```bash
git clone https://github.com/hyeonho613/tlr_ws.git
```

## 2. Configuration

Before building, you may need to configure your camera port.

- **Modify the camera port:**
  - Open the file: `tlr_ws/src/ros2_v4l2_camera/src/v4l2_camera.cpp`
  - Find the following line and change `"your_camera_port"` to your actual device port (e.g., `"/dev/video0"`).
    ```cpp
    auto device = declare_parameter<std::string>("video_device", "your_camera_port", device_descriptor);
    ```

## 3. Build

Navigate to the workspace directory and build the packages:
```bash
cd tlr_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 4. Usage

To run the traffic light detection system, launch the main launch file:
```bash
ros2 launch tlr_detector tlr.launch.py
```

### Notes
- This single command starts both the camera node and the YOLO detection node.
- The first time you run the launch file, it will automatically download and install the YOLO model. This process may take up to a minute.

## Acknowledgements

This project is based on the following open-source projects:

- **`ros2_v4l2_camera`**: Provided by Tier4, Inc.
  - Source: [https://github.com/tier4/ros2_v4l2_camera.git](https://github.com/tier4/ros2_v4l2_camera.git)
- **`ultralytics_ros`**: Provided by Alpaca-zip.
  - Source: [https://github.com/Alpaca-zip/ultralytics_ros.git](https://github.com/Alpaca-zip/ultralytics_ros.git)
