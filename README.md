# Destroyer Workspace

## Purpose
This workspace is designed for the implementation and testing of GridSLAM (Grid-based Simultaneous Localization and Mapping) algorithms. It includes custom simulation environments and tools for odometry and mapping.

## GridSLAM Implementation
The core of this workspace is the GridSLAM implementation, which uses a grid-based approach to map the environment while simultaneously tracking the robot's pose. The implementation handles sensor data integration, occupancy grid updates, and pose estimation.

## Running the Simulation and SLAM

Execute each `ros2 launch` or `ros2 run` command in a separate terminal window. Follow the steps below to build the workspace and launch the necessary components.

### 1. Build the Workspace
First, build all packages in the workspace:
```bash
colcon build
```

### 2. Launch Odometry
In a new terminal, source the workspace and launch the odometry tools:
```bash
source install/setup.bash
ros2 launch tools odom.launch.py
```

### 3. Launch GridSLAM
In a new terminal, source the workspace and launch the GridSLAM node:
```bash
source install/setup.bash
ros2 launch grid_fastslam fastslam.launch.py num_particles:=50
```

### 4. Launch Custom Casa Simulation
In a new terminal, source the workspace and launch the custom simulation environment:
```bash
source install/setup.bash
ros2 launch turtlebot3_custom_simulation custom_casa.launch.py
```

### 5. Run Teleoperation
In a new terminal, run the teleoperation node to control the robot:
```bash
ros2 run turtlebot3_teleop teleop_keyboard
```

## Quick Start
To launch all components at once in separate terminal tabs, you can use the provided Makefile:
```bash
make all
```

