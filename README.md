# Gazebo/ROS2 Quadcopter Simulation

This repository contains vanilla environment based on PX4 x500 quadcopter model using Gazebo and ROS2. The simulation is ideal for development and testing of flight control algorithms using ROS natively.

## Prerequisites

- **Ubuntu 24.04**
- **ROS2 Jazzy**
- **Gazebo Harmonic**
- **RViz2**

For Docker-based setup:
- **Docker**
- **NVIDIA Container Toolkit** (optional for GPU support)

## Installation Instructions

### Ubuntu 24.04 Setup

1. **Clone the repository**:
   ```bash
   git clone https://github.com/damanikjosh/x500_simulation
   ```

2. **Navigate to the cloned directory**:
   ```bash
   cd x500_simulation
   ```

3. **Initialize and update ROS dependencies**:
   ```bash
   sudo rosdep init
   rosdep update
   rosdep install --from-paths src -y --ignore-src
   ```

4. **Launch the simulation**:
   To run the default x500 model:
   ```bash
   ros2 launch x500_bringup x500.launch.py
   ```

### Docker Setup

1. **Preparation**:
   If you do not use an NVIDIA GPU, ensure to comment out the line `runtime: nvidia` in the `docker-compose.yml` to avoid compatibility issues.

2. **Build the Docker image**:
   ```bash
   docker compose build
   ```

3. **Run the simulation**:
   To deploy the default x500 model:
   ```bash
   docker compose up
   ```

## Todo

- [x] Base x500 model
- [x] x500_depth model
- [ ] x500_gimbal model
- [ ] x500_lidar model
- [ ] x500_mono_cam model
- [ ] x500_mono_cam_down model
- [ ] x500_vision

## License

This project is licensed under the BSD 3-Clause - see the `LICENSE.txt` file for details.
