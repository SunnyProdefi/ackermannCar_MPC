# ackermannCar_MPC

MPC Trajectory Tracking for Ackermann Steering Vehicles

## Overview

This project implements a Model Predictive Control (MPC) algorithm for trajectory tracking of vehicles with Ackermann steering geometry.

## Installation

1. Clone the repository:
   ```sh
   git clone https://github.com/SunnyProdefi/ackermannCar_MPC.git
   cd ackermannCar_MPC
   ```

2. Install dependencies:
   ```sh
   sudo apt-get update
   sudo apt install ros-noetic-joint-state-publisher-gui
   sudo apt install ros-noetic-ros-control
   sudo apt install ros-noetic-ros-controllers
   sudo apt install ros-noetic-gmapping
   sudo apt install ros-noetic-ackermann-msgs
   sudo apt install ros-noetic-navigation
   sudo apt install ros-noetic-teb-local-planner
   ```

3. Build the project:
   ```sh
   catkin_make
   source devel/setup.bash
   ```

## Usage

### Running the Simulation

Launch the simulation environment:
```sh
roslaunch racebot_gazebo racebot.launch
```

Run the MPC controller:
```sh
rosrun racebot_control mpc
```

## License

This project is licensed under the MIT License. See the `LICENSE` file for more details.
