# Capstone Project P004285Eng

This repository will contain code for an RMIT University supported engineering capstone project with ID P004285ENG.

The focus will be on the integration of a shared obstacle map into a windowed hierarchical cooperative A* approach to multi-robot navigation.

It will feature simulations of both baseline WHCA, as well as an improved algorithm (yet to be developed).

<b>Team members include:</b>
* Owen Bonney - s4007513
* Caleb Thomas - s3949212
* Jamie Cheong - s3945808

<b>Academic Supervisor</b>:  
Dr Wei Qin Chuah  
wei.qin.chuah@rmit.edu.au

## Running Baseline Simulation
First ensure WSL and ROS2 are installed following the instructions in [INSTALL.md](/INSTALL.md).

For all following steps, ensure you're in the correct working directory, using `cd WHCABaseline/`

Launch RViz by opening a ROS terminal, then running the following command:
```
rviz2 -d silver_comparison_experiment.rviz
```

Then, open another ROS terminal, and launch the experiment with:
```
python3 silver_comparison_experiment.py
```

## Running Gazebo Simulation

In a ROS terminal, build the project using
```
colcon build
```

Source the project files
```
source install/setup.bash
```

Run the simulation.
```
ros2 launch gazebo_test test_sim_launch.py
```

Ensure to press the play button on the Gazebo window that appears.