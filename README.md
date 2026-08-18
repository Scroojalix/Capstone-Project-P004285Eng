# Capstone Project P004285Eng

This repository will contain code for an RMIT University supported engineering capstone project with ID P004285ENG.

The project focuses on implementing David Silver's Windowed Hierarchical Cooperative A\* for Multi-Agent Path Finding in NVIDIA's Isaac Sim. The main contribution involves ensuring that the algorithm can run within a physics simulator environment, and not just in idealised theoretical environments.

<b>Team members include:</b>

- Owen Bonney - s4007513
- Caleb Thomas - s3949212
- Jamie Cheong - s3945808

<b>Academic Supervisor</b>:  
Dr Wei Qin Chuah  
wei.qin.chuah@rmit.edu.au

## Running RViz Baseline Simulation

First ensure WSL and ROS2 are installed following the instructions in [INSTALL.md](/INSTALL.md).

Within a WSL shell, build the whca_rviz_baseline ROS2 package

```
colcon build --packages-select whca_rviz_baseline
```

Source the build files

```
source install/setup.bash
```

And finally, launch the RViz baseline simulation

```
ros2 launch whca_rviz_baseline whca_baseline_launch.py
```

## Running Isaac Sim Simulation

Launch a pixi shell and source ROS

```
cd C:\pixi_ws
pixi shell
call C:\pixi_ws\ros2-windows\local_setup.bat
```

Navigate to isaac directory within this repo.

```
cd <path_to_repo>\Capstone-Project-P004285Eng\isaacsim_files
```

Run provided launch script, via the Python batch script provided by Isaac Sim.

```
C:\isaacsim\python.bat launch_isaac.py
```

In a second ROS2 sourced Pixi shell, run the whca_controller script

```
python3 isaacsim_files/whca_controller.py
```
