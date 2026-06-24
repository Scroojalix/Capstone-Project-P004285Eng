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

For all following steps, ensure you're in the correct working directory, using `cd WHCA/`

Launch RViz by opening a ROS terminal, then running the following command:
```
rviz2 -d silver_comparison_experiment.rviz
```

Then, open another ROS terminal, and launch the experiment with:
```
python3 silver_comparison_experiment.py
```

## Running Gazebo Test Simulation

For all following steps, ensure you are in the correct working directory, using `cd GazeboTestSim/`

In a ROS terminal, launch the gazebo simulation. A gazebo window should appear.
```
gz sim test_sim.sdf
```

In a separate ROS terminal, launch the ROS Gazebo Bridge. May need to make shell script executable using `chmod +x launch_bridge.sh`, however this only needs to be run the first time you launch the program. 
```
./launch_bridge.sh
```

In a third ROS terminal, launch the python script
```
python3 test_script.py
```
