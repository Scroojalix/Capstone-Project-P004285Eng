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

```
cd WHCA/
```

Launch RViz by opening a ROS terminal, then running the following command:
```
rviz2 -d whca_experiment.rviz
```

Then, open another ROS terminal, and launch the experiment with:
```
python3 whca_experiment_node.py
```
