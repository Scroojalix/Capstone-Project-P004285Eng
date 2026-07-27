# Installation Instructions

This file provides installation instructions to run the project.

## NVIDIA IsaacSim Simulation

### Installing ROS 2 Jazzy for Windows via Pixi

1. Install ROS 2 Jazzy Jalisco for Windows by following the instructions on the [ROS website](https://docs.ros.org/en/jazzy/Installation/Windows-Install-Binary.html). When prompted to download the latest ROS package from the releases page, download ROS 2 Jazzy Jalisco - Patch Release 7, as this is the last release with a Windows binary included.
2. Verify ROS installation by sourcing the ROS environment (`call C:\pixi_ws\ros2-windows\local_setup.bat`) and running the `ros2` command.

### Installing NVIDIA Isaac Sim 6.0.0

1. Install NVIDIA Isaac Sim 6.0.0 for Windows by following the instructions on the [Isaac Sim website](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/quick-install.html).
2. Verify Isaac Sim installation by launching Isaac Sim. Open a CMD, navigate to the install location, and call `isaac-sim.bat`.

## Gazebo Simulation (Old)

### Installing Windows Subsystem for Linux (WSL)

1. Open a command line with administrative privileges.
2. Type the command `wsl --install -d Ubuntu-24.04`.
3. Set the installed distro as default distro: `wsl --setdefault Ubuntu-24.04`. Ensure this command is run inside a windows CMD terminal, not within the WSL shell.
4. Launch WSL shell by opening a CMD and typing `wsl`.

### Installing ROS 2 for WSL

1. Open a WSL shell.
2. Install ROS 2 Jazzy Jalisco for Ubuntu by following the instructions on the [ROS website](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html). Download the full desktop install, not the base install.
3. Verify ROS installation by sourcing ROS environment (`source /opt/ros/jazzy/setup.bash`) and running the `ros2` command.

### Installing Gazebo

1. Open a WSL shell.
2. Install Gazebo Harmonic for Ubuntu by following the instructions on the [Gazebo Website](https://gazebosim.org/docs/harmonic/install_ubuntu/)
3. Verify installation was successful by running `gz sim`. Gazebo simulation should launch.

### Installing ROS + Gazebo Bridge

1. Open a WSL shell.
2. Type `sudo apt-get install ros-jazzy-ros-gz`

## (Optional) GitHub Authentication

1. Open a WSL shell.
2. Install Github authenticator using `sudo apt install gh`
3. Type `gh auth login`
4. Press enter on all default selected options, and follow instructions to authenticate GitHub account via web browser.
5. Type the following commands to link git to your GitHub account:
    - `git config --global user.name "YourGitHubUsername"`
    - `git config --global user.email "example@gmail.com"`

### (Optional) Setup VSCode Coding Environment

1. Install [VSCode](https://code.visualstudio.com/) for Windows.
2. Within a WSL shell, type `code` to install VSCode server for linux.
3. VSCode should open. If not, open it manually.
4. Within VSCode, ensure you are connected to the WSL remote host by clicking the `><` symbol in the bottom left corner. If this symbol isn't visible, right click on the bottom of the VSCode window, and ensure the `Remote Host` label is enabled.
5. Navigate to `/home/(name)/doc`. If this directory doesn't exist, create it.
6. Clone this repository using `git clone https://github.com/Scroojalix/Capstone-Project-P004285Eng.git`.
7. Install the `Robotics Developer Environment` extension to allow VSCode to automatically source the ROS environment, thus allowing for syntax highlighting on ROS functions and classes.
