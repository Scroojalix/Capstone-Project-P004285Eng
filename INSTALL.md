# Installation Instructions
This file provides installation instructions to run the project.

## Installing Windows Subsystem for Linux (WSL)

1. Open a command line with administrative privileges.
2. Type the command `wsl --install -d Ubuntu-24.04`.
3. Set the installed distro as default distro: `wsl --setdefault Ubuntu-24.04`. Ensure this command is run inside a windows CMD terminal, not within the WSL shell.
4. Launch WSL shell by opening a CMD and typing `wsl`.

## Installing ROS 2
1. Open a WSL shell.
2. Install ROS 2 Jazzy Jalisco for Ubuntu by following the instructions on the [ROS website](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html). Download the full desktop install, not the base install.

## Installing Gazebo
1. Open a WSL shell.
2. Install Gazebo Harmonic for Ubuntu by following the instructions on the [Gazebo Website](https://gazebosim.org/docs/harmonic/install_ubuntu/)
3. Verify installation was successful by running `gz sim`. Gazebo simulation should launch.

## Installing ROS + Gazebo Bridge
1. Open a WSL shell.
2. Type `sudo apt-get install ros-jazzy-ros-gz`

## (Optional) Setup VSCode Coding Environment
1. Install [VSCode](https://code.visualstudio.com/)
2. Within a WSL shell, type `code` to install VSCode server for linux.
3. VSCode should open. Verify it opened within the WSL environment by clicking `Open Folder` and navigate to `/home/(name)/doc`. If this directory doesn't exist, create it.
4. Clone this repository using `git clone https://github.com/Scroojalix/Capstone-Project-P004285Eng.git`.