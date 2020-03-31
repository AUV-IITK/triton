
# Triton: Software Stack

TODO: UPDATE README

The structure of this repository will follow the broadly be like that of Anahita, but with some changes. 

To run this repository:


### Create a catkin worspace following the guidelines given here

mkdir -p ~/auv_ws/src
cd ~/auv_ws/src
catkin_init_workspace
cd ..

### Clone this repository to your catkin workspace

cd ~/auv_ws/src
git clone https://github.com/AUV-IITK/AnahitaPlus.git

### Add the simulation repository also (uuv_simulator)
git clone https://github.com/AUV-IITK/underwater_simulation

Install all dependency packages to run the repository

You can build and install those packages from their respective sources or you can use the following command in Ubuntu 16.04 to install them. If you are building from source or using a different package manager, make sure you are building the melodic version of these packages to ensure maximum compatibility.

cd ~/catkin_ws

### Contribution Guidelines

To get started with contributing to this repository, look out for open issues here. Kindly read the Developer's Guide before sending a pull request! :)