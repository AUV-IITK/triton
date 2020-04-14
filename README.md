
# Triton: Software Stack

[![Build Status](https://travis-ci.org/AUV-IITK/triton.svg?branch=master)](https://travis-ci.org/AUV-IITK/triton)


The structure of this repository will follow the broadly be like that of Anahita, but with some changes. 

# To run this repository:

### Create a catkin worspace following the guidelines given here
```
mkdir -p ~/auv_ws/src
cd ~/auv_ws/src
catkin_init_workspace
cd ..
catkin build
```

### Add the simulation repository also (uwsim)

Official installation instructions are [here](http://www.irs.uji.es/uwsim/wiki/index.php?title=Installing_UWSim). This should resemble source-based install, since we only want to change the core files, and not the external repositories. If you face any issue during building this project, please file an issue, or contact the admin. We are in development phase.

We are using `create_scenes.sh` to place a `pool.osgt` file in the `.uwsim` directory. 

```
cd ~/auv_ws/src
git clone https://github.com/AUV-IITK/underwater_simulation
cd ~/auv_ws/
rosdep install --from-paths src --ignore-src --rosdistro melodic -y
bash create_scenes.sh
catkin config --install
catkin build -j2
```

### Clone this repository to your catkin workspace
```
cd ~/auv_ws/src
git clone https://github.com/AUV-IITK/triton.git
catkin build
```
Install all dependency packages to run the repository. Currently there are no dependencies, apart from those required by uwsim.

You can build and install those packages from their respective sources or you can use the following command in Ubuntu 16.04 to install them. If you are building from source or using a different package manager, make sure you are building the melodic version of these packages to ensure maximum compatibility.

```
cd ~/catkin_ws
catkin build
```

### Workspace structure
```
.
├── CMakeLists.txt -> /opt/ros/melodic/share/catkin/cmake/toplevel.cmake
├── triton
│   ├── controls
│   ├── decisions
│   ├── hardware
│   ├── mapping
│   ├── README.md
│   ├── utils
│   └── vision
└── underwater_simulation
    ├── README.md
    ├── setup
    ├── underwater_sensor_msgs
    ├── underwater_vehicle_dynamics
    └── uwsim
```

The branches when not doing any development work would be `master` on `triton` and `auv-master` on `underwater_simulation`.
### Contribution Guidelines

To get started with contributing to this repository, look out for open issues here. Kindly read the Developer's Guide before sending a pull request! :)
