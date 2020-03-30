
# UWSim-NET: UWSim with Network Simulator
[![Build Status](http://build.ros.org/job/Mbin_uB64__uwsim__ubuntu_bionic_amd64__binary/badge/icon)](http://build.ros.org/job/Mbin_uB64__uwsim__ubuntu_bionic_amd64__binary/)

The new version of UWSim intregates a Network Simulator to be used along with the dccomms API. This simulator uses the NS3 libraries and the AquaSimNG as a NS3 module. The documentation related to simulate communications is a work in progress and will be included in the Wiki as soon as we can (please, be patience).

### Wiki
http://www.irs.uji.es/uwsim/wiki/index.php?title=Main_Page

### Citations
- Prats, M.; Perez, J.; Fernandez, J.J.; Sanz, P.J., "An open source tool for simulation and supervision of underwater intervention missions", 2012 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), pp. 2577-2582, 7-12 Oct. 2012
- Centelles, D.; Soriano-Asensi, A.; Martí, J.V.; Marín, R.; Sanz, P.J. Underwater Wireless Communications for Cooperative Robotics with UWSim-NET. Appl. Sci. 2019, 9, 3526.


### Issues
**If this repository has been cloned outside the underwater_simulation package originally installed,then replace the  original folder with the underwater_simulation folder in this repository.<br>**
**Change the root path of uwsim folder in "SimulatorConfig.h" as per yours.**

#### Exception GeographicLib::GeographicErr when starting uwsim

If you are getting the following error output when executing rosrun uwsim uwsim:
```
 ...
 data/objects/gun.osg
 Starting UWSim...
 Loading SimulatedDevices plugin: 'AcousticCommsDevice_Factory'
 Loading SimulatedDevices plugin: 'CustomCommsDevice_Factory'
 Loading SimulatedDevices plugin: 'DredgeTool_Factory'
 Loading SimulatedDevices plugin: 'ForceSensor_Factory'
 Loading SimulatedDevices plugin: 'SimDev_Echo_Factory'
  . Setting localized world: 7.1e-05s
 terminate called after throwing an instance of 'GeographicLib::GeographicErr'
  what():  File not readable /usr/share/GeographicLib/geoids/egm96-5.pgm
 /opt/ros/melodic/lib/uwsim/uwsim: line 23:  1452 Aborted                 (core dumped) rosrun uwsim uwsim_binary --dataPath ~/.uwsim/data $@
```

Run the following commands to fix it:
```
  wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
  chmod u+x install_geographiclib_datasets.sh
  sudo ./install_geographiclib_datasets.sh
```



