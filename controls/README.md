# uwsim-controls

**To run the simulink model:**

1)clone the repository.</br>
2)open the pid_simulink.slx file in matlab.</br>
3)run the command:
```
roslaunch underwater_vehicle_dynamics UWSim_g500_dynamics.launch
```
4)Finally run the model in simulink.</br>

**To run the PID-controller using scripts**

**Clone this repo in src folder of your workspace**

**In another terminal window run roscore and simulation**
```bash
roscore
#in another teminal window
roslaunch underwater_vehicle_dynamics UWSim_g500_dynamics.launch
```
**Running the PID controller**
```bash
./uwsim-controls/pid_controller/scripts/dynamic_control.py
```
**Remove the underwater_simulation package from src folder after cloning**
