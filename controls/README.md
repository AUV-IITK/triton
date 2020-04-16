# Controls

### To run the PID-controller using scripts

```bash
$ roscore
$ roslaunch underwater_vehicle_dynamics UWSim_g500_dynamics.launch
```
Running the PID controller
```bash
rosrun controls dynamic_control
```

### To run the simulink model:

1.  Clone the repository.
2. Open the pid_simulink.slx file in matlab.
3. Run the command:
```bash
$ roslaunch underwater_vehicle_dynamics UWSim_g500_dynamics.launch
```
4. Finally run the model in simulink.
