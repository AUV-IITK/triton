# Full state feedback control

The simulink file"fsfb1.slx" contains a simple working model of full state feedback control(fsfb).It takes the velocity as input and gives the position as output.Position(output from model),velocity and position(from sensor) can be seen on the graphs and hence analysed.</br>
Some issues are there regarding the inital postion which must be given to the state space model(the bot in simulator continously changes its position).So it must be rentered everytime from the data from graph.Also one can change the values in gain(-Kx) as per required.</br>
</br>
To run the model :</br>
1)clone the repositiory and open the fsfb1.slx file in fsfb folder,in matlab.</br>
2)open the UWSim simulator(run the command given below)</br>
```
roslaunch underwater_vehicle_dynamics UWSim_g500_dynamics.launch 
```
3)run the simulink model parallelly
