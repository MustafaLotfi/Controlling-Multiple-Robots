# Conrolling-Multiple-Robots
## Introduction
Controlling multiple holonomic mobile robot has been considered in this project.
The task is to transport robots from their initial conditions to their desired destinations without colliding to each other. The solution is based on potential field of each robot. Hence no trajectory planning is done for robots to perform the scenario. In this approach, in every time steps, distances and directions of robots to each other and the walls will be calculated. So robots try to go to their target positions in such a way that haven't any collision with neither other robots nor the walls.

## Dynamical system, inputs and other variables
<div align="left">
  <img src="https://github.com/MustafaLotfi/Conrolling-Multiple-Robots/blob/main/docs/images/1.png">
</div>

## Preview
### 2 robots, park at the nearest destination
<div align="left">
  <img src="https://github.com/MustafaLotfi/Conrolling-Multiple-Robots/blob/main/docs/images/2.gif">
</div>

### 2 robots, park at determined destination
<div align="left">
  <img src="https://github.com/MustafaLotfi/Conrolling-Multiple-Robots/blob/main/docs/images/3.gif">
</div>

### 9 robots, park at determined destination
<div align="left">
  <img src="https://github.com/MustafaLotfi/Conrolling-Multiple-Robots/blob/main/docs/images/4.gif">
</div>

## how ot run
1.Clone the repository.

2.Change initial conditions or number of robots in initialize.m file if you want.

3.Change problem number in line 2 of simulate_system.m file (first input of initialize function) to see both problem simulations. Second input is for finding nearest destinations to the robots.

4.Run main.m file in Matlab.
