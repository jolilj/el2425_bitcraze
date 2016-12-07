# Project Review Week 7

After we had achieved our goal of hovering the crazyflie drone, the goal of the following week was to make one drone follow a trajectory. Subsequently another drone was introduced ... 

## Week 5

The fifth project week was devoted to the goal of making one drone follow a simple straight trajectory. This cannot be done by simply setting a goal point at the target because in this case the internal PID controller is causing large acceleration in the beginning due to the large control error. To ensure a steady flight with a velocity independent from the distance to the goal, we implemented a simple trajectory planner. It sets intermediate goal points that are linearly interpolated between the start and target point. 

We also implemented a customized launch file that is not loading joystick related packages and more importantly, not publishing goals by itself. 

## Week 6

In the sixth project week we implemented a script to plot the planned and actual trajectory of the crazyflie in the RViz environment. One the one hand, this is useful to simulate the trajectory of the crazyflie (which will be even more important with two drones later to see if we successfully avoid collisions). On the other hand, the plotter tool enables us to examine to which extent the crazyflie follows the specified trajectory, i.e. if unwanted behaviour is caused by bugs in the trajectory planning or by falsy position estimates/a bad controller.

...

## Week 7


## Project Plan

There are no major changes in the project plan. 

...moving collision avoidance to week 7?

The original project plan was unspecific about the path planning task that we actually want to complete. We now decided that we want to swap positions of two drones in order to test how successful we implemented collision avoidance. If there is time left, we will also try to implement that the drones are following a moving target e.g. one drone is following after the other.
