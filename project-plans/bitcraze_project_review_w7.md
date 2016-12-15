# The "Bitcraze group"
# Second Project Review (Week 7)

After we had achieved our goal of hovering the crazyflie drone, the goal of the following week was to make one drone follow a trajectory. Subsequently another drone was introduced with the goal to implement a multi-agent path planning task while avoiding collisions of the drones.

## Week 5

The fifth project week was devoted to the goal of making one drone follow a simple straight trajectory. This cannot be done by simply setting a goal point at the target because in this case the internal PID controller is causing large acceleration in the beginning due to the large control error. To ensure a steady flight with a velocity independent from the distance to the goal, we implemented a simple trajectory planner. It sets intermediate goal points that are linearly interpolated between the start and target point. 

We also implemented a customized launch file that is not loading joystick related packages and more importantly, not publishing goals by itself. 

## Week 6

In the sixth project week we implemented a script to plot the planned and actual trajectory of the crazyflie in the RViz environment. On the one hand, this is useful to simulate the trajectory of the crazyflie (which will be even more important with two drones later to see if we successfully avoid collisions). On the other hand, the plotter tool enables us to examine to which extent the crazyflie follows the specified trajectory, i.e. if unwanted behaviour is caused by bugs in the trajectory planning or by falsy position estimates/a bad controller. Furthermore, we implemented a higher level trajectory planner that sets target points that lie on a preplanned trajectory as e.g. a circle. 

We then also implemented a script to log information about our position measurements, to analyse how noisy the measurements actually are and if they can explain the height changes, the drone undergoes while flying.

As the goal of our project is to do multi-agent control for drones we started to look into how to control two drones at the same time. We experienced problems with the build configuration of the crazyflie firmware that had to be changed in order to fly two drones at the same time. But finally we managed to get positions of two drones.

We also started inquiring into collision avoidance algorithms and started writing a script that overrides the present trajectory with a half circle as soon as the drones get to close to each other. We also specify constraints such that the drone is not leaving a defined outer box.

&nbsp;




## Week 7

In this week, the main topic is to fly two drones at the same time and let them perform position swapping without collisions. Early this week we succeeded with having two drones flying from one script, so now the main challenge is to finalize the collision avoidance algorithm and set up a position swapping script.

## Project Plan

There are no major changes in the project plan. The original project plan was unspecific about the path planning task that we actually want to complete. We now decided that we want to swap positions of two drones in order to test how successful we implemented collision avoidance. If there is time left, we will also try to implement that the drones are following a moving target e.g. one drone is following after the other. The original project plan was aiming at finishing a collision avoidance algorithm already in week 6 but as we parallelized the work pretty efficient now, we are confident to catch up again this week.