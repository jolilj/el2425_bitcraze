# Literature review on multi-UAV path planning algorithms

Below are some notes on publications relevant to our mini drones project.

By doing this we intend to find a **collision avoidance** algorithm, based on **waypoint navigation** with an **external localization system**.

For more specific limitations and conditions, we will look into `problem formulation` , algorithms proposed by classic papers, and look into their concrete stages of implementation.  

## Problem Formulation

First we need to agree on **what sort of collision avoidance** we really want to achieve.

### Levels of collision avoidance

There are a whole spectrum of collision avoidance, from **maintaining relative locations to a reference** in formation flight, to **generating a collision-free path** in a cluttered environment with both fixed and moving obstacles.

_(The latter may cause some confusion, since we usually only include **vehicles colliding with each other** when we talk about collisions. However, almost all papers are obstacle-based and do not clearly distinguish between them. Besides, to generate a collision-free path one must take into consideration all obstacles, whether they are other UAVs or walls/ceilings)_

For simplicity, we will limit the concept of "collision-free" to inter-vehicle cases, and we will perform our experiment in an obstacle-free environment. If we manage both the formation flight and simultaneous arrival, we will try those path-generating algorithms. That is, if we get the time to implement a **world model** for forbidden regions, fixed and moving obstacles. 

We will start with collision-free formation flight first, where the trajectory is pre-defined and safety ranges are fixed. 
We will then move on to collision-free simultaneous arrival, which allows more freedom in path planning. 


## Several simple collision-free flight controllers

#### What is formation flight? 
 - It is defined in _Encyclopedia Britannica_ that formation flight is two or more aircraft traveling and maneuvering together in a disciplined, synchronized, predetermined manner. 
 - Formation flight of multiple UAVs is a widely researched topic in the field of cooperative control for multi-agent systems. 
 - The main goal of formation flight of multiple UAVs is to achieve a desired group formation shape while controlling the overall behavior of the group.

#### What are the possible approaches? 

|leader-follower scheme|virtual structure|behavioral approach|
|:--|:--|:--|
|some vehicles are designed as leaders while others are designed as followers;|entire formation is treated as a single virtual rigid body structure, each vehicle follows a moving point;|several desired behaviors are prescribed for each vehicle to generate a weighted average of the control for each behavior;|
|easy to understand and implement;|easy guidance,suitable for synchronized maneuvers;|suitable for uncertain environments;|
|not robust, no error propagation. | difficult to consider obstacle avoidance.|lack of a rigorous theoretic analysis|

We will start with leader-follower scheme and move on to other ones later. 

### Distributed formation control of 2 drones, based on leader-follower scheme

**Control rules**

 - The leader tracks a desired trajectory by connecting sequences of waypoints (hopefully at given velocities)
 - Global locations of leader are sent to the follower by the control center
 - The leader’s desired trajectory can be **either known or unknown** to the follower
 - The follower keeps the desired formation with respect to the leader based on received information
 - This stage of experiment is implemented in an obstacle-free environment

**Milestones for leader-follower scheme**

- [ ] accomplish a **distance-keeping hover** from a fixed distance range in a fixed direction to the initial target position;
- [ ] establish communication between two UAVs;
- [ ] replace the initial target position above, with current position of leader UAV; attempt the same distance-keeping hover and **update periodically**. There is no need to actually fly the leader UAV now, just walk around with it to see how the follower UAV catchs up with you. [Here](https://youtu.be/qvsvSVx7QfY?t=1m21s) is a video example of a leader-follower scheme on 2 drones. In our case, the current position of leader UAV should be broadcast by ROS(not by leader) ;
- [ ] actual leader-follower formation flying, leader UAV tracks a given trajectory by navigating along waypoints, while follower UAV uses the received leader UAV position information as its desired location;

_(We need to select a suitable saftety range on certain directions to avoid collision and alleviate the aerodynamic disturbance)_

If we manage this, we can try **adding more UAVs for complex formations**, possibly with virtual structure. We may also try some **decoupled** path-planning implementation, hopefully can be solved by MPC methods.  


**The hidden part of below section is messy. will clean up later**

### Decoupled simultaneous arrival path planning controllers

##### What is "decoupled"?

TODO: comparison of control architecture etc
<!---
from: Johansson, Ronnie. "Intelligent motion planning for a multi-robot system." (2001).

 | Off-line | On-line |
 | :--- | :----|
 | Sensitive to representations of the environment | Can compensate for deviations |
 | Small computation, only once | Larger and real-time computation |
 | Consider the whole workspace | Risk the local minimum |
 
(guess we have to go off-line? )

First you need to know the difference between centralized and decentralized control architecture. 

| Centralized | Decentralized |
| :-- | :-- | 
| motion planning of all UAVs is handled by a single planner | each UAV plans its motion individually |
| Global optimization is possible with global knowledge | Inherently uncoordinated |
| Growing computation demand with more UAVs | No extra computation load for extra UAVs |
| Prone to error and breakdowns | More robust |
| Limited publication | Common approach with more applications |

definition: 
to plan the motion of each robot more or less independently of the other robots and to consider the interactions among the paths in a second phase of planning. 

Chapter 2.3, Latombe, Jean-Claude. "Robot Motion Planning." (1991).

--->


##### What is simultaneous arrival? 

TODO: formulate problem clearly

[//]: # (problem formulation in: Ch 6.2, Tsourdos, Antonios, Brian White, and Madhavan Shanmugavel. Cooperative path planning of unmanned aerial vehicles. Vol. 32. John Wiley & Sons, 2010. illustration: see figure 6.4)




## Publications on Collision Avoidance

1. Hui, Cheng, Chen Yousheng, and Wong Wing Shing. "Trajectory tracking and formation flight of autonomous UAVs in GPS-denied environments using onboard sensing." Guidance, Navigation and Control Conference (CGNCC), 2014 IEEE Chinese. IEEE, 2014.

We adopt parts of their **experiment formulation**, even though the modelling and localizing techniques are very different. 

2. Chao, Zhou, et al. "Collision-free UAV formation flight control based on nonlinear MPC." Electronics, Communications and Control (ICECC), 2011 International Conference on. IEEE, 2011.

It provides a summary of pros and cons for classic simple formation flight algorithms. 
(Later we may want to try MPC it mentioned for path planning)


## Publication on Crazyflies

1. Hönig, Wolfgang, et al. "Mixed reality for robotics." Intelligent Robots and Systems (IROS), 2015 IEEE/RSJ International Conference on. IEEE, 2015.

**must cite this paper** because we use their ROS system! 


2. Luis, Carlos, and Jérôme Le Ny. "Design of a Trajectory Tracking Controller for a Nanoquadcopter." arXiv preprint arXiv:1608.05786 (2016).
Main contribution for us: 
 - develops the *mathematical model* that describes the dynamics of the Crazyflie 2.0 quadcopter;
 - Create a *simulation environment* for testing position and trajectory tracking control algorithms;

3. Landry, Benoit. Planning and control for quadrotor flight through cluttered environments. Diss. Massachusetts Institute of Technology, 2015.

GitHub repository: https://github.com/blandry/crazyflie-tools









