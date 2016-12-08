# Crazyflie Project

## TODO
- [ ] Solder push buttons
- [ ] Write higher level target position publisher (Imran and Saber)
- [ ] Review simple collision avoidance algorithms (Hui)
- [ ] Model collision avoidance (Design) (Hui and Pedram)
- [ ] Implement collision avoidance (Coding!) (Hui and Pedram)
- [ ] Set up automatic logging and a matlab script(Pedram and Imran)
- [ ] Control two crazyflies with one or two radios (Robert, Joakim)
- [ ] Start writing project report (Caro)
- [ ] Write project review and revised project plan (Caro)
- [ ] Find optimal spot in the room for good position estimation (?) 
- [ ] Document and comment current code (Joakim)
- [x] Make custom  launch file
- [x] Override goal publisher
- [x] Try implement a custom goal publisher and publish new goal positions slowly incremented
- [x] Look up possible ways of simulating the drone

## Setup
Clone this repo along with the crazyflie ROS repo and the Bitcraze Loco Positioning repo into your catkin workspace
```
cd ~/catkin_ws/src
git clone https://github.com/jolilj/el2425_bitcraze.git
git clone https://github.com/whoenig/crazyflie_ros.git
git clone https://github.com/bitcraze/lps-ros.git
```
Make and source
```
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## Structure
The project includes the anchor position file *anchor_pos.yaml*. This needs to be copied to the *lps-ros* package.
```
cd ~/catkin_ws/src
cp el2425_bitcraze/anchor_pos.yaml lps-ros/data/anchor_pos.yaml
```

## Flying
In order to hover you need to update the crazyflie firmware config. This is done by creating a config.mk file and place it inside `crazyflie-firmware/tools/make`. The file is now in this repo and consists only of two lines. The reason is to enable the kalman filter onboard the crazyflie. See [bitcraze wiki](https://wiki.bitcraze.io/doc:lps:index) for more information.

To understand the different steps it's important to distinguish between the different reference points.
First of all the crazyflie has an internal PID controller that receives a reference point along with the position estimate from the **lps-ros package**. This reference point is published on the **goal** topic. 

Secondly, we have implemented a higher level abstraction from this. The user instead specifies a **target point** representing the wanted position of the crazyflie. The reason for this is to be able to handle large step changes in the reference, keep constant velocity etc. When the user calls the `/crazyflie/set_target_position`, intermediate **goal points** are calculated and published on the **goal topic**. These are linearly interpolated between the previous target point and the new target point.

### Step 1

Call custom launch file connect.py that takes a channel as input (ch:= 80 or ch:=125) as well as initial target position
```
roslaunch el2425_bitcraze connect.launch ch:=channel x:=x0 y:=y0 z:=z0
```


### Step 2

Wait for a while for the filter to converge, then check the position(in another terminal tab)
```
rostopic echo /crazyflie/crazyflie_position
```

### Step 3

Run `fly.py` to start hovering at initial target position.

```
rosrun el2425_bitcraze fly.py
```

### Step 4

To change the target position open a new terminal tab and call the `/crazyflie/set_target_position` service with x, y and z as arguments. **Important:** If you call the service with at least one negative input argument, you have to add ` -- ` before setting the argument
```
rosservice call /crazyflie/set_target_position x y z
```

Example:
```
rosservice call /crazyflie/set_target_position -- -1.0 1.0 1.0
```
The position can be changed by simply calling the `/set_target_position` service again.

### Step 5

Stop hovering by pressing any key in the terminal where the fly.py script is running.

### Trajectory plotting
The reference trajectory and the corresponding crazyflie trajectory is plotted in rviz. The crazyflie trajectory is plotted once a target position has been set (by calling setgoal).

## Flying Drone in a circle

Follow the step 1 to step 3 of Flying as described above.

### Step 4

Open the 'follow_traj.py' in the editor and make sure that the variables self.x_init, self.y_init and self.z_init must be equal or close enough to x0, y0 and z0, the points that you specify while launching the connect.launch in step1. 

Now run 'follow_traj.py' to fly the UAV in a circle. 
```
rosrun el2425_bitcraze follow_traj.py X r
```
where,
X = 'c' if you want to fly the drone in a circle and
r = radius of the circle and r<=0.75

## Hardware notes

### Radio addresses
To change the radio address of the Crazyflie, use the CFclient (easiest through the [virtual machine](https://www.bitcraze.io/getting-started-with-the-crazyflie-2-0/#inst-comp) to avoid installing a lot of dependencies). Instructions [here](https://wiki.bitcraze.io/doc:crazyflie:client:pycfclient:index#firmware_configuration). Go to firmware configuration section.

Crazyflie 1 URI:
`radio://0/80/2M`

Crazyflie 2 URI:
`radio://0/125/2M`

It might be possible to change the config file in `crazyflie-firmware/src/config/config.h`.
We haven't tried this though.
