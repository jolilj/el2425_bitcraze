#Instructions
## Flying
In order to hover you need to update the crazyflie firmware config. This is done by creating a config.mk file and place it inside `crazyflie-firmware/tools/make`. The file is now in this repo and consists only of two lines. The reason is to enable the kalman filter onboard the crazyflie. See [bitcraze wiki](https://wiki.bitcraze.io/doc:lps:index) for more information.

To understand the different steps it's important to distinguish between the different reference points.
First of all the crazyflie has an internal PID controller that receives a reference point along with the position estimate from the **lps-ros package**. This reference point is published on the **goal** topic. 

Secondly, we have implemented a higher level abstraction from this. The user instead specifies a **target point** representing the wanted position of the crazyflie. The reason for this is to be able to handle large step changes in the reference, keep constant velocity etc. When the user calls the `/crazyflie/set_target_position`, intermediate **goal points** are calculated and published on the **goal topic**. These are linearly interpolated between the previous target point and the new target point.

### Step 1

Call custom launch file connect.launch that takes a channel and address ending (the last two hexadecimal numbers) as input (defaults to `ch:=125 address:='E7'`) as well as initial target position (defaults to `x0:=-1.0 y0:=1.0 z:=1.5`). E.g.
```
roslaunch el2425_bitcraze connect.launch
```
to hover with crazyflie0  at -1.0 1.0 1.5 or
```
roslaunch el2425_bitcraze connect.launch address:='BC' x0:=1.0 y0:=1.0 z0:=1.5
```
to hover with crazyflie1 at 1.0 1.0 1.5.

### Step 2

Wait for a while for the filter to converge (check RViz).

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

Stop hovering by pressing `Enter` in the terminal where the fly.py script is running.

### Trajectory plotting
The reference trajectory and the corresponding crazyflie trajectory is plotted in RViz. The crazyflie trajectory is plotted once a target position has been set (by calling `/set_target_position`).
