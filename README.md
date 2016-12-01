# Crazyflie Project

## TODO
- [ ] Solder push buttons
- [x] Make custom  launch file
- [ ] Set up log tools so we can draw some conclusions about the noise and get nice plots :)
- [x] Override goal publisher
- [ ] Try implement a custom goal publisher and publish new goal positions slowly incremented
- [ ] Look up possible ways of simulating the drone

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
catking_make
source devel/setup.bash
```

## Structure
The project includes the anchor position file *anchor_pos.yaml*. This needs to be copied to the *lps-ros* package.
```
cd ~/catkin_ws/src
cp el2425_bitcraze/anchor_pos.yaml lps-ros/data/anchor_pos.yaml
```

## Hovering
In order to hover you need to update the crazyflie firmware config. This is done by creating a config.mk file and place it inside `crazyflie-firmware/tools/make`. The file is now in this repo and consists only of two lines. The reason is to enable the kalman filter onboard the crazyflie. See [bitcraze wiki](https://wiki.bitcraze.io/doc:lps:index) for more information.

### Step 1

Call custom launch file connect.py that takes a channel as input (ch:= 80 or ch:=125).

```
roslaunch el2425_bitcraze connect.py ch:=channel
```


### Step 2

Wait for a while for the filter to converge, then check the position(in another terminal tab)
```
rostopic echo /crazyflie/crazyflie_position
```

### Step 3

Run `fly.py` to start hovering with goal coordinates as argument.

```
rosrun el2425_bitcraze fly.py x y z
```
Example:

```
rosrun el2425_bitcraze fly.py -1.0 1.0 1.0
```

### Step 4

To change the goal open a new terminal tab and call the `/setgoal` service with x, y and z as arguments. **Important: ** If you call the service with at least one negative input argument, you have to add ` -- ` before setting the argument
```
rosservice call /crazyflie/setgoal x y z
```

Example:
```
rosservice call /crazyflie/setgoal -- -1.0 1.0 1.0
```
The position can be changed by simply calling the `/setgoal` service again.

### Step 5

Stop hovering by pressing any key.

## Hardware notes

### Radio addresses
To change the radio address of the Crazyflie, use the CFclient (easiest through the [virtual machine](https://www.bitcraze.io/getting-started-with-the-crazyflie-2-0/#inst-comp) to avoid installing a lot of dependencies). Instructions [here](https://wiki.bitcraze.io/doc:crazyflie:client:pycfclient:index#firmware_configuration). Go to firmware configuration section.

Crazyflie 1 URI:
`radio://0/80/2M`

Crazyflie 2 URI:
`radio://0/125/2M`

It might be possible to change the config file in `crazyflie-firmware/src/config/config.h`.
We haven't tried this though.
