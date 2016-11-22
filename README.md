# Crazyflie Project
## Setup
Clone into your catkin workspace

```
cd ~/catkin_ws/src
```
```
git clone https://github.com/jolilj/el2425_bitcraze
```
## Structure
The project includes the anchor position file *anchor_pos.yaml*. This needs to be copied to the *lps-ros* package.
```
cd ~/catkin_ws/src
```
```
cp el2425_bitcraze/anchor_pos.yaml lps-ros/data/anchor_pos.yaml
```
## Hovering
In order to hover you need to update the crazyflie firmware config. This is done by creating a config.mk file and place it inside `crazyflie-firmware/tools/make`. The file is now in this repo and consists only of two lines. The reason is to enable the kalman filter onboard the crazyflie. See [bitcraze wiki](https://wiki.bitcraze.io/doc:lps:index) for more information.

### Step 1
Assuming crazyflie channel is CH (Either **80** or **125** at the moment) and bitrate is RATE(**2M** at the moment) and goal position is ``x0, y0, z0``

```roslaunch bitcraze_lps_estimator dwm_loc_ekf_hover.launch uri:=radio://0/CH/RATE x:=x0 y:=y0 z:=z0```

### Step 2
Wait for a while for the filter to converge, then check the position(in another terminal tab)
```rostopic echo /crazyflie/crazyflie_position```
### Step 3
Open new terminal tab and call takeoff service
```rosservice call /crazyflie/takeoff```
or run hover.py
```rosrun el2425_bitcraze hover.py```

### Step 4
Stop hovering by stopping first script

## Hardware notes
### Radio addresses
To change the radio address of the Crazyflie, use the CFclient (easiest through the [virtual machine](https://www.bitcraze.io/getting-started-with-the-crazyflie-2-0/#inst-comp) to avoid installing a lot of dependencies). Instructions [here](https://wiki.bitcraze.io/doc:crazyflie:client:pycfclient:index#firmware_configuration). Go to firmware configuration section.

Crazyflie 1 URI:
`radio://0/80/2M`

Crazyflie 2 URI:
`radio://0/125/2M`

It might be possible to change the config file in *crazyflie-firmware/src/config/config.h*. We haven't tried this though.
