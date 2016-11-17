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

## Hardware notes
To change the radio address of the Crazyflie, use the CFclient (easiest through the [virtual machine](https://www.bitcraze.io/getting-started-with-the-crazyflie-2-0/#inst-comp) to avoid installing a lot of dependencies). Instructions [here](https://wiki.bitcraze.io/doc:crazyflie:client:pycfclient:index#firmware_configuration). Go to firmware configuration section.

It might be possible to change the config file in *crazyflie-firmware/src/config/config.h*. We haven't tried this though.
