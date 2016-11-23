# Bitcraze Project Review W4
The first four weeks have been devoted to assembly of hardware followed by configuration and installation of packages. This is all in line with the proposed project plan. So far the idea has been for all group members to review all tasks for each week to get an understanding of the basics needed to progress into the more advanced tasks that are yet to come.

## Week 1
The first week was devoted to assembly the crazyflie hardware and read up on both the crazyflie (guides etc) and ROS in general. Also a project plan was formulated.
This was straight forward as Bitcraze provides great tutorials for the hardware setup.

## Week 2
This week was devoted to doing the necessary firmware updates in order to connect to both the crazyflies and the loco positioning system. There were some confusion wether to use the provided virtual machine from Bitcraze or to do it by USB. We ended up doing it over USB as this limited the overhead of installing a virtual machine including the bitcraze client as we would only need one of its many features

## Week 3
In week 3 things started to get really interesting. We got the wall mounts for the Loco Positioning System so that we could, in a proper way, mount the anchors in the room. There was, and is still, a lot of confusion about the sofware architecture within the different ros-packages. Namely the unoffical ROS package for the crazyflie, **crazyflie-ros**, developed by github user **whoenig** and the official ROS package for the Loco positioning system, **lps-ros**. Furtheremore we encountered problems with serious interference causing connection issues. This was solved by updating the radio frequency of the crazyflie transmitter. By the end of this week we had positioned the anchors and got relatively good position estimates of the crazyflie.

## Week 4
The goal of this week is for the crazyflie to hover steadily. We've managed that. However there are some investigations that need to be conducted for us to proceed and complete the goals of week 5.

* General dependencies between crazyflie-ros package and lps-ros package?
* What type of controller is actually running when using lps-ros package(with dwm_loc_ekf_hover.launch for example). (This is the setup we do now in order to hover)
*  Would it be better to get the crazyflie-ros controller up and running instead?
	* (This is through the launch file crazyflie2.launch)
* Why is there a bias in the z-direction? 
