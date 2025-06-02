# Schunk PG70 ROS Package
## _ROS Control package for the schunk pg70_
Control package of a schunk pg70, comprhensive of services, action servers and urdf description. The package is been test in melodic and noetic version of ros.

## Install
To control the package is required, firstly to add the User to the dialout group to have the permission to access the serial device.
```
$ usermod -a -G dialout USER_NAME
```
The package to work is required to use the [Serial Library](http://wiki.ros.org/serial), available to be installed in ros-kinetic and ros-melodic. In alternative for further version of ROS, the library can be downloaded from github and installed following his instruction. To install the package with ros use:
```
$ sudo apt-get install ros-YOUR_ROS_VERSION-serial
```

The instruction's list to control the gripper can be found in the [schunk](https://schunk.com/fileadmin/pim/docs/IM0010976.PDF) website, where is possible to find further instruction to control the gripper and information in case of the error.

The control of the gripper can be activated with:
```
$ roslaunch schunk_pg70 pg70_control.launch
```

### Service
List of available Service:
 - schunk_pg70/reference: Initialize the gripper **!Be Aware that the gripper wil move once the service is called!**
 - schunk_pg70/set_pvac: Grasp service
 - schunk_pg70/get_error: Get the current error, use the manual to check the correspondents error code
 - schunk_pg70/get_position: Get the gripper position in mm
 - schunk_pg70/move_time 
 - schunk_pg70/move_time_loop
 - schunk_pg70/get_current: Get the setted current
 - schunk_pg70/acknowledge_error": Move the error to the state "acknowledge"
 - schunk_pg70/stop": Request the block of the gripper

The *Set* service return true in case the service is been accepted and not when is been completed, in that case an action must be used.

### Action Server
List of available Action Server:
 - schunk_pg70/Grasp: Action server used for a controlled grasp with a certain force by imposing a current
 - schunk_pg70/Move: Action server to move the fingers to a given position with a certain velocity


------------------------------------------------------------------------------------------------

## Troubleshooting
When an error arise, an error flag will be raised. The gripper will remanin in an Errror status untile the acknowledge error is called by using the proper service. After that should be used the service to get new reference, be aware that after the call the robot will move to the "0" width, in order to avoid damage remove any possible obstacle between the fingers and eventually the fingers itself.
```
$ rosservice call /schunk_pg70/acknowledge_error'
$ rosservice call /schunk_pg70/reference "{}" 
```
After that the gripper should be ready again to operate correclty, however is always better to **check the manual!**

------------------------------------------------------------------------------


#### Usefull links 
Links to Github repo and ros.wiki pages.
https://github.com/SmartRoboticSystems/schunk_grippers
http://wiki.ros.org/schunk_pg70
http://wiki.ros.org/rosserial

