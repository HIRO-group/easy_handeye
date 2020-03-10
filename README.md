

Automatic Calibration implementation of easy_handeye: TF / VISP Hand-Eye Calibration
============================================ 

This is an edit to the [Easy Hand-Eye calibration program](https://github.com/IFL-CAMP/easy_handeye) to create an automatic calibration program for the HIRO Sawyer robot. 

This utilizes the eye-on-base implementation of the program using an Intel RealSense camera mounted on the ceiling above Sawyer.


### Installation:
This tutorial is picks up from the basic ROS setup outline in the HIRO wiki. If you have not yet setup your workspace using that [guide](https://hiro-group.ronc.one/new_member.html), please do so first!

These are the same basic steps for implementation of Easy Hand-Eye.

- clone this fork of the repository into your workspace - this fork is different than the master version of easy_handeye, so if you have the original version, you will need to delete that fersion first:
```
cd ~/catkin_ws/src  # replace with path to your workspace (if you followed the WIKI guide, this should be correct)
```
if you have the original version, remove it from your workspace first:
```
rm -r easy_handeye/
```
clone the HIRO fork of the repository:
```
git clone https://github.com/HIRO-group/easy_handeyee
```

satisfy dependencies
```
cd ..  # now we are inside ~/catkin_ws
rosdep install -iyr --from-paths src
```
installing other dependent packages (feel free to skip any packages you already have in your workspace - the only package in the program that is different from the master is the easy_handeye):
```
cd ~/catkin_ws/src # replace with path to your workspace (if you followed the WIKI guide, this should be correct)
git clone https://github.com/pal-robotics/aruco_ros   # aruco_ros package


```

- build
```
catkin build
```

### Calibration
The calibration is built on the principle of a 3D grid. The poses of the upper left and the lower right corners of the Intel RealSense's view are used to define the grid.

## New Camera Position
If this is the first time you are running the calibration or you have changed the position of the camera above the Sawyer, you will need to intialize the 

* Connect to the HIROLab_5G wifi network
* Plug the Intel RealSense camera cable into your machine
* Turn on Sawyer

For all terminal tabs used:
* Enter your intera workspace:
```
cd catkin_ws
./intera.sh
```
* Easy Hand-Eye runs on Python 2. To check your python version, run:
```
python
```
 If the version shows 3.X, you can set up a [temporary Python 2 environment](https://stackoverflow.com/questions/7237415/python-2-instead-of-python-3-as-the-temporary-default-python)
 
 
#### Launching 
* Launch the sawyer_realsense launch file:
```
cd src/easy_handeye/easy_handeye/launch/
roslaunch sawyer_realsense_handeyecalibration.launch 
```
This should launch Rviz Easy Hand-Eye Config and RQT Easy Hand-Eye Perspective. These are the GUIs used to perform a manual calibration with Easy Hand-Eye.

#### Intel RealSense Camera Image
* Open another terminal tab (making sure to enter into the intera workspace and run Python 2) and show the Intel RealSense camera image:
```
rosrun image_view image_view image:=/aruco_tracker/result 
```
#### Run the Automatic Calibration
* Navigate to the automatic calibration and run the script:
```
cd easy_handeye/rqt_easy_handeye/src/rqt_easy_handeye/
./automatic_calibration.py 
```
