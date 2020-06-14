# slam_nav

## Getting Started
```
cd PATH_TO_CATKIN_WS/src
git clone https://github.com/trns1997/slam_nav.git
cd ..
catkin_make 
```
Make sure to source the catkin_ws. Check if the `slam_nav` package is properly setup by trying:
```
roscd slam_nav/
```

## Launching Gmapping using Visual Odom
*Terminal 1*
```
roslaunch slam_nav build_map_vo.launch 
```
*Terminal 2*
```
rosbag play --clock PATH_TO_CATKIN_WS/src/slam_nav/data/2020-05-14-16-09-36-traj1-os1-t265-pix.bag 
```
The Result should look something like this:

<img src= https://github.com/trns1997/slam_nav/blob/master/media/map.gif/>

Once the map building process is complete:

*Terminal 3*
```
rosrun map_server map_saver -f PATH_TO_CATKIN_WS/src/slam_nav/maps/NAME_OF_MAP_FILE

Example:
rosrun map_server map_saver -f PATH_TO_CATKIN_WS/src/slam_nav/maps/built_map_09_bag 
```
If Launching Gmapping using Optitrack Odom:
```
roslaunch slam_nav build_map_o.launch 
```

## Launching AMCL using Visual Odom
*Terminal 1*
```
roslaunch slam_nav nav_map.launch map_file:= PATH_TO_CATKIN_WS/src/slam_nav/maps/NAME_OF_MAP_FILE.yaml

Example:
roslaunch slam_nav nav_map.launch map_file:= PATH_TO_CATKIN_WS/src/slam_nav/maps/built_map_09.yaml
```
If no `map_file` is spcified then it defaults to `PATH_TO_CATKIN_WS/src/slam_nav/maps/built_map_09.yaml`

*Terminal 2*
```
rosbag play --clock PATH_TO_CATKIN_WS/src/slam_nav/data/2020-05-14-16-14-37-traj2-os1-t265-pix.bag
```
The Result should look something like this:

<img src= https://github.com/trns1997/slam_nav/blob/master/media/loc.gif/>

