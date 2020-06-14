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

## Launching Gmapping using Optitrack Odom
```
roslaunch slam_nav build_map_o.launch 
```

