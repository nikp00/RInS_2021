# HW_3

## Comands to run
```
roscore
roslaunch simulation_envoirment rins_world.launch
roslaunch simulation_envoirment amcl_simulation.launch
roslaunch turtlebot_rviz_launchers view_navigation.launch
roslaunch hw_3 simple_waypoints.launch
```

## Other settings
To add waypoints, eddit the /data/waypoints.json file
```
code $(rospack find hw_3)/data/waypoints.json
```