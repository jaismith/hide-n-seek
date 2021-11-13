# hide-n-seek
Automated Object of Interest Detection, Mapping, Localization, and Obstacle Avoidance

### Mapping
- output: occupancy grid map, odom corrections (if needed), separate map with cell 'seen' state based on camera FOV
- input: lidar scan, odometry, camera FOV

### Navigation
- input: occupancy and seen grid, target coords (if found)
- output: polyline path for upcoming `n` points

### Movement
- input: desired movement (waypoints in map frame)
- output: `cmd_vel` commands incl. error correction

### Object Detection
- input: raw RGB image
- output: bounding box around object

## Launch

```sh
catkin_make
rosrun hide_n_seek <node_name>.py
```
