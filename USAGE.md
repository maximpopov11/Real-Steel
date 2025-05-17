# Usage Guide

## Roslaunch

### What is roslaunch?

`roslaunch` is a tool that allows you to:
- Launch multiple ROS nodes simultaneously
- Set parameters on the ROS parameter server
- Automatically respawn processes that die
- Configure node namespaces, remappings, and more
- Manage complex multi-node systems efficiently

### How to Use roslaunch

#### Run Main Version
```
roslaunch reel_steel main.launch
```

#### Run Debug Version
```
roslaunch reel_steel debug.launch
```

#### Replay
```
roslaunch reel_steel replay.launch
```

### Run

### Understanding Launch File Output

When you run a launch file, you'll see:

1. A list of all nodes being launched
2. Node initialization messages
3. Topics being published/subscribed
4. Any errors or warnings

The output is color-coded:
- Green: Info messages
- Yellow: Warnings
- Red: Errors

## Running Individual Nodes

### When to Run Individual Nodes

You might run individual nodes when:
- Debugging specific components
- Testing new functionality
- Running a minimal subset of the system
- Custom configuration needs

### How to Run Individual Nodes

Individual nodes can be run using the `rosrun` command. In order to run the whole pipeline without the launch script, these nodes much all be run simultaneously:

#### Landmarks
```
rosrun landmarks main.py
```
In order to debug the project's output, it is often useful to run the landmarks node with the `-combined` flag -- this will also publish the `/raw` and `/preprocessed` topics, intermediate steps in the node's preprocessing. The `/scaled` topic is always published to facilitate the next step in point production.

#### graph_points (Debug)
```
# Launches raw pose point graphing node
rosrun graph_points raw.py

# Launches preprocessed (smoothed and translated) point graphing node
rosrun graph_points preprocessed.py

# Launches scaled (scaled to robot proportions) point graphing node
rosrun graph_points scaled.py
```
Additionally, there is a launch file that launches all three nodes side-by-side:
```
roslaunch graph_points graph_points.launch
```

#### MoveIt Services
```
roslaunch g1_moveit_config demo.launch
```
#### Kinematics
```
rosrun kinematics main.py
```

#### Simulator (Debug)
```
rosrun simulator main.py
```

#### Robot_CSV
```
rosrun kinematics robot_csv.py
```
