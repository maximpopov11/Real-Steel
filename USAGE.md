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
rosluanch reel_steel main.launch
```

#### Run Debug Version
```
rosluanch reel_steel main.launch
```

#### Run Main
```
rosluanch reel_steel main.launch
```

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

