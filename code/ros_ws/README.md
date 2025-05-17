# Setup


1. **Clone the repository**  
```bash
git clone https://git.las.iastate.edu/SeniorDesignComS/2025s/402c/sd15_reel-steel.git sd15_reel-steel
cd sd15_reel-steel
```
2. **Set up a Python virtual environment**

```bash
python -m venv .venv
# Linux/Mac
source .venv/bin/activate  
# Windows
.\.venv\Scripts\activate
Install Python dependencies
```
3. Install Python dependencies
```bash
pip install -r requirements.txt
```
4. **Build the ROS workspace**
```bash
cd ros_ws
catkin_make
# Linux/Mac
source devel/setup.bash  
# Add to ~/.bashrc for persistence
```

# How to run
## Using roslaunch
### Main
```
roslaunch reel_steel main.launch
```
For running production version

### Debug
```
roslaunch reel_steel debug.launch
```
For running graph visualization 

### Replay
```
roslaunch reel_steel replay.launch
```