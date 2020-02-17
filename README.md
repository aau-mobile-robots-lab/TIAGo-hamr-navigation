# TIAGo human-aware navigation
This repository was developed for a TIAGo robot as part of a Master Thesis procejt at Aalborg University created by Balazs Reiser, Martin Bieber Jensen and Simon Nanoq Callisen

The repository is structured according to general ROS practices, see e.g. https://github.com/leggedrobotics/ros_best_practices/wiki

## Clone and Build
### Install necessary tools
```bash
sudo apt-get install python-catkin-tools
sudo apt-get install python-rosdep
```

### Cloning
To set up the simulation environment you need to clone the necessary repositories into an existing or new catkin workspace.
Follow the steps below to set up a new catkin workspace and clone:
```bash
mkdir -p ~/tiago_aau_ws/src
cd ~/tiago_aau_ws/src
catkin_init_workspace
git clone https://github.com/rejzi/TIAGo-Human-Aware-Navigation.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
```

### Building
Build the project with catkin build
```bash
cd ~/tiago_aau_ws
catkin build
source devel/setup.bash
```

