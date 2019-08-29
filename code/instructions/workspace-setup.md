# Setting up a catkin workspace

## Create a ROS workspace
Create a catkin workspace:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

The catkin_make command is a convenience tool for working with catkin workspaces.
Running it the first time in your workspace, it will create a CMakeLists.txt link in your 'src' folder.
Additionally, if you look in your current directory you should now have a 'build' and 'devel' folder.
Inside the 'devel' folder you can see that there are now several setup.*sh files.
Sourcing any of these files will overlay this workspace on top of your environment.
Before continuing source your new setup.*sh file: 
```bash
source devel/setup.bash
```

To make sure your workspace is properly overlayed by the setup script,
make sure ROS_PACKAGE_PATH environment variable includes the directory you're in. 
```bash
echo $ROS_PACKAGE_PATH
# Result should be: /home/youruser/catkin_ws/src:/opt/ros/kinetic/share
```

Now that you've setup a catkin workspace, we can proceed with the rest of [introduction.md](introduction.md)
