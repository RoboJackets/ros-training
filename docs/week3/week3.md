# What are we doing today?

-   Encoders
-   launch files
-   Gazebo
    -   urdf


# Encoders

<div class="NOTES">
keep it high level. Do an example of ticks

</div>

-   used to determine the speed of a spinning wheel
    -   measure number of ticks during a time step
-   2 main types
    -   optical
    -   magnetic


## Optical Encoders

<div class="NOTES">
sensitive to dust and broken disks

</div>

-   uses a light and a glass wheel to measure ticks
-   generally a high tick count
-   used by IGVC and RoboRacing

![img](https://i.imgur.com/d5Rx7nQ.jpg)


## Magnetic Encoders

<div class="NOTES">
uses the hall effect

</div>

-   uses magnets to measure ticks
-   generally lower tick count
-   requires occasional tuning

![img](https://automation-insights.blog/wp-content/uploads/2015/09/bml-evalkit.jpg)


# Update your local fork

```shell
git pull
```


# Launch Files

<div class="NOTES">
mention how it can be more human understandable. Just a shorthand for rosrun

</div>

-   XML files
-   a way to launch multiple ros nodes
-   recursive


# Why Launch Files

```shell
rosrun igvc mapper ekf/base_link_frame=base_footprint mapper/topics=
/scan/pointcloud /usb_cam_center/line_cloud /usb_cam_left/line_cloud
max_correspondence_distance=0.1 max_iterations=30 search_radius=0.03
```


# Commandline

<div class="NOTES">
make sure to mention tab complete

</div>

```shell
roslaunch igvc mapper.launch
```

```shell
roslaunch [PACKAGE_NAME] FILE_NAME.launch
```


# Structure

-   everything is encompassed in the launch tag

```XML
<launch>
  <!-- here -->
</launch>
```


# Valid Tags

<div class="NOTES">
mention that these are the tags the go under the launch tag

</div>

| tag        | usage                                                           |
|---------- |--------------------------------------------------------------- |
| `node`     | launches a node                                                 |
| `param`    | sets up a parameter for that node                               |
| `remap`    | remap a topic name to something else                            |
| `rosparam` | enables the use of YAML files for setting lots of parameters    |
| `include`  | includes and launches other roslaunch files                     |
| `env`      | specify and environment variable                                |
| `arg`      | sets a variable that can be used in the rest of the launch file |
| `group`    | allows you to group nodes in a single launch file               |


# <node>

| tag    | usage                                     |
|------ |----------------------------------------- |
| `name` | the name given to that instance of a node |
| `pkg`  | the package that the node is in           |
| `type` | The package to launch                     |
| `args` | arguments to pass to node                 |

```XML
<launch>
  <node name="NODE_NAME" pkg="PKG_NAME" type="NODE_TYPE" .../>

  <node name="NODE_NAME" pkg="PKG_NAME" type="NODE_TYPE">
    <!-- tags local to this node -->
  </node>
</launch>
```


# <param>

| tag     | usage                      |
|------- |-------------------------- |
| `name`  | the name of the parameter  |
| `type`  | the variable type          |
| `value` | the value of the parameter |

```XML
<launch>
  <node name="NODE_NAME" pkg="PKG_NAME" type="NODE_TYPE">
    <param name="PARAM_NAME" type="VAR_TYPE" value="VALUE">
  </node>
</launch>
```


# Substitution arguments

-   $(env `ENV_VAR`)
    -   will set the value from environment variable
    -   will fail if `ENV_VAR` is not set
-   $(opentv `ENV_VAR` `default_value`)
    -   will set the value from environment variable if set
    -   will set default if `ENV_VAR` is not defined
-   $(find `PKG_NAME`)
    -   used for relative paths to files in packages
-   $(arg `ARG_NAME`)
    -   will set to `ARG_NAME` if it is defined in file


# Useful Commandline Arguments

| option          | effect                                               |
|--------------- |---------------------------------------------------- |
| `--screen`      | force node output to screen                          |
| `--dump-params` | print parameters in yaml format                      |
| `--nodes`       | prints the nodes launched by this launch file        |
| `--args`        | prints the command line arguments set in launch file |


# Gazebo

-   The simulator used by both RoboRacing and IGVC

![img](https://upload.wikimedia.org/wikipedia/en/thumb/1/13/Gazebo_logo.svg/1024px-Gazebo_logo.svg.png)


# Simulation

<div class="NOTES">
talk about how it crashes. Also that we can simulate friction, mass, etc

</div>

-   simulators are wonderful
-   allow to test code in somewhat real life situations
-   not a ROS product
    -   interacts with ROS through plugins that publish to topics
    -   these plugins are not perfect so it crashes **a lot**


# Package Structure

-   `TEAM_NAME_Description`
    -   contains the URDF file
    -   defines the world
-   `TEAM_NAME_Control`
    -   contains the nodes that control the environment


# URDF

-   unified robot description format
-   this is what gazebo uses to generate the robot
-   XML file format
-   defines everything in a 3-dimensional grid


# Links

<div class="NOTES">
take about what each is and how meshes can be used as geometries. visual is required

</div>

-   links contain
    -   required for ROS
        -   visual geometry
    -   required for gazebo
        -   collision geometry
        -   intertial geometry


# Links Basic Example

<div class="NOTES">
make sure to launch rviz with this urdf

</div>

```XML
<robot>
  <link name="base_link">
    <visual>
      <geometry>
	<box size="0.8 0.3 0.1"/>
      </geometry>
    </visual>
  </link>
</robot>
```


# Gazebo Link

<div class="NOTES">
launch this in gazebo

</div>

```XML
<link name="body">
  <inertial>
    <origin xyz="0 0 0" />
    <mass value="50.0" />
    <inertia  ixx="0.0" ixy="0.0"  ixz="1.0"  iyy="0.0"  iyz="0.0"  izz="0.0" />
  </inertial>
  <visual>
    <origin rpy="0 0 0" xyz="0 0 0"/>
    <geometry>
      <mesh filename="model://urdf/meshes/Body.dae"/>
    </geometry>
  </visual>
  <collision>
    <geometry>
      <mesh filename="model://urdf/meshes/Body.dae"/>
    </geometry>
  </collision>
</link>
```


# Joints

-   links can be connected using joints
    -   all joints have a parents and a child
-   all positions are realtive to its parents
    -   entire tree should have a single root


# I AM ROOT

![img](https://www.syfy.com/sites/syfy/files/wire/legacy/groot_0.jpg)


# Joint Types

| type         | usage                                      |
|------------ |------------------------------------------ |
| `continuous` | rotates in an axis and has to limits       |
| `fixed`      | does not move                              |
| `floating`   | 6 degrees of freedom                       |
| `revolute`   | rotates on an axis and has rotation limits |


# Example Joint

```XML
<joint name="base_link_to_left_wheel" type="continuous">
  <parent link="base_link"/>
  <child link="left_wheel"/>
  <axis xyz="0 0 -1" rpy="0 0 0" />
  <origin xyz="0 0 0" rpy="0 0 0" />
</joint>
```


# Useful Commands

<div class="NOTES">
explain the output

</div>

-   `check_urdf`
    -   "compiles" the urdf and gives a printout of the tf tree

```BASH
robot name is: hal
---------- Successfully Parsed XML ---------------
root Link: base_footprint has 4 child(ren)
  child(1):  base_link
  child(2):  body
    child(1):  back_ball
    child(2):  left_wheel
    child(3):  right_wheel
  child(3):  usb_cam_center
    child(1):  optical_cam_center
  child(4):  lidar
```


# Introducing HAL

![img](https://i.imgur.com/IGlRSWv.png)