# Baxter 仿真使用说明

## 1.安装

### 1.1依赖安装

```
$ sudo apt-get install gazebo2 ros-indigo-qt-build ros-indigo-driver-common ros-indigo-gazebo-ros-control ros-indigo-gazebo-ros-pkgs ros-indigo-ros-control ros-indigo- control-toolbox ros-indigo-realtime-tools ros-indigo-ros-controllers ros-indigo-xacro python-wstool ros-indigo-tf-conversions ros-indigo-kdl-parser
```

### 1.2Baxter模拟器安装

1.安装baxter_simulator

```
$ cd ~/ros_ws/src
$ wstool init .
$ wstool merge https://raw.githubusercontent.com/RethinkRobotics/baxter_simulator/master/baxter_simulator.rosinstall
$ wstool update
```

2.建立资源

```
$ source /opt/ros/indigo/setup.bash
$ cd ~/ros_ws
$ catkin_make
$ catkin_make install # 有的教程里加了这句 我应该都执行了不知道不执行会怎么样 \捂脸
# 下面是修改ip地址 可以用geidt或者vim或者vi
$ cp src/baxter/baxter.sh . 
```

注释：可能有些语句执行会报错……但是好像没什么关系

3.一个bug的解决

应该是gazebo升级之后把自己的资源放的位置改了导致的错误。目前的解决方法是直接下载文件。

下载https://bitbucket.org/osrf/gazebo_models/downloads的文件，解压到``~/.gazebo/models``中（需要新建文件夹，并且是将内容解压到该文件夹，而非还套着一个文件夹）



## 2.使用

运行命令

```
$ ./baxter.sh sim #开启仿真环境
$ roslaunch baxter_gazebo baxter_world.launch #打开gazebo世界
```

等待打开，应该要加载一会。

官网提示要看到以下代码：

```
[ INFO] [1400513321.531488283, 34.216000000]: Simulator is loaded and started successfully
[ INFO] [1400513321.535040726, 34.219000000]: Robot is disabled
[ INFO] [1400513321.535125386, 34.220000000]: Gravity compensation was turned off
```

但是实际上输出的信息非常多，很容易淹没了。大概的判断就是全部界面（一个Gazebo窗口，一个控制器（就是手臂上的那个）的模拟器）都出来了，语句停止在

``` 
[ INFO] [1541658565.449163942, 595.950000000]: left_joint_position_controller was started and left_joint_velocity_controller and left_joint_effort_controller were stopped succesfully
```

应该就可以了。

然后**再**打开一个终端，进入仿真环境，就可以像实际使用一样来操控仿真的机器人了。

```
rosrun baxter_tools enable_robot.py -e #使能机器人
```



## 参考目录

* 官网教程 http://sdk.rethinkrobotics.com/wiki/Baxter_Simulator
* 参考教程 https://blog.csdn.net/yangchao_emigmo/article/details/52518064
* gazebo的bug讨论 http://answers.ros.org/question/199401/problem-with-indigo-and-gazebo-22/>
* https://blog.csdn.net/wxflamy/article/details/79608080 启动文件解释

```html
<?xml version="1.0" encoding="utf-8"?>
<launch>

  <!-- 这是一些控制选项, for example paused:=true -->
  <arg name="paused" default="false"/>
  <arg name="use_sim_time" default="true"/>
  <arg name="gui" default="true"/>
  <arg name="headless" default="false"/>
  <arg name="debug" default="false"/>

  <!-- 这部分内容加载电子抓手, for example left_electric_gripper:=true -->
  <arg name="left_electric_gripper" default="true"/>
  <arg name="right_electric_gripper" default="true"/>

  <!-- 向ROS参数服务器中加载URDF模型文件 -->
  <!-- This xacro will pull in baxter_base.urdf.xacro, left_end_effector.urdf.xacro,
                                           and right_end_effector.urdf.xacro
       Note: if you set this to false, you MUST have set the robot_description prior
             to launching baxter_world -->
  <arg name="load_robot_description" default="true"/>
  <param if="$(arg load_robot_description)" name="robot_description"
      command="$(find xacro)/xacro.py --inorder $(find baxter_description)/urdf/baxter.urdf.xacro gazebo:=true"/>

  <!--加载启动的仿真环境 We resume the logic in empty_world.launch, changing the name of the world to be launched -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="world_name" value="$(find baxter_gazebo)/worlds/baxter.world"/>
    <arg name="debug" value="$(arg debug)" />
    <arg name="gui" value="$(arg gui)" />
    <arg name="paused" value="$(arg paused)"/>
    <arg name="use_sim_time" value="$(arg use_sim_time)"/>
    <arg name="headless" value="$(arg headless)"/>
  </include>

  <!-- 在ROS参数服务器中加载软件版本 Load the software version into the ROS Parameter Server -->
  <param name="rethink/software_version" value="1.2.0" />

  <!-- 发布世界坐标系与机器人基座之间的静态变换 Publish a static transform between the world and the base of the robot -->
  <node pkg="tf2_ros" type="static_transform_publisher" name="base_to_world" args="0 0 0 0 0 0 1 world base" />

  <!--运行一个python脚本，发送服务启动urdf机器人 Run a python script to the send a service call to gazebo_ros to spawn a URDF robot -->
   <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen"
    args="-param robot_description -urdf -z 0.93 -model baxter
          -J baxter::right_s0 -0.272659
          -J baxter::right_s1 1.04701
          -J baxter::right_e0 -0.00123203
          -J baxter::right_e1 0.49262
          -J baxter::right_w0 -0.0806423
          -J baxter::right_w1 -0.0620532
          -J baxter::right_w2 0.0265941
          -J baxter::left_s0 0.192483
          -J baxter::left_s1 1.047
          -J baxter::left_e0 0.000806359
          -J baxter::left_e1 0.491094
          -J baxter::left_w0 -0.178079
          -J baxter::left_w1 -0.0610333
          -J baxter::left_w2 -0.0124707" />

  <!-- ros_control baxter launch file -->
  <include file="$(find baxter_sim_hardware)/launch/baxter_sdk_control.launch">
      <arg name="right_electric_gripper" value="$(arg right_electric_gripper)"/>
      <arg name="left_electric_gripper" value="$(arg left_electric_gripper)"/>
  </include>

</launch>
```

