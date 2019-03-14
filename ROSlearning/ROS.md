# ROS入门

每个命令的更多用法见 --h 获取帮助

## Sturcture

### Node

`rosnode `

`roscore`是主节点，每个节点启动时需要向主节点registration

`rosrun [package] [node]`

#### roslaunch

`roslaunch [package] [file.launch]`启动文件中多个节点

`roslaunch`会自动启动`roscore`

.launch xml文件

```shell
   <launch>
#参数
	<arg name="arg1" default="true"/>
	<arg name="world_name" default="new_world"/>
	#use in .launch:
	#$(arg arg_name) 
	#set arg like:
	#roslaunch XXX.launch arg1:=false world_name:=myworld
#include
	<include file="package_name"/>
	<include file="$(find package_name)/XXX/XXX"/>
	#传参
	<arg name="arg_in_XXX" value="$(arg arg1)"/>
	
      <group ns="turtlesim1"> #ns:namespace
        <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
      </group>

      <group ns="turtlesim2">
        <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
      </group>

      <node pkg="turtlesim" name="mimic" type="mimic">
      #mimic 模仿节点
        <remap from="input" to="turtlesim1/turtle1"/>
        <remap from="output" to="turtlesim2/turtle1"/>
      </node>

    </launch>
```



### topic

`rostopic`

publish & subscribe messages

`rostopic type XXX`

#### ros message

`rosmsg` ros消息

`rostopic pub /topic_name msg_type args`

e.g.  `rostopic pub -r 1 /turtles/cmd_dev gem/Twist '[2.0,0.0,0.0]' '[0.0,0.0,2.0]'` -r for repeat at 1Hz; -1 for immediately

#### 创建消息

`WORKSPACE/src/YOUR_PACKAGE/msg/MSG_NAME.msg`

```
string first_name
string last_name
uint8 age
uint32 score
```

在`package.xml`中必须包含

```xml
<build_depend>message_generation</build_depend>
  <run_depend>message_runtime</run_depend>
```

在`CMakelist.txt`增加生成消息服务的依赖

```xml
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation)
```

增加运行依赖

```xml
catkin_package(
  ...
  CATKIN_DEPENDS message_runtime ...
  ...)
```

声明消息类型

```xml
add_message_files(
   FILES
   Message1.msg
   Message2.msg
 )
```







### Service

`rosservice:当前运行的服务`

`rossrv:所有服务`

### Parameter

`rosparam`

set & get

## rqt

logging framework

使用`rqt`打开rqt服务窗口选择，也可以用`rosrun rqt_XX rqt_XX`来运行服务

# referrence

https://www.shiyanlou.com/courses/854