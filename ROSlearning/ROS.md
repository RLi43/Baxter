# ROS入门

每个命令的更多用法见 --h 获取帮助

## ros文件系统

$ROS_PACKAGE_PATH

`rospack` `find [package]`

`roscd [package]`/`rosls`

`rosed [package] [file]`(使用`vim`,配置在～/.zshrc文件中，EDITOR='gedit')

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

#### Parameter sever

ROS参数服务器在运行时可以存储和取回参数、使用静态数据作为配置成参数，通常定义在launch文件火独立的YAML文件中。

`rosparam`

set & get

##### C++ API

```C++
ros::NodeHandle nodeHandle("~");
std::string topic;
if(!nodeHandle.getParam("XX/XX/parameter_name",topic)){
    ROS_ERROR("Could not find topic parameter!");
}
```



### topic

`rostopic`

publish & subscribe messages

`rostopic type XXX`

#### Message

消息(msg): msg文件就是一个描述ROS中所使用消息类型的简单文本。它们会被用来生成不同语言的源代码。 msg文件存放在package的msg目录下，srv文件则存放在srv目录下。 msg文件实际上就是每行声明一个数据类型和变量名。

###### 可用数据类型

```c++
int8, int16, int32, int64 (plus uint*)
float32, float64
string
time, duration
other msg files
variable-length array[] and fixed-length array[C]
Header，它含有时间戳和坐标系信息。
```



`rosmsg` ros消息

`rostopic pub /topic_name msg_type args`

e.g.  `rostopic pub -r 1 /turtles/cmd_dev gem/Twist '[2.0,0.0,0.0]' '[0.0,0.0,2.0]'` -r for repeat at 1Hz; -1 for immediately

##### 创建消息

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



#### Service

服务(srv): 一个srv文件描述一项服务。它包含两个部分：请求和响应。

```c++
int64 A //请求
int64 B
---//分割符
int64 Sum //响应
```



##### 





### Service
`rosservice:当前运行的服务`

`rossrv:所有服务`



## Others

### rqt

logging framework

使用`rqt`打开rqt服务窗口选择，也可以用`rosrun rqt_XX rqt_XX`来运行服务

# referrence

https://www.shiyanlou.com/courses/854