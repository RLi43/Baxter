# Baxter 机械控制

APIwiki http://api.rethinkrobotics.com/baxter_interface/html/index.html

## 开始

```python
# Import the necessary Python modules
# rospy - ROS Python API
import rospy 
# baxter_interface - Baxter Python API
import baxter_interface
# initialize our ROS node, registering it with the Master
rospy.init_node('XXXXX')
```

## 手臂

`from baxter_interface import Limb`

详细信息：http://api.rethinkrobotics.com/baxter_interface/html/baxter_interface.limb.Limb-class.html

手臂类

```Python
right_arm = Limb('right')
left_arm = Limb('left') 

#关节名称
[str]	joint_names(self)
#关节角度 单个
float	joint_angle(self, joint)
#关节角度 全部
dict({str:float})	joint_angles(self)
#关节速度 单个关节
float	joint_velocity(self, joint)
#关节速度 全部
dict({str:float})	joint_velocities(self)
#关节 
float	joint_effort(self, joint)
#
dict({str:float})	joint_efforts(self)
#手臂末端位置  {position, orientation}
dict({str:Limb.Point,str:Limb.Quaternion})	endpoint_pose(self)
#手臂末端速度 {linear, angular}
dict({str:Limb.Point,str:Limb.Point})	endpoint_velocity(self)
#手臂 {force, torque}
dict({str:Limb.Point,str:Limb.Point})	endpoint_effort(self)
 	
set_command_timeout(self, timeout)
 	
exit_control_mode(self, timeout=0.2)
Clean exit from advanced control modes (joint torque or velocity).	
 	
set_joint_position_speed(self, speed)
Set ratio of max joint speed to use during joint position moves.	source code
 	
set_joint_positions(self, positions, raw=False)#不知道为什么不能用
Commands the joints of this limb to the specified positions.	
IMPORTANT: 'raw' joint position control mode allows for commanding joint positions, without modification, directly to the JCBs (Joint Controller Boards). While this results in more unaffected motions, 'raw' joint position control mode bypasses the safety system modifications (e.g. collision avoidance). Please use with caution.

 	
set_joint_velocities(self, velocities)
Commands the joints of this limb to the specified velocities.	source code
 	
set_joint_torques(self, torques)
Commands the joints of this limb to the specified torques.	source code
 	
move_to_neutral(self, timeout=15.0)
Command the joints to the center of their joint ranges	source code
#移动到指定位置	
move_to_joint_positions(self, positions, timeout=15.0, threshold=0.008726646)
(Blocking) Commands the limb to the provided positions.
```

limb中的参数

```python
Limb.joint_angles()			#返回当前状态角度
#包含的数据 #值是16位的
    {'right_s0': 0.0, 
     'right_s1': 0.0, 
     'right_w0': 0.0,
     'right_w1': 0.0, 
     'right_w2': 0.0,
     'right_e0': 0.0, 
     'right_e1': 0.0}
# 赋值示例
angles['right_s0']=0.0
#可以只改变一项
```

函数

```python
Limb.move_to_joint_positions(angles) #移动到指定角度状态
# 该函数会将angles中的'name'移动到'value'位置
# 关节命令示例
# angles = {'right_s0'：1.0}
# 名字不对会报错
```

## 爪子

```python
from baxter_interface import Gripper
right_gripper = Gripper('right')
left_gripper = Gripper('left')

	
stop(self, block=True, timeout=5.0)
#移动 0-100 	
command_position(self, position, block=False, timeout=5.0)
#吸 	
command_suction(self, block=False, timeout=5.0)

set_velocity(self, velocity)# 0=stop 100=max [50.0]
set_moving_force(self, force) #0=none 100=max [30.0]
set_holding_force(self, force)#0=none 100=max [30.0]
set_dead_band(self, dead_band)#% of full position [5.0]
```

## 头

http://api.rethinkrobotics.com/baxter_interface/html/baxter_interface.head.Head-class.html

```python
from baxter_interface import Head
<component name> = Head()
 	
#目前头朝向位置
float	pan(self)
#在点头？
bool	nodding(self)
#在旋转？
bool	panning(self)
#旋转到指定角度
set_pan(self, angle, speed=100, timeout=10.0)
#点头一次
command_nod(self, timeout=5.0)
```



## 导航器

就是手臂上的控制器

```python
from baxter_interface import Navigator
right_arm_navigator = Navigator('right')
left_arm_navigator = Navigator('left')
right_torso_navigator = Navigator('torso_right')
left_torso_navigator = Navigator('torso_left')
```

## 其他

http://sdk.rethinkrobotics.com/wiki/Baxter_Interface