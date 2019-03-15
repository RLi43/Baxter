## 消息发布和订阅器

在scripts文件夹中写脚本

### 发布

```python
#!/usr/bin/env python #所有ROS node都需要这个声明 保证作为Python程序执行
# license removed for brevity
import rospy	#使用Python写ros节点必须调用
from std_msgs.msg import String	#std_msg里有输出显示需要的string类型

def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    # 定义接口 即消息类型为String，主题为chatter 队列数量在hydro版本里
    rospy.init_node('talker', anonymous=True)
    # 节点声明 必须声明，不然无法与ROS Master交流
    # anonymous 节点名字='talker'+随机数字
    rate = rospy.Rate(10) # 10hz
    # 循环频率 只要处理速度没有超过0.1s
    while not rospy.is_shutdown():	#检查是否退出
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)	#loginfo会1.显示在屏幕上
        #2.写在这个node的log file里 3.写在rosout里，对debug很有用
        pub.publish(hello_str)	#在主题上发布消息
        rate.sleep()	#直到下一个循环时间开始
    # 以上是一个很典型的rospy结构

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        # 意外/Ctrl+C 退出 会有来自sleep()的异常
        pass
```

### 订阅器

```python
#!/usr/bin/env python
import rospy
from std_msgs.msg import String

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("chatter", String, callback)

    # spin() simply keeps python from exiting until this node is stopped 而且它拥有自己的线程，不受回调函数影响
    rospy.spin()

if __name__ == '__main__':
    listener()
```

### 编译构建节点

catkin_make

## 服务和客户端

### 服务节点

使文件可执行

```
chmod +x scripts/add_two_ints_server.py
```

```python
#!/usr/bin/env python

from beginner_tutorials.srv import *
import rospy

def handle_add_two_ints(req):
    print "Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b))
    return AddTwoIntsResponse(req.a + req.b)

def add_two_ints_server():
    rospy.init_node('add_two_ints_server')
    s = rospy.Service('add_two_ints', AddTwoInts, handle_add_two_ints) #声明一个服务，其名字为'add_two_ints'，其服务类型为AddTwoInts，所有请求都被送到handle_add_two_ints这个函数
    print "Ready to add two ints."
    rospy.spin()#spin函数保持服务在运行中，直到被结束

if __name__ == "__main__":
    add_two_ints_server()
```

### 客户端节点

```python
#!/usr/bin/env python

import sys
import rospy
from beginner_tutorials.srv import *

def add_two_ints_client(x, y):
    rospy.wait_for_service('add_two_ints')#客户端不需要新建节点，直接调用服务
    try:	#如果调用失败可能会抛出异常
        add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)        # 直接调用函数 服务名，服务类型
        resp1 = add_two_ints(x, y)
        return resp1.sum #AddTwoIntsResponse对象 也就是服务类型定义的下面的数据类型
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

def usage():
    return "%s [x y]"%sys.argv[0]

if __name__ == "__main__":
    if len(sys.argv) == 3:
        x = int(sys.argv[1])
        y = int(sys.argv[2])
    else:
        print usage()
        sys.exit(1)
    print "Requesting %s+%s"%(x, y)
    print "%s + %s = %s"%(x, y, add_two_ints_client(x, y))
```

## 录制与回放

http://wiki.ros.org/cn/ROS/Tutorials/Recording%20and%20playing%20back%20data

`rosbag`