## 消息发布和订阅器

在scripts文件夹中写脚本

### 发布

### python

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

#### C++

```c++
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "talker");//初始化ROS，指定节点名字

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;//句柄

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok())//1.Ctrl+C 2.被同名节点踢出ROS网络 3.ros::shutdown() 4.节点中所有NodeHandles都被销毁 ： 会使ok()返回false
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());//和printf有点像

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
```



### 订阅器

### Python

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

### C++

```c++
#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("chatter", 1000, chatterCallback);//当这个话题有发布内容时就会调用chatterCallback函数
    //ros::Subscriber 被销毁时会自动退订话题

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin(); //进入自循环，可以尽可能快得调用消息回调函数

  return 0;
}
```

#### 编译

1.在`CMakeLists.txt`中加入依赖

```c++
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(talker src/talker.cpp)
target_link_libraries(talker ${catkin_LIBRARIES})

add_executable(listener src/listener.cpp)
target_link_libraries(listener ${catkin_LIBRARIES})
```

2.在可执行文件`WS/devel/lib/<package name>/<file name>`中加入对消息文件的依赖

`add_dependencies(talker beginner_tutorials_generate_messages_cpp)`

3.`catkin_make`

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

### C++

```c++
#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"

bool add(beginner_tutorials::AddTwoInts::Request  &req,
         beginner_tutorials::AddTwoInts::Response &res)
{
  res.sum = req.a + req.b;
  ROS_INFO("request: x=%ld, y=%ld", (long int)req.a, (long int)req.b);
  ROS_INFO("sending back response: [%ld]", (long int)res.sum);
  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("add_two_ints", add);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
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

### C++

```c++
#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "add_two_ints_client");
  if (argc != 3)
  {
    ROS_INFO("usage: add_two_ints_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
  beginner_tutorials::AddTwoInts srv;
  srv.request.a = atoll(argv[1]);
  srv.request.b = atoll(argv[2]);
  if (client.call(srv))//可能会调用失败 call()返回false
  {
    ROS_INFO("Sum: %ld", (long int)srv.response.sum);
  }
  else
  {
    ROS_ERROR("Failed to call service add_two_ints");
    return 1;
  }

  return 0;
}
```

#### 编译

1.`CMakelists.txt`增加依赖

```
add_executable(add_two_ints_server src/add_two_ints_server.cpp)
target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
add_dependencies(add_two_ints_server beginner_tutorials_gencpp)

add_executable(add_two_ints_client src/add_two_ints_client.cpp)
target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
add_dependencies(add_two_ints_client beginner_tutorials_gencpp)
```

2.`catkin_make`

## 录制与回放

http://wiki.ros.org/cn/ROS/Tutorials/Recording%20and%20playing%20back%20data

`rosbag`