import os
import sys
import argparse
import rospy
import baxter_interface
import cv2
import cv_bridge

from sensor_msgs.msg import (
    Image,
)

def send_image(path):
    """
    Send the image located at the specified path to the head
    display on Baxter.

    @param path: path to the image file to load and send
    """
    img = cv2.imread(path)
    msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
    pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
    pub.publish(msg)
    # Sleep to allow for image to be published.
    rospy.sleep(1)


def main():
    # 一些声明的东西，先不删了……
    """RSDK Xdisplay Example: Image Display

    Displays a given image file on Baxter's face.

    Pass the relative or absolute file path to an image file on your
    computer, and the example will read and convert the image using
    cv_bridge, sending it to the screen as a standard ROS Image Message.
    """
#     epilog = """
# Notes:
#     Max screen resolution is 1024x600.
#     Images are always aligned to the top-left corner.
#     Image formats are those supported by OpenCv - LoadImage().
#     """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__）#,
                                     #epilog=epilog)
    # required = parser.add_argument_group('required arguments')
    # required.add_argument(
    #     '-f', '--file', metavar='PATH', required=True,
    #     help='Path to image file to send'
    # )
    # parser.add_argument(
    #     '-d', '--delay', metavar='SEC', type=float, default=0.0,
    #     help='Time in seconds to wait before publishing image'
    # )
    # args = parser.parse_args(rospy.myargv()[1:])
    parser.parse_args(rospy.myargv()[1:])

    #开始

rospy.init_node('Goodbye_Baxter', anonymous=True)

# if not os.access(args.file, os.R_OK):
#     rospy.logerr("Cannot read file at '%s'" % (args.file,))
#     return 1

# # Wait for specified time
# if args.delay > 0:
#     rospy.loginfo(
#         "Waiting for %s second(s) before publishing image to face" %
#         (args.delay,)
#     )
#     rospy.sleep(args.delay)
pic1 = '2.png'
pic2 = 'test.png'

#leftl = baxter_interface.Limb('left')
rightl = baxter_interface.Limb('right')
#挥舞手臂
wave_1 = {'right_s0': -0.459, 'right_s1': -0.202, 'right_e0': 1.807, 'right_e1': 1.714, 'right_w0': -0.906, 'right_w1': -1.545, 'right_w2': -0.276}
wave_2 = {'right_s0': -0.395, 'right_s1': -0.202, 'right_e0': 1.831, 'right_e1': 1.981, 'right_w0': -1.979, 'right_w1': -1.100, 'right_w2': -0.448}

head = baxter_interface.Head()
head.command_nod()

for _move in range(3):
    head.set_pan(1)
    rightl.move_to_joint_positions(wave_1) 
    head.set_pan(-1)
    rightl.move_to_joint_positions(wave_2)
    head.set_pan(0)
    head.command_nod()
    rospy.sleep(1)
    # send_image(pic1)
    # rospy.sleep(1)
    # send_image(pic2)
    # rospy.sleep(1)
    
    return 0

for _move in range(3):
    rightl.move_to_joint_positions(wave_1) 
    rightl.move_to_joint_positions(wave_2)

if __name__ == '__main__':
    sys.exit(main())

