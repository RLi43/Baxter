#-- Baxter Learning --
#-- Main Goal: Right Arm follows a blue object 
#-- Author: R.Li
#-- References:
#     https://blog.csdn.net/hey_chaoxia/article/details/81914729 
#     http://sdk.rethinkrobotics.com/wiki
import os
import sys
import argparse
import rospy

import baxter_interface


import cv2
import cv_bridge

# def show_image():
#     """
#     Send the image located at the specified path to the head
#     display on Baxter.

#     @param path: path to the image file to load and send
#     """
#     path = 'cameras/right_camera/image'
#     img = cv2.imread(path)
#     msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
#     pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
#     pub.publish(msg)
#     # Sleep to allow for image to be published.
#     rospy.sleep(1)
class Follow_Blue_Point()
    def __init__(self):
		self._image_sub = rospy.Subscriber("/cameras/left_hand_camera/image", Image, self._image_callback)
		self._bridge = CvBridge() 
		self._original_image = None #从相机获取的原始图像(opencv格式)
		self._bin_img = None	#经过处理后的二值图像 
		self._color_dict = {'blue':[[90,130,30],[130,200,150]]} #颜色在HSV空间中的上下阈值

        self._limb = baxter_interface.Limb('right')
    
    def begin()
        #TODO:xdisplay
        quiting = False
        while not rospy.is_shutdown(): 
            direct = dirDetection()
            hand_turn(direct)
            #Sleep(1)

    #转换为opencv图像并更新
	def _image_callback(self, img_data):
	"""
    收到相机信息的回调函数，进行蓝色物体的识别，获取其二维图像空间的位姿
    """
		try:
			self._original_image = self._bridge.imgmsg_to_cv2(img_data, "bgr8") #从ros image转换到openCV的图像格式
		except CvBridgeError as e:
			print e

    def dirDetection(self)
        #dir = 'n' 'l' 'r' 'u' 'd'
        direct = 'n'

		hsv = cv2.cvtColor(self._original_image, cv2.COLOR_BGR2HSV)    
        # 提取图像中的蓝色部分
		lower_color = np.array(self._color_dict[color][0])
		upper_color = np.array(self._color_dict[color][1])
		obj = cv2.inRange(hsv, lower_color, upper_color)   #在hsv颜色空间中获取图像中的颜色部分，用inRange制作掩膜，即该部分为感兴趣的部分
 
        #TODO: 计算obj的中心 判断方向

		# # Bitwise-AND mask and original image
		# res = cv2.bitwise_and(self._original_image, self._original_image, mask= mask)  #原图像与掩膜进行与操作，则只剩下掩膜部分
 
		# kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (8, 8)) #获得构造元素
		# open_img = cv2.morphologyEx(res, cv2.MORPH_OPEN, kernel)    #开操作 #cv可以彩图开操作？
 
		# gray_img = cv2.cvtColor(open_img, cv2.COLOR_BGR2GRAY)
		# ret, self._bin_img = cv2.threshold(gray_img, 10, 255, cv2.THRESH_BINARY)

    def hand_turn(direct)
        #direct -- 'r' for right 'l' for left
        joint_name = 'right_w1'
        delta = 0.1
        if direct == 'l'
            delta = -delta
        cur_angle = limb.joint_angle(joint_name)
        joint_cmd = {joint_name:cur_angle+delta}
        limb.set_joint_positions(joint_cmd)

        



def main()
    """Follow Example: Right Arm -Version 0.1

    Use the camera on right hand, follow a red object
    Try to show the right hand camera data on xdisplay?
    """

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.parse_args(rospy.myargv()[1:])

    print("Initializing node... ")
    rospy.init_node("following_example_version_0_1")

    print("Enabling robot... ")
    rs.enable() #在程序执行之前Enable机器人

    Follow_Blue_Point fbp


    print("Done.")


if __name__ == '__main__':
    main()