# Baxter抓取物块——基于单应性矩阵
# 作者：Hey_chaoxia 
# 来源：CSDN 
# 原文：https://blog.csdn.net/hey_chaoxia/article/details/81914729 

# 最近在学习Baxter双臂机器人，做了一个简单的单臂物块抓取来作为起步阶段的成果。
# 大体思路很简单：让手臂到达指定的起始位置，获取图像，
# 找到物块的轮廓并计算单应性矩阵，再计算手臂实际需要移动的偏移量，
# 最后控制手臂抵达目标位置。
# 程序中机械臂末端的姿态固定（抓手坐标系Z轴垂直于工作平面），
# 致使Baxter手中的摄像头总是垂直于工作平面的。
# 该程序中机器人视觉是二维的，深度信息（抓手应下降的距离）由操作者提供。
# 总体上来说十分基础，还望大佬们批评指正

# 图像处理部分：这一部分是整个程序的核心。
# 在基于位置的视觉控制中，想要抓取物体，则必须要检测到物体
# ，还需要知道物体在世界坐标系中的位置。

#订阅摄像头的话题信息（Image），
# 在回调函数中将接收到的图像（ROS信息）转换为opencv格式。
class image_converter:
	def __init__(self):
		self._image_sub = rospy.Subscriber("/cameras/left_hand_camera/image", Image, self._image_callback)
		self._bridge = CvBridge()
 
		self._original_image = None #从相机获取的原始图像(opencv格式)
 
		self._bin_img = None	#经过处理后的二值图像
 
		self._obj_pose_pic = None #图像在图像空间的姿态（最小外接矩形的）（坐标及转角）
 
		self._homomatrix = None	#单应性矩阵（从图像空间到实际空间）
 
		self._color_dict = {'blue':[[90,130,30],[130,200,150]]} #颜色在HSV空间中的上下阈值
 
 
	#转换为opencv图像并更新
	def _image_callback(self, img_data):
	"""
    收到相机信息的回调函数，进行蓝色物体的识别，获取其二维图像空间的位姿
    """
		try:
			self._original_image = self._bridge.imgmsg_to_cv2(img_data, "bgr8") #从ros image转换到openCV的图像格式
		except CvBridgeError as e:
			print e

    #图像处理
    # 转换到HSV图像空间（HSV空间更容易分辨颜色）-->提取图像中的蓝色部分
    # -->腐蚀与膨胀去除噪点-->转换为灰度图像-->二值化
	def _image_process(self, color):
        
		# Convert BGR to HSV
		hsv = cv2.cvtColor(self._original_image, cv2.COLOR_BGR2HSV)    
        # 提取图像中的蓝色部分
		lower_color = np.array(self._color_dict[color][0])
		upper_color = np.array(self._color_dict[color][1])
		# Threshold the HSV image to get only blue colors
		mask = cv2.inRange(hsv, lower_color, upper_color)   #在hsv颜色空间中获取图像中的颜色部分，用inRange制作掩膜，即该部分为感兴趣的部分
 
		# Bitwise-AND mask and original image
		res = cv2.bitwise_and(self._original_image, self._original_image, mask= mask)  #原图像与掩膜进行与操作，则只剩下掩膜部分
 
		kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (8, 8)) #获得构造元素
		open_img = cv2.morphologyEx(res, cv2.MORPH_OPEN, kernel)    #开操作 #cv可以彩图开操作？
 
		gray_img = cv2.cvtColor(open_img, cv2.COLOR_BGR2GRAY)
		ret, self._bin_img = cv2.threshold(gray_img, 10, 255, cv2.THRESH_BINARY)
# 利用棋盘格上实际距离已知的多个点来计算单应性矩阵，
# 即可得到当前图像平面到实际工作平面到单应性矩阵，
# 在抓手到工作平面的垂直距离不变的情况下一直有效，若变化则需要重新计算。
# 使用opencv的单应性矩阵计算函数，实际的计算算法已经隐去。

# 得到单应性矩阵以后，即可计算与物体间的实际偏差了。
# 二值化后的图像可以由opencv函数cv2.findContours()处理，
# 可在图像中找到物块的轮廓（一系列坐标点），
# 再使用cv2.minAreaRect()得到轮廓最小外接矩形的中心点坐标、长宽、以及旋转的角度。
# 其中中心点坐标与旋转角度是后面计算必须的。

def _get_homomatrix(self):
		"""
        计算从图像平面上的棋盘格点到实际棋盘格点之间的单应性矩阵
		相当于从图像平面上的棋盘格点到垂直于棋盘格拍摄的1:1大小图像之间的映射
        """
		
		board_size = (8, 6) #棋盘格内点8x6
 
		rospy.loginfo("正在检测棋盘格..........\n")
		gray_img = cv2.cvtColor(self._original_image, cv2.COLOR_BGR2GRAY)
		ret, corners = cv2.findChessboardCorners(gray_img, board_size, None)	#corners 是一维向量，记录了6x8=48个点的图像坐标, 从左到右，从下到上，每行排序
		if corners == None:
			return "error：没有发现棋盘格"
 
		# 24.5mm
		unit = 24.5
		table_points = []
		for i in range(board_size[1]):	#将棋盘格的实际坐标（相对的）录入table_points，也是按行排列
			for j in range(board_size[0]):
					temp = [unit * (j + 1), unit * (i + 1)]
					table_points.append(temp)
 
		corners = np.squeeze(np.array(corners), axis=(1,))	#去掉一个多余维度,还剩两个
		table_points = np.array(table_points).astype(np.float32) #使table_points格式与corners相同
		
		#从corners图像平面向已知的棋盘格平面的映射
		self._homomatrix, status = cv2.findHomography(corners, table_points, cv2.RANSAC)
		if self.flag == 0:
			
			self.flag = 1
		print self._homomatrix
		return 1

#将物体中心点坐标与单应性矩阵相乘，即可得到其在工作平面坐标系的坐标值。
# 这里的工作平面坐标系是在计算单应性矩阵时定义的，由棋盘格的最左下角的像素点位置决定。
# 随着机械臂末端在平行于桌面的平面中移动，工作坐标系也在不断变化，
# 不方便计算由机器人基坐标系到工作坐标系的变换关系，故这里使用相对距离来计算：
# 得到物体中心点在工作平面坐标系的坐标值后，再计算图像中心点在该坐标系的坐标值，
# 两者相减，即可得到机械臂末端与物块间在x、y方向上的误差（以基坐标系为参考系），
# 而机械臂末端坐标值已知，将其加上偏差，再由操作者提供深度信息，即可得到物块的位姿。

#要注意的一点是opencv图像坐标系的x、y方向与Baxter基坐标系x、y轴方向有所差异，
# 在返回实际偏差值时要做一些修改。同时，相机与夹具间的距离也需要补偿。

def _get_obj_world_pose(self, color):
		"""
		获取单应性矩阵后，首先寻找物体的轮廓，
        Return the pose of target object in the real world.
        delta-->x, y ,angle
        """
		self._image_process(color)
 
		self._get_homomatrix() #先获取单应性矩阵
 
		contours, hierarchy = cv2.findContours(self._bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE) #CHAIN_APPROX_NONE
		cv2.drawContours(self._original_image, contours, -1, (0, 0, 255), 3)
 
		# 得到轮廓数量（即物体数量），并将轮廓转存为numpy
		contours_array = np.array(contours)
		contours_total_num = contours_array.shape[0]
 
		object_contour = contours_array[0]
		
		# Get min area rect.
		rect = cv2.minAreaRect(object_contour) #找到最小外接矩形
		box = cv2.cv.BoxPoints(rect)    #得到矩形的四个点
		box = np.int0(box)  #转换数据类型？？
 
		x, y = rect[0]      #矩形的中心
		width, height = rect[1]	#矩形的长与宽
		angle = rect[2] #矩形的转角
 
		
		#数据转储
		self._obj_pose_pic = [x, y, angle]	#存储矩形的中心、转角
 
		x_img = self._obj_pose_pic[0]	#得到代表物体的矩形中心点位置
		y_img = self._obj_pose_pic[1]
		angle_img = self._obj_pose_pic[2]	#转角
		width_img = self._original_image.shape[1]    # 640
		height_img = self._original_image.shape[0]   # 400
 
		#计算物体中心点与图像中心点在桌平面坐标系的位置
		obj_img = np.array([x_img, y_img, 1])	#1*3 numpy矩阵
		obj_world = np.dot(self._homomatrix, obj_img) #矩阵乘法，将物体的中心点映射到棋盘格平面（桌子平面），即得到物体中心点在桌坐标系中的位置
 
		center_img = np.array([width_img/2.0, height_img/2.0, 1])	#获得图像中心点坐标
		center_world = np.dot(self._homomatrix, center_img)	#得到桌坐标系中的中心点坐标
 
		delta_world = obj_world - center_world	#得到桌坐标系中的两者误差（则抓手应是垂直于桌面的）
		#print delta_world
 
		if delta_world[0] > 0:    # 桌面x轴/Baxtery轴误差		#这里有一个桌面到机器人坐标系间的变换，Baxter正前方是x轴
			print "move right(cm): ", delta_world[0]/10.0
		else:
			print "move left(cm): ", abs(delta_world[0]/10.0)
 
		if delta_world[1] > 0:    # 桌面y轴/Baxterx轴误差
			print "move back(cm): ", delta_world[1]/10.0
		else:
			print "move front(cm): ", abs(delta_world[1]/10.0)
 
		angle = angle_img	#注意！符号和两个坐标轴
		delta_pose_info = [-delta_world[1]/1000.0, -delta_world[0]/1000.0, angle] #转换为米的Baxterx轴/y轴误差
		delta_pose_info[0] = delta_pose_info[0] - 0.041   # 相机与抓手之间的误差补偿
	        delta_pose_info[1] = delta_pose_info[1] + 0.024   
 
		return delta_pose_info


