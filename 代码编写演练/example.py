# 最近在学习Baxter双臂机器人，做了一个简单的单臂物块抓取来作为起步阶段的成果。
# 大体思路很简单：让手臂到达指定的起始位置，获取图像，找到物块的轮廓并计算单应性矩阵，
# 再计算手臂实际需要移动的偏移量，最后控制手臂抵达目标位置。
# 程序中机械臂末端的姿态固定（抓手坐标系Z轴垂直于工作平面），致使Baxter手中的摄像头总是垂直于工作平面的。
# 该程序中机器人视觉是二维的，深度信息（抓手应下降的距离）由操作者提供。
# 总体上来说十分基础，还望大佬们批评指正
# 图像处理部分：这一部分是整个程序的核心。
# 在基于位置的视觉控制中，想要抓取物体，则必须要检测到物体，还需要知道物体在世界坐标系中的位置。
# --------------------- 
# 作者：Hey_chaoxia 
# 来源：CSDN 
# 原文：https://blog.csdn.net/hey_chaoxia/article/details/81914729 
# 版权声明：本文为博主原创文章，转载请附上博文链接！

# 首先是订阅Baxter相机发布的话题，在回调函数中将接收到的图像（ROS信息）转换为opencv格式。
#订阅摄像头的话题信息（Image），并对其做处理
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
# 对图像的初步处理步骤如下：转换到HSV图像空间（HSV空间更容易分辨颜色）-->提取图像中的蓝色部分-->腐蚀与膨胀去除噪点-->转换为灰度图像-->二值化
        #图像处理
	def _image_process(self, color):
 
		# Convert BGR to HSV
		hsv = cv2.cvtColor(self._original_image, cv2.COLOR_BGR2HSV)    #转换到HSV颜色空间
 
		lower_color = np.array(self._color_dict[color][0])
		upper_color = np.array(self._color_dict[color][1])
		# Threshold the HSV image to get only blue colors
		mask = cv2.inRange(hsv, lower_color, upper_color)   #在hsv颜色空间中获取图像中的颜色部分，用inRange制作掩膜，即该部分为感兴趣的部分
 
		# Bitwise-AND mask and original image
		res = cv2.bitwise_and(self._original_image, self._original_image, mask= mask)  #原图像与掩膜进行与操作，则只剩下掩膜部分
 
		kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (8, 8)) #获得构造元素
		open_img = cv2.morphologyEx(res, cv2.MORPH_OPEN, kernel)
 
		gray_img = cv2.cvtColor(open_img, cv2.COLOR_BGR2GRAY)
		ret, self._bin_img = cv2.threshold(gray_img, 10, 255, cv2.THRESH_BINARY)
# 利用棋盘格上实际距离已知的多个点来计算单应性矩阵，即可得到当前图像平面到实际工作平面到单应性矩阵，在抓手到工作平面的垂直距离不变的情况下一直有效，若变化则需要重新计算。
# 使用opencv的单应性矩阵计算函数，实际的计算算法已经隐去。
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
# 得到单应性矩阵以后，即可计算与物体间的实际偏差了。二值化后的图像可以由opencv函数cv2.findContours()处理，可在图像中找到物块的轮廓（一系列坐标点），再使用cv2.minAreaRect()得到轮廓最小外接矩形的中心点坐标、长宽、以及旋转的角度。其中中心点坐标与旋转角度是后面计算必须的。
#         将物体中心点坐标与单应性矩阵相乘，即可得到其在工作平面坐标系的坐标值。这里的工作平面坐标系是在计算单应性矩阵时定义的，由棋盘格的最左下角的像素点位置决定。随着机械臂末端在平行于桌面的平面中移动，工作坐标系也在不断变化，不方便计算由机器人基坐标系到工作坐标系的变换关系，故这里使用相对距离来计算：得到物体中心点在工作平面坐标系的坐标值后，再计算图像中心点在该坐标系的坐标值，两者相减，即可得到机械臂末端与物块间在x、y方向上的误差（以基坐标系为参考系），而机械臂末端坐标值已知，将其加上偏差，再由操作者提供深度信息，即可得到物块的位姿。
#         要注意的一点是opencv图像坐标系的x、y方向与Baxter基坐标系x、y轴方向有所差异，在返回实际偏差值时要做一些修改。同时，相机与夹具间的距离也需要补偿。
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


# 机械臂末端位姿获取：
#         Baxter启动后会将自身各坐标系的变换关系发布到 '/tf' 话题中，我们只需要使用TF包（具体参考wiki）即可。定义的current_pose()函数是为了将矩阵转换为ROS的pose Message。
class tf_listener():
	def __init__(self):
		#订阅'/tf'，使各坐标系变换实时更新
		rospy.Subscriber('/tf', TFMessage, self.frame_handler)
		#Baxter的坐标系变换
		self.tf_baxter = tf.TransformerROS()
		
		self.trans_left = None	
		self.trans_right = None		
	
 
	def frame_handler(self, tf_message):
		temp_msg = TransformStamped()
		#将收到的TFMessage中的各个坐标系的信息进行更新
		for i in range(0,len(tf_message.transforms)):
			temp_msg = tf_message.transforms[i]
			self.tf_baxter.setTransform(temp_msg)
		
		#Baxter中torso与base的坐标系姿态相同,求从left_gripper到基坐标系的变换
		try:
			self.trans_left = self.tf_baxter.lookupTransform('torso','left_gripper',rospy.Time())
			self.trans_right = self.tf_baxter.lookupTransform('torso','right_gripper',rospy.Time())
			
		except:
			rospy.loginfo("正在尝试与tf同步............. \n")
		
	def current_pose(self, name):	
		"""
	该函数用于获得当前左/右抓手的位姿
	"""
		if name == 'left':
			ans = Pose()				
			ans.position.x = self.trans_left[0][0]
			ans.position.y = self.trans_left[0][1]
			ans.position.z = self.trans_left[0][2]
			ans.orientation.x = self.trans_left[1][0]
			ans.orientation.y = self.trans_left[1][1]
			ans.orientation.z = self.trans_left[1][2]
			ans.orientation.w = self.trans_left[1][3]
			return ans
		else:
			ans = Pose()				
			ans.position.x = self.trans_right[0][0]
			ans.position.y = self.trans_right[0][1]
			ans.position.z = self.trans_right[0][2]
			ans.orientation.x = self.trans_right[1][0]
			ans.orientation.y = self.trans_right[1][1]
			ans.orientation.z = self.trans_right[1][2]
			ans.orientation.w = self.trans_right[1][3]
			return ans 
#  Baxter内置的逆运动学求解器：
#        这一部分参考Baxter sdk wiki，会调用Baxter的服务就行。
#Baxter内置的逆运动学求解器	
class baxter_ik_srv:
	def __init__(self):
		rospy.wait_for_service('/ExternalTools/left/PositionKinematicsNode/IKService')
		self.ik_service = rospy.ServiceProxy('/ExternalTools/left/PositionKinematicsNode/IKService',SolvePositionIK)	#创建服务原型
	def solve(self,pose):
		posestamped = PoseStamped()	#服务要求的输入为PoseStamped（多个，我们只送一个）
		posestamped.pose = pose	#传入的参数类型为Pose
		posestamped.header.stamp = rospy.Time.now()	#将pose打上stamp
		posestamped.header.frame_id = 'base'	
		req = SolvePositionIKRequest()	#请求实例
		req.pose_stamp.append(posestamped)
		rospy.wait_for_service('/ExternalTools/left/PositionKinematicsNode/IKService')	#等待服务可用
		try:
			resp = self.ik_service(req)	#请求服务
			if resp.isValid[0] == True:	#True代表有解，只送一个进去，故索引为0
				return resp
			else:
				rospy.logerr("反解器无解..........\n")
				return None
		except rospy.ServiceException as exc:
			rospy.logerr("请求服务出错：" + str(exc))
# Baxter控制模块：
 #       设置了抓手的最大抓取力矩，以防止抓手损坏（一个还是挺贵的），函数的用法均可以在Baxter sdk wiki上查找到。这里包装的也很简单，就不多做描述。
 class baxter_control():
	def __init__(self):
		#手臂初始化
		self.IK_srv = baxter_ik_srv()	#启动逆运动学求解器
		self.left_arm = Limb('left')    #手臂实例
		self.left_arm.set_joint_position_speed(0.2)	#设置位置控制时的关节速度
 
		#末端执行器初始化，打开并设置最大力矩
		self.left_gripper = Gripper('left')	#抓手实例
		if self.left_gripper.calibrated() == False:	#若抓手未校准，则校准
			self.left_gripper.calibrate()
		self.left_gripper.open(block=True)
		self.left_gripper.set_velocity(5)	#设置抓手移动时的速度
		self.left_gripper.set_moving_force(10)	#设置抓手移动时的力（最大值）
		self.left_gripper.set_holding_force(5)  #抓住物体时的保持力矩
 
		#末端红外测距初始化（备用）
		self.left_range = AnalogIO('left_hand_range') 	#红外测距,有效距离30厘米
 
	def go(self,pose):
		"""
		输入参数——姿态pose
		经逆运动学求解后，控制机械臂抵达目标位置
		"""
		ik_response = self.IK_srv.solve(pose)
		try:
  	  		limb_joints = dict(zip(ik_response.joints[0].name, ik_response.joints[0].position)) #dict zip从两个列表构造字典
   			self.left_arm.move_to_joint_positions(limb_joints)
		except:
			rospy.logerr("无法抵达目标位置")
 
	def go_start_position(self):
		start_pose = Pose()	#设置起始位置并让手臂抵达
		start_pose.position.x = 0.65
		start_pose.position.y = 0.14
		start_pose.position.z = 0.27
		start_pose.orientation.x = 0.0
		start_pose.orientation.y = 1.0 
		start_pose.orientation.z = 0.0
		start_pose.orientation.w = 0.0
		self.go(start_pose)	#移动
# 主函数 ：
        #下面是一段比较简单的主函数，起始位置->找到物块->移动->下降并抓取。目标位置是由自身位置加上偏差获取的。
 
def main():
        rospy.init_node('pick_demo_no_moveit')
	baxter_ctrl = baxter_control()	#初始化Baxter移动控制器
	visual_processor = image_converter()	#开启相机图像处理
	pose_processor = tf_listener()	#开启坐标系变换追踪
	rospy.sleep(5)
 
        baxter_ctrl.go_start_position()
        delta_pose =  visual_processor._get_obj_world_pose('blue')	#得到应在桌面坐标系移动的距离	
        tar = pose_processor.current_pose('left')
        tar.position.x += delta_pose[0]	#根据偏差移动
	tar.position.y += delta_pose[1]
	self.go(tar)
        tar.position.z -= 0.21	#下降（抓取）姿态
	self.go(tar)	#移动
	self.left_gripper.close(block=True)	#抓手抓取
#整个程序仍然有很多地方可以改进，比如识别蓝色以外的物体，图像中有多个物体时如何处理，物体摆放的角度不同时要如何处理等等。注意在进行连续抓取时，单应性矩阵只需进行一次计算（当然每次抓取前都要回到起始位置所在的水平面），而不是每次都要识别棋盘格的存在。
 #        以上就是Baxter抓取物块的一个简单demo。