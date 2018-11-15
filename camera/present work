# 相机使用

## 概述

Baxter有三台彩色摄像机，2台在手中，'left_hand_camera'和'right_hand_camera'，以及Baxter头上的“head_camera”。

**重要说明：**由于带宽能力有限，**只能**同时操作**两台**摄像机。在其他两个相机都在运行时启动相机将导致错误，相机将无法打开。

**重要说明：** Baxter启动时的默认行为是两个手持摄像头在分辨率为320x200，帧速率为25 fps的情况下运行。

### **支持的帧率**



| Mode Index | Resolution key | Imager Width | Window width | Window height | Windowed? | Max. Refresh Rate |
| ---------- | -------------- | ------------ | ------------ | ------------- | --------- | ----------------- |
| 0          | RES_HIGH       | 1280         | 1280         | 800           | N         | 14.1 fps          |
| 1          | RES_HIGH       | 1280         | 960          | 600           | Y         | 23.8 fps          |
| 2          | RES_HIGH       | 1280         | 640          | 400           | Y         | 27.8 fps          |
| 3          | RES_HIGH       | 1280         | 480          | 300           | Y         | 27.9 fps          |
| 4          | RES_HIGH       | 1280         | 384          | 240           | Y         | 27.8 fps          |
| 5          | RES_HIGH       | 1280         | 320          | 200           | Y         | 27.9 fps          |
| 6          | RES_LOW        | 640          | 640          | 400           | N         | 47.6 fps          |
| 7          | RES_LOW        | 640          | 480          | 300           | Y         | 55.5 fps          |
| 8          | RES_LOW        | 640          | 384          | 240           | Y         | 55.5 fps          |
| 9          | RES_LOW        | 640          | 320          | 200           | Y         | 55.5 fps          |
| 10         | RES_LOW        | 640          | 240          | 150           | Y         | 55.5 fps          |
| 11         | RES_LOW        | 640          | 192          | 120           | Y         | 55.5 fps          |

**在不支持的帧宽和高度打开相机时：** 

使用camera_control.py示例 - ValueError: Invalid Camera mode

使用baxter_inteface camera.py API - 相机驱动程序将以最接近指定框架宽度和高度的模式打开。

直接使用服务调用 - 相机驱动程序将以最接近指定帧宽度和高度的模式打开。

### 其他参数

- Frame Width and Height to supported operating modes.
- Frame Rate (fps) 帧率
- Windowing
- Binning
- Exposure 曝光
- Gain 增益
- White Balance 白平衡

## 相机控制示例

查看图片

```
 $ rosrun image_view image_view image:=/cameras/head_camera/image #头部
 $ rosrun image_view image_view image:=/cameras/left_hand_camera/image #左手
 $ rosrun image_view image_view image:=/cameras/right_hand_camera/image #右手
```

 #注意，实际使用时只能打开两个摄像头

**问题**

在仿真的时候可以打开三个摄像头的图像，无法设置摄像头。（调用camera_control.py时超时）

''目前的问题：实际环境可以设置摄像头（camera_control.py），但是无法读取数据？''

## 参考

官方wiki[http://sdk.rethinkrobotics.com/wiki/Cameras]
