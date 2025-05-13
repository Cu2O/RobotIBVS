# RobotIBVS
基于yolov5目标识别，moveit动作规划，gazebo仿真的eye-in-hand视觉伺服Image-Based Visual Servoing，实现相机检测到目标，移动到其上方。

## 环境
ubuntu18.04 + python3.8 + torch1.11 + ros melodic

具体配环境方法可参考：https://blog.csdn.net/Chris121345/article/details/122563536


### Yolov5包
通过网盘分享的文件：Yolov5_ros.zip
链接: https://pan.baidu.com/s/17ybWmtQZ_Hb5RbViyzZQ3A 提取码: s2hi 
--来自百度网盘超级会员v1的分享

放入robot_visionservo下，与description/gazebo/plan/moveit并列。

### python3环境验证
在目录```\robot_visionservo\Yolov5_ros\yolov5_ros\yolov5_ros\yolov5```下

```python3 detect.py --source ./data/images/ --weights weights/yolov5s.pt```验证yolov5是否正常运行

### 可能遇见问题及解决方案
1. https://blog.csdn.net/qq_38156743/article/details/124091508
2. https://blog.csdn.net/m0_46928770/article/details/135535062
3. https://blog.csdn.net/m0_74923682/article/details/129255801（虚拟机存储空间不足）

### 为gazebo添加新模型
四个模型文件，对应视频中的三个多边形体以及。
模型文件的制作以及贴图的教程可以看博客。博客地址：
https://blog.csdn.net/qq_48427527/article/details/124477608?spm=1001.2014.3001.5502

打开隐藏文件夹，将四个模型拷贝放置到.gazebo/models路径，如不存在就创建models文件夹。注意不能将整个包拷贝，仅将四个模型文件拷贝即可。

## 使用方法
### robot_visionservo
1. 放入 ```./workspace(自己的工作空间)/src```下
2. 在 ```./workspace(自己的工作空间) ```下 ```catkin_make``` 编译

### 启动仿真环境
1. ```./workspace(自己的工作空间) ```下启动一个终端
2. ```conda activate yolov5```激活python3.8环境
3. ```source devel/setup.bash```
4. ```roslaunch my_robot_gazebo robot.launch```开启gazebo+rviz+相机视野共三个窗口的仿真环境

### 直线/曲线运动仿真
1. ```./workspace(自己的工作空间) ```下启动新终端
2. ```source devel/setup.bash```
3. ```python2 /home/name(自己的home)/workspace(自己的工作空间)/src/my_robot_visionservo/my_robot_planing/scripts/moveit_yolo/grasp_linemove.py ```
开启直线运动仿真
4. ```python2 /home/name(自己的home)/workspace(自己的工作空间)/src/my_robot_visionservo/my_robot_planing/scripts/moveit_yolo/grasp_circlemove.py ```
开启曲线运动仿真

### 视觉伺服运动仿真
1. 和上述相同，使用命令：
```python2 /home/name(自己的home)/workspace(自己的工作空间)/src/my_robot_visionservo/my_robot_planing/scripts/moveit_yolo/grasp_IBVS.py```打开视觉伺服控制仿真
2. 将三角形添加到视野可见范围，则视角会随之移动到中央。

3. 需注意控制频率较低，可换moveit规划为基于雅可比的逆运动学规划。

## 感谢
感谢中文博客论坛
https://github.com/Lord-Z/sunday?tab=readme-ov-file
对上述内容启发。