### description
 存储机械臂、夹爪、相机的urdf与xacro文件
 需注意camera的parent为claw_base，rpy改变其x轴朝向
### gazebo
 启动gazebo与yolov5节点在此的launch文件中存储，其他的难度不大
### moveit_config
 本次未使用moveit，仅使用通信机制完成
### planing
## scripts--moveit_yolo
 基于位置的视觉伺服，运用moveit，不太好使，就图一乐
## scripts--my_ur_kinematics
 大部分代码是对UR5机器人运动学分析的代码，开源下的，仅有计算雅可比的代码是我自己写的
# ur_config
 存储UR5机械臂DH建系配置
# commonly_used
 存储各个齐次变换矩阵的地方
# control_abs_angles_in_pi
 控制角度极限
# Foward_Kinematics
 正向运动学下三个函数，计算齐次变换矩阵；计算雅可比矩阵；将速度从相机系转换到base系
 前两个原理需阅读机器人ppt，速度换系就是乘R
# Inverse_Kinematics
 逆向运动学，8个解全部解决，需要阅读网上ur5正逆运动学教程
# select_ik_solution
 根据上一步的位置，从8个解中选择最佳解

### 在理解move前需要仔细理解伺服控制
我们的思路很清晰 让物体的重心始终在屏幕的中心
如果可以根据图像调整相机位置，再调节机械臂位置就好了

基于图像雅可比和基于学习的控制方法等
没有深度信息（到手术器械的距离）的内窥镜是很大的限制，那么如何应用图像雅可比是第一个问题

但我们最先解决的是第二个问题，因为机器人的运动学是需要最先解决的问题
*代码解决了 theta'（各关节角速度）= Jacob#（雅可比的伪逆）*vbase（在base下的机械臂末端速度）
因此如果知道vbase就好了

之后完成的是基于YOLOv5的定位，以获得vend（在机械臂末端下速度）
YOLOv5优秀在适合工程应用，配置较为简单，数据集标注也还好。
将YOLOv5给我的预测框计算中心 这个点就是我们要追踪的重心

此时可以考虑运动学分析时需要解决的 如何根据图像获得速度
*vend速度大小好定，根据PID控制，可以确定个大小。
*vend最重要的是方向。我们希望的是我们可以和物体重心指向图片中心的方向
*图像雅可比完成的是 从物体速度到图像速度的转换，需要自行了解
*那么图像雅可比的伪逆乘图像速度取单位向量，就是物体速度的方向。

解决vend，通过R.T*vend获得vbase，theta' = Jacob# * vbase
我最后用的action通信，用theta = theta + theta' * time 获得位置
可以考虑下速度控制

### move