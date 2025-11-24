###################################
PAVO2S ROS2 package 2.2.1
###################################

//////修改记录////////////
1. [20240418 v2.1.0] 对SDK点云数据处理进行优化，提高处理效率，降低CPU消耗
2. [20240418 v2.1.0] 对ROS编译选项进行优化，提高ROS Driver处理效率
3. [20240424 v2.1.1] 对ROS Driver的输入参数进行检查
4. [20240424 v2.1.1] 解决老版本雷达由于缺少“电机转速能力” “分辨率能力”寄存器而不能与现有Driver兼容的问题
5. [20240426 v2.1.2] 对所有输入参数进行检查和确认
6. [20240523 v2.2.0] 
    (1) 清理代码中冗余部分
    (2) 打印所有参数保存在雷达中的值
    (3) 修复上位机接收不及时, 导致内存无序增长的问题
6. [20240529 v2.2.1]
    (1) 对launch文件中参数类型进行固化
    (2) 输出pcd launch文件中冗余参数"range_min"和"range_max"   

使用方法：
1.在系统中安装ros环境，具体安装方法参考下面连接：
  安装链接：https://www.guyuehome.com/10226
  或 https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Binary.html
 
2.将pavo2s_ros复制到pavo2s_ws工作目录下的src目录

3.编译工程
  colcon build
  如果提示colcon没有安装的话，可以使用如下命令安装：
  sudo apt install python3-colcon-common-extensions
  
4.设置环境变量
  source install/setup.sh
  
5.配置上位机IP
  与雷达连接的网卡IP：默认上位机IP配置为 10.10.10.100
  
6.配置雷达参数
  打开pavo2s_ros2/launch/pavo2s_scan.launch.py文件，进行参数配置
  参数说明：
  1.frame_id 雷达id，default=laser_frame
  2.scan_topic 雷达的topic名称，default=scan
  5.range_min 最小距离，单位米，default=0.10
  6.range_max 最大距离，单位米，default=35.0
  7.angle_start 雷达起始角度，default=4000，即40度
  8.angle_end 雷达结束角度，default=32000，即320度
  10.inverted 是否设置翻转，取值范围true，false . default=false
  13.lidar_ip 所要连接的雷达IP地址，默认为10.10.10.101.
  14.lidar_port 所要连接的雷达端口号，默认为2368.
  15.dest_port 主机端口号，默认为2368.
  16.echo_mode 二回波使能，使能二回波true，不使能二回波false，default=true 

7.启动PAVO2 ros节点
  1)发布LaserScan消息
    1.ros2 launch pavo2s_ros pavo2s_scan.launch.py
    2.ros2 launch pavo2s_ros pavo2s_scan_view.launch.py (使用rviz显示）
  2)发布PointCloud消息
    1.ros2 launch pavo2s_ros pavo2s_pcd.launch.py
    2.ros2 launch pavo2s_ros pavo2s_pcd_view.launch.py (使用rviz显示）



