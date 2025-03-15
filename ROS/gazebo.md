### gazebo仿真



#### URDF文件的XML标签

(wiki.ros.org/urdf/XML)

用于描述模型的文件

**robot：**机器人本体

**Link：**力臂，手臂

**Joint：**关节

​		*Child和Parent是描述一个关节必须的标签*

**inertial：** 惯性相关，重量、惯性矩阵等

**visual：**视觉部分，大小，长度

**collision：**碰撞相关

**检查urdf文件：**在urdf文件目录下，使用check_urdf  （文件名） 检查urdf文件是否合法，显示其中link





#### **gazebo安装后还需要安装controller**

**1.安装controller**

​	`sudo apt-get install ros-noetic-gazebo-ros-control` 

**2.安装ros controllers**

​	`sudo apt-get install ros-noetic-ros-control`	

**3.更新controllers**

​	`sudo apt-get install ros-noetic-ros-controllers`



#### keyboard controller

**1.安装keyboard**

​	`sudo apt-get install ros-noetic-teleop-twist-keyboard`

**2.安装sdformat库**

​	`sudo apt-get install libsdformat11-dev`

​	其中sdformat库的版本“11”要对应gazebo版本

**3.catkin build**

**4.启动键盘控制节点**

​	将键盘映射成速度发布至“/cmd_vel“话题



#### gazebo崩溃或意外退出

gazebo进程未正常退出，会导致部分端口（如11345）一直被占用，导致新实例无法启动。

`ps aux | grep -E 'gazebo|gzserver|gzclient'` 查询gazebo相关残留进程

`kill -9 <ID>` 杀死对应进程，<ID>为对应进程id

`lsof -i :11345` 确认端口11345是否被释放
