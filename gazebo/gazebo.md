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

