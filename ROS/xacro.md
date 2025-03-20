### Xacro文件

xacro文件是一种基于XML的宏语言，用于简化URDF文件的编写。

例：

```xml
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="hero">

    <xacro:arg name="load_chassis" default="true"/>
    <xacro:arg name="use_simulation" default="true"/>
    <xacro:arg name="roller_type" default="realistic"/>

    <xacro:if value="$(arg load_chassis)">
        <xacro:include filename="$(find rm_description)/urdf/hero/chassis.urdf.xacro"/>
        <xacro:chassis roller_type="$(arg roller_type)"/>
    </xacro:if>

    <xacro:if value="$(arg use_simulation)">
        <gazebo>
            <plugin name="rm_ros_control" filename="librm_robot_hw_sim.so">
                <robotNamespace>/</robotNamespace>
                <robotSimType>rm_gazebo/RmRobotHWSim</robotSimType>
            </plugin>
        </gazebo>
    </xacro:if>

</robot>
```

#### `<robot>`根标签

​								**`<robot>`根标签是ros的urdf规范规定好的！**

用来定义一个完整机器人模型的根标签。它作为整个机器人描述文件的起始与核心，将机器人各个组件（如连杆、关节等）以及相关属性和配置信息整合在一起。

```xml
<robot name="my_robot">
    <link name="base_link">
        <!-- 连杆的具体描述 -->
    </link>
    <joint name="base_to_arm" type="revolute">
        <!-- 关节的具体描述 -->
    </joint>
    <gazebo>
        <!-- Gazebo仿真配置 -->
    </gazebo>
</robot>
```





##### `xmlns:xacro`标签用于定义命名空间

`robot xmlns:xacro="http://www.ros.org/wiki/xacro"` 声明使用 Xacro 命名空间，这样就可以使用 Xacro 的宏和条件语句等功能。

##### `xacro:arg`标签用于定义可传入的参数

`load_chassis`：一个布尔类型的参数，默认值为 `true`，用于控制是否加载机器人的底盘。

`use_simulation`：一个布尔类型的参数，默认值为 `true`，用于控制是否使用仿真环境。

`roller_type`：一个字符串类型的参数，默认值为 `realistic`，用于指定滚轮的类型。

##### `xacro:if`标签用于进行条件判断

`value="$(arg load_chassis)"` 表示如果 `load_chassis` 参数的值为 `true`，则执行标签内的代码。

##### `xacro:include`标签用于包含另一个xacro文件



##### `<gazebo>`标签用于配置gazebo仿真相关

**`<plugin>`标签用于设置插件相关**

插件将帮助与ros控制器进行交互

```xml
<gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
        <robotNamespace>/</robotNamespace>
        <robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>
    </plugin>
</gazebo>
```

`name`：插件名称（可自定义）

`filename`：插件的动态链接库文件名

`robotNamespace`：机器人的命名空间

`robotSimType`：指定机器人的仿真类型

