# ROS学习笔记

### 发布者Publisher的编程实现

1. 创建功能包

```c++
$ cd ~/catkin_ws/src
$ catkin_create_pkh learning_topic roscpp rospy std_msgs geometry_msgs turtlesim
```

2. 创建代码文件

   + ==在功能包的src文件夹==创建一个.cpp文件用于书写代码

   ```c++
   $ touch velocity_publisher.cpp
   ```

   + 书写小海龟运动代码

     + 代码流程
       + 初始化ROS节点
       + 向ROS Master注册节点信息，包括发布的话题名和话题中的消息类型
       + 创建消息数据
       + 按照一定频率循环发送消息

     ```c++
     /**
     该例程将发布turtle1/cmd_vel话题，消息类型geo,etry_msgs::twist
     */
     
     #include <ros/ros.h>
     #include <geometry_msgs/Twist.h>
     
     int main(int argc,char **argv)
     {
     	//ros节点初始化
     	ros::init(argc,argv,"velocity_publisher");
     	
     	//创建节点句柄
     	ros::NodeHandle n;
     	
     	//创建一个Publisher，发布名为/turtle1/cmd_vel的topic，消息类型为geometry_msgs::Twist,队列长度10
     	ros::Publisher turtle_vel_pub = n.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel",10);
     	
     	//设置循环的频率
     	ros::Rate loop_rate(10);
     	
     	int count=0;
     	while(ros::ok())
     	{
     		//初始化geometry_msgs::Twist类型的消息
     		geometry_msgs::Twist vel_msg;
     		vel_msg.linear.x = 0.5;
     		vel_msg.angular.z = 0.2;
     		
     		//发布消息
     		turtle_vel_pub.pulisher(vel_msg);
     		ROS_INFO("Publish turtle velocity command[%0.2f m/s,%0.2f rad/s]",
     						vel_msg.linear.x,vel_msg.angular.z);
     		
     		//按照循环频率延时
     		loop_rate.sleep();
                             
     	}
     	return 0;
     	
     }
     ```

     3. 配置发布者代码编译规则

        * 在功能包的==CMakeLists.txt==文件中加入编译规则

        * 设置需要编译的代码和生成的可执行文件

          ```c++
          //需要编译的代码（路径）：src/velocity_publisher.cpp
          //需要生成的可执行文件：velocity_publisher
          add_executable(velocity_publisher src/velocity_publisher.cpp)
          ```

        * 设置链接库

          ```c++
          //velocity_publisher 即为编译之后生成的可执行文件
          target_link_libraries(velocity_publisher ${catkin_LIBRARIES})
          ```

        * 将以上两条语句放入CMakeLists.txt文件中有关编译的沙盒里
          ########

          ##Build##  ~
          ########

     4. 编译并运行发布者

        * 进入工作空间根目录

          ```c++
          $ cd /~catkin_ws
          ```

        * 编译

          ```c++
          $ catkin_make
          ```

          

        * 设置工作空间环境变量

          ```c++
          $ sourc devel/setup.bash
          ```

          *  也可以在主文件夹中“==Ctrl+H==”找到.bash文件，在最后一行添加上该工作空间的环境变量设置代码

            ```c++
            $ sourc /home/用户名/catkin_ws/devel/setup.bash
            ```

            

        * 运行

          ```c++
          $ roscore									//运行ros
          $ rosrun turtlesim							//打开海龟仿真器，查看节点
          $ rosrun turtlesim_node						//打开海龟仿真器节点					  
          $ rosrun learning_topic velocity_publisher	 //运行发布者命令
          ```

          ## 订阅者Subscriber的编程实现

          1.订阅者程序
          
          * 初始化ROS节点
          * 订阅需要的话题
          * 循环等待话题消息，接收到消息后进入回调函数
          * 在回调函数中完成消息处理
          
          ```c++
          /**
          *该例程将订阅/turtle1/pose话题，消息类型turtlesim::pose
          */
          
          #include <ros/ros.h>
          #include "turtlesim/Pose.h"
          
          //接收到订阅的消息后，会进入消息回调函数
          void poseCallback(const turtlesim::Pose::Constptr& msg)
          {
          	//将接收到的消息打印出来
          	ROS_INFO("Turtle pose: x:%0.6f,y:%0.6f",msg->x,msg->y);
          }
          
          int main(int agrc,char **argv)
          {
              //初始化ROS节点
              ros::init(agrc,agrv,"pose_subscriber");
              
              //创建节点句柄
              ros::NodeHandle n;
              
              //创建一个Subscriber，订阅名为/turtle1/pose的topic，注册回调函数poseCallback
              ros::Subscriber pose_sub = n.subscribe("/turtle1/pose",10,poseCallback);
              
              //循环等待回调函数
              ros::spin();
              return 0;
          }
          ```
          
          2. 配置发布者代码编译规则==（和发布者相同）==
          
             * 在功能包的==CMakeLists.txt==文件中加入编译规则
          
             * 设置需要编译的代码和生成的可执行文件
          
               ```c++
               //需要编译的代码（路径）：src/pose_subscriber.cpp
               //需要生成的可执行文件：pose_subscriber
               add_executable(pose_subscriber src/pose_subscriber.cpp)
               ```
          
             * 设置链接库
          
               ```c++
               //pose_subscriber 即为编译之后生成的可执行文件
               target_link_libraries(pose_subscriber ${catkin_LIBRARIES})
               ```
          
             * 将以上两条语句放入CMakeLists.txt文件中有关编译的沙盒里
               ########
          
               ##Build##  ~
               ########
          
             3. 编译并运行发布者==（和发布者相同）==
          
                * 进入工作空间根目录
          
                  ```c++
                  $ cd /~catkin_ws
                  ```
          
                * 编译
          
                  ```c++
                  $ catkin_make
                  ```
          
                  
          
                * 设置工作空间环境变量
          
                  ```c++
                  $ sourc devel/setup.bash
                  ```
          
                  * 也可以在主文件夹中“==Ctrl+H==”找到.bash文件，在最后一行添加上该工作空间的环境变量设置代码
          
                    ```c++
                    $ sourc /home/用户名/catkin_ws/devel/setup.bash
                    ```
          
                    
          
                * 运行
          
                  ```c++
                  $ roscore									//运行ros
                  $ rosrun turtlesim							//打开海龟仿真器，查看节点
                  $ rosrun turtlesim_node						//打开海龟仿真器节点					  
                  $ rosrun learning_topic pose_subscriber	 //运行订阅者命令
                  ```
          

### 服务和参数

1.  ros服务

​	节点之间的通讯方式，允许节点发送一个请求（request）并获得一个相应（response）

- ​	rosservice

  ```
  rosservice list         输出活跃的服务
  rosservice call         用给定的参数调用服务
  rosservice type         输出服务的类型
  rosservice find         按服务的类型查找服务
  rosservice uri          输出服务的ROSRPC uri
  ```

  - 用rosservice list查看活跃的服务

  - 根据list给出的服务名称，rosservice type查看某个服务的详细信息

    ```
    rosservice type [service]
    ```

    

    **服务通讯**

    ![image-20250126190923304](C:/Users/Lenovo/AppData/Roaming/Typora/typora-user-images/image-20250126190923304.png)

    master 管理者				server 服务端（talker）   			client客户端（listener）

    **ps：**
    
    ​	1.保证顺序，客户端发起请求时，服务端需要已经启动
    
    ​	2.客户端和服务端都可以存在多个
    
    ​	3.ROS Master 负责保管 Server 和 Client 注册的信息，并匹配话题相同的 Server 与 Client ，帮助 Server 		与 Client 建立连接，连接建立后，Client 发送请求信息，Server 返回响应信息。
    
    **服务流程**
    
    ​	Server启动后在master中注册信息 –> Client启动也在master中注册信息 –> master匹配对应的server和client –> client给server发送请求 –> server接收请求，响应结果返回client
    
    
    
    #### **服务通信自定义srv**（与自定义msg类似）
    
    ##### 1.定义srv文件
    
    数据分成两部分：请求 和 响应
    
    ==在srv文件中请求和响应的数据使用“- - - ”（三个减号）分割==
    
    功能包下新建srv目录，添加xxx.srv文件：
    
    ```c++
    #功能：客户端请求时发送两个数字 服务端响应二者之和
    
    # 客户端请求时发送的两个数字(Request)
    int32 num1
    int32 num2
    ---
    # 服务器响应发送的数据(Response)
    int32 sum
    ```
    
    ##### 2.编辑配置文件
    
    **package.xml**中添加编译依赖与执行依赖
    
    ```c++
    <build_depend>message_generation</build_depend>
     <exec_depend>message_runtime</exec_depend>
    ```
    
    **其中message_generation是一个用于生成自定义消息、服务和动作的CMAKE工具包**
    
    
    
    编辑**CmakeList.txt**
    
    ```
    
    # ------编辑find_package-----------
    find_package(catkin REQUIRED COMPONENTS
      roscpp
      rospy
      std_msgs
      message_generation
    )
    # 需要加入 message_generation,必须有 std_msgs
    
    
    # -------加入服务文件-------
    add_service_files(
      FILES
      AddInts.srv
    )
    #包含上一步创建的xxx.srv文件
    
    
    # ------加入自定义消息工具包------
    generate_messages(
    	DEPENDENCIES
    	std_msgs
    )
    
    
    
    ```
    
    #### 服务通信调用自定义srv
    
    **流程：**
    
    编写服务端实现—–编写客户端实现—–编辑配置文件—–编译并执行
    
    
    
    **查看需要调用的head文件**(.../工作空间/devel/include/包名/xxx.h)
    
    将头文件路径加入c_cpp_properies.json中的includePath中
    
    
    
    ##### 1.服务端
    
    ```c++
    /*
        需求: 
            编写两个节点实现服务通信，客户端节点需要提交两个整数到服务器
            服务器需要解析客户端提交的数据，相加后，将结果响应回客户端，
            客户端再解析
    
        服务器实现:
            1.包含头文件
            2.初始化 ROS 节点
            3.创建 ROS 句柄
            4.创建 服务 对象
            5.回调函数处理请求并产生响应
            6.由于请求有多个，需要调用 ros::spin()
    */
    #include "ros/ros.h"
    #include "demo03_server_client/AddInts.h" //包含自定义srv所生成的头文件
    
    /*
     服务端处理函数
     bool 返回值由于标志是否处理成功
    */
    bool doReq(demo03_server_client::AddInts::Request& req,
              demo03_server_client::AddInts::Response& resp){
        int num1 = req.num1;
        int num2 = req.num2;
        ROS_INFO("服务器接收到的请求数据为:num1 = %d, num2 = %d",num1, num2);
        
        //逻辑处理
        if (num1 < 0 || num2 < 0)
        {
            ROS_ERROR("提交的数据异常:数据不可以为负数");
            return false;
        }
        //如果没有异常，那么相加并将结果赋值给 resp
        resp.sum = num1 + num2;
        return true;
    }
    
    int main(int argc, char *argv[])
    {
        // 1.将程序的所有本地化类别设置为系统默认的环境
        setlocale(LC_ALL,"");
        // 2.初始化 ROS 节点
        ros::init(argc,argv,"AddInts_Server");
        // 3.创建 ROS 句柄
        ros::NodeHandle nh;
        // 4.创建 服务 对象
        ros::ServiceServer server = nh.advertiseService("AddInts",doReq);//服务名称"AddInts"
        ROS_INFO("服务已经启动....");
        // 5.回调函数处理请求并产生响应
        // 6.由于请求有多个，需要调用 ros::spin()
        ros::spin();
        return 0;
    }
    ```
    
    ##### 2.客户端
    
    ```c++
    /*
        需求: 
            编写两个节点实现服务通信，客户端节点需要提交两个整数到服务器
            服务器需要解析客户端提交的数据，相加后，将结果响应回客户端，
            客户端再解析
    
        服务器实现:
            1.包含头文件
            2.初始化 ROS 节点
            3.创建 ROS 句柄
            4.创建 客户端 对象
            5.请求服务，接收响应
    
    */
    #include "ros/ros.h"
    #include "demo03_server_client/AddInts.h"
    
    int main(int argc, char *argv[])
    {
        setlocale(LC_ALL,"");
    
        // 调用时动态传值,如果通过 launch 的 args 传参，需要传递的参数个数 +3
        if (argc != 3)
        // if (argc != 5)//launch 传参(0-文件路径 1传入的参数 2传入的参数 3节点名称 4日志路径)
        {
            ROS_ERROR("请提交两个整数");
            return 1;
        }
    
    
        // 2.初始化 ROS 节点
        ros::init(argc,argv,"AddInts_Client");
        // 3.创建 ROS 句柄
        ros::NodeHandle nh;
        // 4.创建 客户端 对象
        ros::ServiceClient client = nh.serviceClient<demo03_server_client::AddInts>("AddInts");
        //等待服务启动成功
        //方式1
        ros::service::waitForService("AddInts");
        //方式2
        // client.waitForExistence();
        
        // 5.组织请求数据
        demo03_server_client::AddInts ai;
        ai.request.num1 = atoi(argv[1]);
        ai.request.num2 = atoi(argv[2]);
        
        // 6.发送请求,返回 bool 值，标记是否成功
        bool flag = client.call(ai); 
        // 7.处理响应
        if (flag)
        {
            ROS_INFO("请求正常处理,响应结果:%d",ai.response.sum);
        }
        else
        {
            ROS_ERROR("请求处理失败....");
            return 1;
        }
    
        return 0;
    }
    ```
    
    ##### 3.配置CMakeLists
    
    ```c++
    /*加入服务端、客户端两个可执行文件*/
    add_executable(AddInts_Server src/AddInts_Server.cpp)
    add_executable(AddInts_Client src/AddInts_Client.cpp)
    
    /*加入两个节点的依赖，参数是两个ros节点的节点名*/
    add_dependencies(AddInts_Server ${PROJECT_NAME}_gencpp)
    add_dependencies(AddInts_Client ${PROJECT_NAME}_gencpp)
    
    /*链接依赖*/
    target_link_libraries(AddInts_Server
      ${catkin_LIBRARIES}
    )
    target_link_libraries(AddInts_Client
      ${catkin_LIBRARIES}
    )
    ```
    
    ##### 4.执行
    
    * 先启动服务端：rosrun 包名 服务端
    * 再调用客户端：rosrun 包名 客户端 参数1 参数2
    
    
    
    ### 参数服务器
    
    #### 参数服务器理论模型
    
    ![image-20250203194952796](C:/Users/Lenovo/AppData/Roaming/Typora/typora-user-images/image-20250203194952796.png)
    
    
    
    * ROS Master（管理者）
    * Talker（参数设计者）
    * Listener（参数调用者）
    
    ROS Master作为一个公共容器保存参数，Talker可以向容器中设置参数，Listener可以获取参数。
    
    
    
    **流程**
    
    * Talker通过RPC向参数服务器发送参数（参数名与参数值），Master将参数保存到参数列表中。
    
    * Listener通过RPC向参数服务器发送参数查找请求，请求
    
      中包含要查找的参数名。
    
    * Master根据Listener提供的参数名查找参数值，并将查询结果通过RPC发送给Listener。
    
    
    
    ### 常用命令
    
    #### **rosnode用于获取节点信息**
    
    ```shell
    rosnode ping 测试到节点的连接状态
    rosnode list 列出活动
    rosnode info 打印节点信息
    rosnode machine 列出指定设备上节点
    rosnode kill 杀死某个节点
    rosnode cleanup 清除不可连接的节点
    ```
    
    #### **rostopic**
    
    ```shell
    rostopic bw     显示主题使用的带宽
    rostopic delay  显示带有 header 的主题延迟
    rostopic echo   打印消息到屏幕
    rostopic find   根据类型查找主题
    rostopic hz     显示主题的发布频率
    rostopic info   显示主题相关信息
    rostopic list   显示所有活动状态下的主题
    rostopic pub    将数据发布到主题
    rostopic type   打印主题类型
    ```
    
    * rostopic list -v: 获取话题详情
    
    * **rostopic pub**
    
      直接向订阅者发布消息
    
      ```shell
      rostopic pub /主题名称 消息类型 消息内容
      rostopic pub /chatter std_msgs gagaxixi
      
      //发送小乌龟话题数据
      rostopic pub /turtle1/cmd_vel geometry_msgs/Twist
       "linear:
        x: 1.0
        y: 0.0
        z: 0.0
      angular:
        x: 0.0
        y: 0.0
        z: 2.0"
      ```
    
      以特定频率循环发送rostopic pub -r 频率 /主题名称 消息类型 内容
    
      ```shell
      //以10Hz频率发送
      rostopic pub -r 10 /turtle1/cmd_vel geometry_msgs/Twist
       "linear:
        x: 1.0
        y: 0.0
        z: 0.0
      angular:
        x: 0.0
        y: 0.0
        z: 2.0"
      ```

​					***输入完消息类型后双击“TAB”键，会自动出现消息成员***

#### 	**rosmsg显示消息**

```shell
rosmsg show 消息名称   显示消息描述
rosmsg info           显示消息信息
rosmsg list    		  列出ROS中所有消息
rosmsg md5             显示 md5 加密后的消息
rosmsg package 包名    显示某个功能包下的所有消息
rosmsg packages        列出包含消息的功能包
```

#### 	rosservice查询ROSServices

```shell
rosservice args    打印服务参数
rosservice call    使用提供的参数调用服务~~（可生成一组新的参数）
rosservice find    按照服务类型查找服务
rosservice info    打印有关服务的信息
rosservice list    列出所有活动的服务
rosservice type    打印服务类型
rosservice uri    打印服务的 ROSRPC uri
```

 * rosservice call 服务名

   ```
   rosservice call /spawn "x: 1.0
   y: 2.0
   theta: 0.0
   name: 'xxx'"
   name: "xxx"
   
   //新生成一只叫 xxx 的乌龟
   ```

   #### rossrv显示服务类型的信息

   ```shell
   rossrv show 消息名称    显示服务消息详情
   rossrv info    		   显示服务消息相关信息
   rossrv list    		   列出所有服务信息
   rossrv md5    		   显示 md5 加密后的服务消息
   rossrv package 包名     显示某个包下所有服务消息
   rossrv packages         显示包含服务消息的所有包
   ```



### 常用API

==ps：以下常用API代码部分只给出了关键部分，实际调用时需记得创建NodeHandle句柄！！！==

#### 命名空间

是一种层级化资源管理结构，用于组织节点、话题、服务和参数等资源。

实现资源隔离，不同命名空间下的同名资源不会冲突，/robot1和/robot2可各自独立运行同名的/sensor节点。

**全局命名空间**- - - 以/ 开头，全局唯一，可在任何位置访问；

**相对命名空间**- - - 无/前导，例sensor/topic，系统会根据默认命名空间自动补全路径；

**私有命名空间**- - - 以~开头，资源归属节点的私有空间；

#### 初始化init

```c++
/** @brief ROS初始化函数。
 *
 * 该函数可以解析并使用节点启动时传入的参数(通过参数设置节点名称、命名空间...)  
 *
 * \param argc 参数个数
 * \param argv 参数列表
 * \param name 节点名称，需要保证其唯一性，不允许包含命名空间
 * \param options 节点启动选项，被封装进了ros::init_options
 *
 */
void init(int &argc, char **argv, const std::string& name, uint32_t options = 0);
```

#### 话题与服务对象

话题和服务对象一般由NodeHandle创建，故应先实例化NodeHandle得到“nh”句柄。

##### 1.发布对象

```c++
/**
* \brief 根据话题生成发布对象
*
* 在 ROS master 注册并返回一个发布者对象，该对象可以发布消息
*
* 使用示例如下:
*
*   ros::Publisher pub = handle.advertise<std_msgs::Empty>("my_topic", 1);
*
* \param topic 发布消息使用的话题
*
* \param queue_size 等待发送给订阅者的最大消息数量
*
* \param latch (optional) 如果为 true,该话题发布的最后一条消息将被保存，并且后期当有订阅者连接时会将该消息发送给订阅者
*
* \return 调用成功时，会返回一个发布对象
*
* “M”为发布消息类型的泛型
*/
template <class M>
Publisher advertise(const std::string& topic, uint32_t queue_size, bool latch = false)

/**
* 发布消息
*/
template <typename M>
void publish(const M& message) const
```

##### 2.订阅对象

```c++
/**
   * \brief 生成某个话题的订阅对象
   *
   * 该函数将根据给定的话题在ROS master 注册，并自动连接相同主题的发布方，每接收到一条消息，	   都会调用回调
   * 函数，并且传入该消息的共享指针，该消息不能被修改，因为可能其他订阅对象也会使用该消息。
   * 
   * 使用示例如下:

void callback(const std_msgs::Empty::ConstPtr& message){...}
ros::NodeHandle nodeHandle;
ros::Subscriber sub = nodeHandle.subscribe("my_topic", 1, callback);
if (sub) // Enter if subscriber is valid
{
...
}

*
* \param M [template] M 是指消息类型
* \param topic 订阅的话题
* \param queue_size 消息队列长度，超出长度时，头部的消息将被弃用
* \param fp 当订阅到一条消息时，需要执行的回调函数
* \return 调用成功时，返回一个订阅者对象，失败时，返回空对象
* 
*/
template<class M>
Subscriber subscribe(const std::string& topic, uint32_t queue_size, 
                     void(*fp)(const boost::shared_ptr<M const>&), 
                     const TransportHints& transport_hints = TransportHints())
```

##### 3.服务对象

```c++
/**
* \brief 生成服务端对象
*
* 该函数可以连接到 ROS master，并提供一个具有给定名称的服务对象。
*
* 使用示例如下:

\verbatim
bool Foo::callback(std_srvs::Empty& request, std_srvs::Empty& response)
{
return true;
}
ros::NodeHandle nodeHandle;
Foo foo_object;
ros::ServiceServer service = nodeHandle.advertiseService("my_service", callback);
if (service) // Enter if advertised service is valid
{
...
}
\endverbatim

*
* \param service 服务的主题名称
* \param srv_func 接收到请求时，需要处理请求的回调函数
* \return 请求成功时返回服务对象，否则返回空对象:

*/
template<class MReq, class MRes>
ServiceServer advertiseService(const std::string& service, bool(*srv_func)(MReq&, MRes&))
```

##### 4.客户端对象

[3.1.2 话题与服务相关对象 · Autolabor-ROS机器人入门课程《ROS理论与实践》零基础教程](http://www.autolabor.com.cn/book/ROSTutorials/di-3-zhang-ros-tong-xin-ji-zhi-jin-jie/31/313-hua-ti-dui-xiang-chuang-jian.html)

#### 时间函数

==*PS：xxx.toSec()表示将Timer对象转变成double类型的浮点数；std::string(xxx)将xxx转换为字符串*==

##### 1.获取时刻 *==Time==*

​	*1970年01月01日为Unix纪元*

```c++
ros::Time right_now = ros::Time::now();//将当前时刻封装成对象
ROS_INFO("当前时刻:%.2f",right_now.toSec());//获取距离 1970年01月01日 00:00:00 的秒数
ROS_INFO("当前时刻:%d",right_now.sec);//获取距离 1970年01月01日 00:00:00 的秒数

ros::Time someTime(100,100000000);// 参数1:秒数  参数2:纳秒
ROS_INFO("时刻:%.2f",someTime.toSec()); //100.10
ros::Time someTime2(100.3);//直接传入 double 类型的秒数
ROS_INFO("时刻:%.2f",someTime2.toSec()); //100.30
```

##### 2.持续时间（设置一个时间间隔） *==Duration==*

```c++
ROS_INFO("当前时刻:%.2f",ros::Time::now().toSec());
ros::Duration du(10);//持续10秒钟,参数是double类型的，以秒为单位

du.sleep();//按照指定的持续时间休眠
ROS_INFO("持续时间:%.2f",du.toSec());//将持续时间换算成秒
ROS_INFO("当前时刻:%.2f",ros::Time::now().toSec());
```

##### 3.Time和Duration之间的计算

```c++
ros::Time now = ros::Time::now(); //Time
ros::Duration du1(10); //Duration
ros::Duration du2(20); //Duration
ROS_INFO("当前时刻:%.2f",now.toSec());

//1.time 与 duration 运算
ros::Time after_now = now + du1;
ros::Time before_now = now - du1;
ROS_INFO("当前时刻之后:%.2f",after_now.toSec()); //显示运算后时刻
ROS_INFO("当前时刻之前:%.2f",before_now.toSec());

//2.duration 之间相互运算
ros::Duration du3 = du1 + du2;
ros::Duration du4 = du1 - du2;
ROS_INFO("du3 = %.2f",du3.toSec());//显示运算后时间
ROS_INFO("du4 = %.2f",du4.toSec());
```

​	***PS: Time与Time之间不可运算***

##### 4.设置频率

```c++
ros::Rate rate(1);//指定频率
while (true)
{
    ROS_INFO("-----------code----------");
    rate.sleep();//休眠，休眠时间 = 1 / 频率。
}
/*
* 也可用ros::Duration du(1); 
*	   du.sleep();
* 延迟1s来实现控制频率为1Hz
*/
```

##### 5.定时器

```c++
 /**
* \brief 创建一个定时器，按照指定频率调用回调函数。
*	Timer createTimer(Duration period, const TimerCallback& callback, bool oneshot = 	*	false,bool autostart = true) const;

* \param period 时间间隔
* \param callback 回调函数
* \param oneshot 如果设置为 true,只执行一次回调函数;设置为 false,就循环执行。默认false
* \param autostart 如果为true，返回已经启动的定时器;设置为 false，需要手动启动。默认ture
*/

 // 也可以只传入两个参数nh.createTimer(ros::Duration(0.5),doSomeThing);其他保持默认
 ros::Timer timer = nh.createTimer(ros::Duration(0.5),doSomeThing,true);//只执行一次

/* 
* ros::Timer timer = nh.createTimer(ros::Duration(0.5),doSomeThing,false,false);
* 需要手动启动timer.start();
*/
 ros::spin(); //必须 spin

//回调函数
void doSomeThing(const ros::TimerEvent &event){
    ROS_INFO("-------------");
    ROS_INFO("event:%s",std::to_string(event.current_real.toSec()).c_str());
}
```

#### 回旋函数

##### 1.spinOnce()

```c++
/**
 * \brief 处理一轮回调
 *
 * 一般应用场景:
 *     在循环体内，处理所有可用的回调函数
 */
ROSCPP_DECL void spinOnce();
```

##### 2.spin()

```c++
/** 
 * \brief 进入循环处理回调 
 */
ROSCPP_DECL void spin();
```

##### 两者区别

**ros::spin()**一旦调用就会进入死循环，持续监听并处理回调队列中的信息，不会返回直到节点停止（处理频率完全依赖消息到达频率）。适用于只需被动接收消息且无需执行其他任务的节点（如纯订阅者）。

**ros::spinOnce()**每次调用时，仅处理当前回调队列中的所有消息一次，随后立即返回继续执行后续代码。可搭配ros::Rate和循环机构控制处理的频率。

![image-20250226140944115](C:/Users/Lenovo/AppData/Roaming/Typora/typora-user-images/image-20250226140944115.png)



#### 部分其他常用函数

##### 1.节点状态相关

发布实现时，循环发布消息的判断条件一般由节点状态来控制，C++中用ros::ok()判断节点状态是否正常。

**节点退出的原因：**收到了关闭信息（如ctrl+c）、同名节点启动、节点内部其他部分调用了节点关闭相关的API（C++中ros::shutdown()）



##### 2.日志相关

​	**日志等级：**

​		**DEBUG**(调试)：只在调试时使用，此类消息不会输出到控制台；

​		**INFO**(信息)：标准消息，一般用于说明系统内正在执行的操作；

​		**WARN**(警告)：提醒一些异常情况，但程序仍然可以执行；

​		**ERROR**(错误)：提示错误信息，此类错误会影响程序运行；

​		**FATAL**(严重错误)：此类错误将阻止节点继续运行。

```c++
ROS_DEBUG("hello,DEBUG"); //不会输出
ROS_INFO("hello,INFO"); //默认白色字体
ROS_WARN("Hello,WARN"); //默认黄色字体
ROS_ERROR("hello,ERROR");//默认红色字体
ROS_FATAL("hello,FATAL");//默认红色字体
```

### 头-源文件调用

#### 自定义头文件调用

##### 1.头文件hello.h

```c++
#ifndef _HELLO_H
#define _HELLO_H

namespace hello_ns{  //使用命名空间囊括所需的类 调用类时应具体说明hello_ns::HelloPub

class HelloPub {

public:
    void run();//声明成员函数
};
}
#endif
```

##### 2.源文件hello.cpp

```c++
#include "ros/ros.h"
#include "test_head/hello.h"

namespace hello_ns {

//定义成员函数
void HelloPub::run(){
    ROS_INFO("自定义头文件的使用....");
}

}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"test_head_node");
    hello_ns::HelloPub helloPub;
    helloPub.run();
    return 0;
}
```

##### 3.配置文件

加入cmakelist中的头文件目录

```
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)
```

配置可执行文件

​	添加可执行、添加依赖、连接依赖

#### 自定义源文件调用

与《自定义头文件调用》步骤类似，但是对应的hello.cpp文件只用来编写成员函数的定义。再用一个.cpp文件专门作为可执行文件去执行相应函数。

头文件&源文件配置：

```c++
include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

## 声明C++库 给hello.h库取名head
add_library(head
  include/test_head_src/hello.h
  src/haha.cpp
)

add_dependencies(head ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(head
  ${catkin_LIBRARIES}
)
```

可执行文件配置：

​	和普通可执行文件配置一致，在连接依赖的时候，要将可执行点与head连接。





### 运行管理

#### 元功能包相关

MetaPackge是一个文件管理系统的概念。是ROS中的一个虚包，里面没有实质性的内容，但是他依赖了其他的软件包。

可以是一个索引目录，告诉我们这个包集合中有哪些子包，并且该去哪里下载。

**相关实现**

```xml
<exec_depend>被集成功能包的名称</exec_depend>exec_depend>
.....
<export>
	<metapackage />
</export>
```

修改CMakeList.txt的内容如下：

```
cmake_minimum_required(VERSION 3.0.2)
project(demo)
find_package(catkin REQUIRED)
catkin_metapackage()
```

http://wiki.ros.org/catkin/package.xml#Metapackages



#### launch文件的xml相关

###### **1.launch根标签**

​	<launch>是所有launch文件的根标签，其他所有标签都是launch的子级。

###### 2.node标签

​	<node>用于指定节点。But!! roslaunch命令不能保证按照node的声明顺序来启动节点（节点的启动是多进程的）；

**pkg='****包名****'**			节点所属的包
**type='**node Type**'**				  节点类型（节点类型
**name='**nodeName**'**   			节点名称
**args="**xxx**"**						   将参数传递给节点
**machine="**机器名**"**		      在指定机器上启动节点
**respawn="**ture | flase**"**	      如果节点退出，是否自动重启
**respawn_delay='**N**'**		       如果respawn打开了，那么延迟N秒后启动节点
**require="**ture | false**"**            该节点是否必须，如果为必须（ture），则节点退出后将杀死整个roslaunch
**ns="**xxx**"**					           在指定命名空间xxx中启动节点**—>**节点名称变为
**clear_params="**ture | false**"** 在启动前，删除节点的私有空间的所有参数
**output="**log | screen**"**			日志发送目标，可以设置为：log日志文件，screen屏幕；默认log



**ex：**

将一个节点加入launch文件中使其roslaunch后能自动启动

```xml
<launch>
  <!-- 启动 my_node -->
  <node
    pkg="my_package"
    type="my_node"
    name="my_node"
    output="screen"
    args="--param1 value1 --param2 value2"<!-- 传入的参数-->
    />
</launch>
```



###### 3.include标签

用于将另一个xml格式launch文件导入到当前文件

* **file=“**$(find 包名)/xxx/xxx.launch**”**

​	  要包含的文件路径

* **ns=“**xxx**”**（可选） 在指定命名空间导入文件

###### 4.remap标签

用于话题重命名

*  **from="**xxx**"** 原始话题名称
* **to="**yyy**"** 目标名称

###### 5.param标签

用于在参数服务器上设置参数，参数源可以在标签中通过value指定，也可以通过外部文件加载，在`<node>`标签中时，相当于私有命名空间。

* **name="**命名空间/参数名**"**	参数名称、可以包含命名空间
* **value="**xxx**"**（可选） 定义参数值，如果此处省略，必须指定外部文件作为参数源。
* **type="**str | int | double | bool | yaml**"** （可选） 制定参数类型，如果未指定，roslaunch会尝试确定参数类型

###### 6.rosparam标签

可以从YAML文件导入参数，或将参数导出到YAML文件，也可以用来删除参数。在<node>标签中视为私有。

* **command="**load | dump | delete **"**  (默认load) 加载、导出或删除参数
* **file="$(**find xxxxx)/xxx/yyy...**"**   加载或导出到的yaml文件
* **param="**参数名称**"**
* **ns="**命名空间**"**    (可选)





### TF坐标转换

帮助进行坐标变换

**坐标转换常用的msg:** 

`geometry_msgs/TranformStamped`  用于传输坐标系相关位置信息  （含有三轴偏移量和四元数）

`geometry_msgs/PointStamped`		用于传输某个坐标系内坐标点的信息	（三轴坐标）



#### 静态坐标变换

所谓静态坐标变换，是指两个坐标系之间的相对位置是固定的。

##### 实现分析：

1.坐标系相对关系，通过发布方发布（geometry_msgs/TranformStamped）

2.订阅方，订阅到发布方的坐标系相对关系，再传入坐标点信息，然后借助tf实现坐标变换。

##### 相关实现

项目功能包依赖tf2 , tf2_ros , tf2_geometry_msgs , roscpp , std_msgs , geometry_msgs

**发布方**

```cpp
/* 
    静态坐标变换发布方:
        发布关于 laser 坐标系的位置信息 

    实现流程:
        1.包含头文件
        2.初始化 ROS 节点
        3.创建静态坐标转换广播器
        4.创建坐标系信息
        5.广播器发布坐标系信息
        6.spin()
*/
#include "ros/ros.h"
#include "tf2_ros/static_transform_broadcaster.h" //tf2的静态坐标广播器
#include "geometry_msgs/TransformStamped.h" //要发布的坐标消息类型
#include "tf2/LinearMath/Quaternion.h" 	//tf2转换相关

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ROS 节点
    ros::init(argc,argv,"static_brocast");
    // 3.创建静态坐标转换广播器
    tf2_ros::StaticTransformBroadcaster broadcaster;
    // 4.创建坐标系信息
    geometry_msgs::TransformStamped ts;
    //----设置头信息
    ts.header.seq = 100;
    ts.header.stamp = ros::Time::now();
    ts.header.frame_id = "base_link";
    //----设置子级坐标系
    ts.child_frame_id = "laser";
    //----设置子级相对于父级的偏移量
    ts.transform.translation.x = 0.2;
    ts.transform.translation.y = 0.0;
    ts.transform.translation.z = 0.5;
    //----设置四元数:将 欧拉角数据转换成四元数
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,0);
    ts.transform.rotation.x = qtn.getX();
    ts.transform.rotation.y = qtn.getY();
    ts.transform.rotation.z = qtn.getZ();
    ts.transform.rotation.w = qtn.getW();
    // 5.广播器发布坐标系信息
    broadcaster.sendTransform(ts);
    ros::spin();
    return 0;
}
```

**订阅方**

```cpp
/*  
    订阅坐标系信息，生成一个相对于 子级坐标系的坐标点数据，转换成父级坐标系中的坐标点

    实现流程:
        1.包含头文件
        2.初始化 ROS 节点
        3.创建 TF 订阅节点
        4.生成一个坐标点(相对于子级坐标系)
        5.转换坐标点(相对于父级坐标系)
        6.spin()
*/
//1.包含头文件
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h" //tf数据
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h" //注意: 调用 transform 必须包含该头文件

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    // 2.初始化 ROS 节点
    ros::init(argc,argv,"tf_sub");
    ros::NodeHandle nh;
    // 3.创建 TF 订阅节点，接收tf广播信息
    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    ros::Rate r(1);
    while (ros::ok())
    {
    // 4.生成一个坐标点(相对于子级坐标系)，手动输入模拟
        geometry_msgs::PointStamped point_laser;
        point_laser.header.frame_id = "laser";
        point_laser.header.stamp = ros::Time::now();
        point_laser.point.x = 1;
        point_laser.point.y = 2;
        point_laser.point.z = 7.3;
    // 5.转换坐标点(相对于父级坐标系)
        //新建一个坐标点，用于接收转换结果  
        //--------------使用 try 语句或休眠，否则可能由于缓存接收延迟而导致坐标转换失败------------------------
        try
        {
            geometry_msgs::PointStamped point_base;
            point_base = buffer.transform(point_laser,"base_link");//接收转换后的数据
            ROS_INFO("转换后的数据:(%.2f,%.2f,%.2f),参考的坐标系是:",point_base.point.x,point_base.point.y,point_base.point.z,point_base.header.frame_id.c_str());

        }
        catch(const std::exception& e)
        {
            // std::cerr << e.what() << '\n';
            ROS_INFO("程序异常.....");
        }
        r.sleep();  
        ros::spinOnce();
    }
    return 0;
}
```

**ps1:**

坐标系之间相对位置固定时，所需参数也是固定的：父系坐标名称、子级坐标系名称、x偏移量、y偏移量、z偏移量、roll、pitch、yaw。

ros已经封装好了专门的节点：

`rosrun tf2_ros static_transform_publisher x偏移量 y偏移量 z偏移量 z偏航角度 y俯仰角度 x翻滚角度 父级坐标系 子级坐标系`

**PS2：**

可借助rviz显示坐标系关系：

* Fixed Frame设置为base_link；
* add中选择TF组件，即可显示坐标关系；
