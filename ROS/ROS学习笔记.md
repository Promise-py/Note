# ROS学习笔记

## 发布者Publisher的编程实现

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
          

## 服务和参数

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
    
    * Listener通过RPC向参数服务器发送参数查找请求，请求中包含要查找的参数名。
    
    * Master根据Listener提供的参数名查找参数值，并将查询结果通过RPC发送给Listener。
    
    
    
    



