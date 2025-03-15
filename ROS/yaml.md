### yaml文件传输参数

程序中参数量大，开发过程中需要对参数进行修改调试，避免反复编译。

参数需要写入到ros中的master中，参数文件较少时可直接在launch中加载（<param>标签中value指定)

参数文件较多时从yaml.cpp文件中加载（<rosparam>标签中file中指定文件)



* 从launch中加载

  ```xml
  <!-- 参数名name 参数类型string 参数内容frank -->
  <param name="name" type="string" value="frank"/>
  ```

* 从yaml文件中加载

  launch:
  
  ```xml
  							<!-- launch文件 -->
  <?xml version="1.0"?>
  <launch>
  	<!-- 参数加载到全局 使用params.yaml文件进行加载 路径直接就是/params.yaml -->
      <rosparam command="load" file="$(find param)/params.yaml"/>
      
      <node pkg= "param_node" type="param_node" name="param_node" output="screen">
          
          <!-- 参数加载到param_node内 采用param的value属性直接赋予 -->
      	<param name="name" type="string" value="frank"/>
          
      </node>    
  </launch>
  ```
  
  yaml:
  
  ```yaml
  param1: 1
  param2: 2
  param3: 3
  ```
  
  

#### 参数读取方法 

```cpp
int main(int argc, char **argv)
{
    ros::init(argc, argv, "param_test_node");
    ros::NodeHandle nh;
    int parameter1, parameter2, parameter3;

    // ① ros::param::get()获取参数“param1”的value，写入到parameter1上
    bool ifget1 = ros::param::get("param1", parameter1);

    // ② ros::NodeHandle::getParam()获取参数，与①作用相同
    bool ifget2 = nh.getParam("param2", parameter2);

    // ③ ros::NodeHandle::param()类似于①和②, 如果get不到指定的param，设置为默认值(如33333)
    nh.param("param3", parameter3, 33333); 
    // parameter3 = nh.param("param3", 33333); //这两种方式都可以获取参数

    if (ifget1)
        ROS_INFO("Get param1 = %d", parameter1);
    else
        ROS_WARN("Didn't retrieve param1");
    if (ifget2)
        ROS_INFO("Get param2 = %d", parameter2);
    else
        ROS_WARN("Didn't retrieve param2");
    if (nh.hasParam("param3"))
        ROS_INFO("Get param3 = %d", parameter3);
    else
        ROS_WARN("Didn't retrieve param3");

}
```

**ps:** api中用于指向对应参数的"param1"等，可以是参数的全局路径"/param1"也可以是参数名"param1"