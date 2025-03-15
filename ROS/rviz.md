### Rviz可视化

对一项目第一次启动rviz时，需要将大地坐标"Fixed Frame"、机器人模型"RobotModel"等导入设置好。

##### 1.保存配置文件

先设置好查看的话题，大地坐标系，世界，机器人模型等参数。

用Ctrl+Shift+S保存配置文件至功能包中对应的config文件夹中，命名为xxx.rviz

##### 2.在launch文件中加入自动打开rviz的node

```xml
<launch>
	<node
          name="rviz"
          pkg="rviz"
          type="rviz"
          args="-d $(find your_node)/相对路径/xxx.rviz"
          />
</launch>
```

##### 3.启动launch文件后rviz将自动启动并配置完成