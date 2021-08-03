# AUBO_ROS

#### 介绍
基于ROS系统对真实的AUBO机器人进行控制（利用AUBO-API，而不是moveit!）。

#### 软件架构
real_aubo_control.cpp


#### 安装与使用

1.  下载aubo的ros功能包，并编译通过，可删除不用的文件再编译，下图是我保留的文件。
![仅保留的文件](https://images.gitee.com/uploads/images/2020/1124/132730_549bac91_7474817.png "2020-11-24 13-27-13屏幕截图.png")
2.  将real_aubo_control.cpp放置在aubo_driver/src文件夹内。
3.  在aubo_driver/cmakelist.txt中添加以下三行：

add_executable(real_aubo_control src/real_aubo_control.cpp)

target_link_libraries(real_aubo_control ${catkin_LIBRARIES} auborobotcontroller otgLib)

add_dependencies(real_aubo_control aubo_msgs_gencpp)

4. 将aubo_driver/src/aubo_driver.cpp中第515行改为（更改ip地址）
    server_host_ = (s=="")? "192.168.1.5" : s;

5.  进入工作空间再次编译。
6.  （开启ros master并与机械臂Ping通的前提下）rosrun aubo_driver real_aubo_control

