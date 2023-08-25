# 1.基础知识
## 1.1工作空间/功能包/节点建立

### 1.1.1工作空间建立
（1）创建工作空间 catkin_ws
~~~shell
mkdir -p ~/catkin_ws/src	%创建src文件，放置功能包源码
cd ~/catkin_ws/src			%进入src文件夹
catkin_init_workspace		%初始化文件夹
~~~
（2）编译工作空间 catkin_make
~~~shell
catkin_make
~~~
（3）设置环境变量
echo “source ~/工作空间/devel/setup.bash” >> ~/.bashrc   。也可以直接打开 ~/.bashrc 这个文件手动添加和删除source


### 1.1.2功能包建立
（1）catkin_create_pkg [包名] [依赖1] [依赖2] ...
（2）返回工作空间路径，编译catkin_make
（3）echo “source ~/工作空间/devel/setup.bash” >> ~/.bashrc   。也可以直接打开 ~/.bashrc 这个文件手动添加和删除source

### 1.1.3 节点建立
（1）在package.xml文件中添加需要依赖的包`<build_depend>...</build_depend>`和`<run_depend>...</run_depend>`。一般来说都要包括以下内容
~~~xml
<build_depend>roscpp</build_depend>
<build_depend>std_msgs</build_depend>
<!-- 旧版本是 -->
<run_depend>roscpp</run_depend>
<run_depend>std_msgs</run_depend>
<!-- 新版本替换run_depend为exec_depend -->
<exec_depend>roscpp</exec_depend>
<exec_depend>std_msgs</exec_depend>
~~~

（2）编写节点文件

~~~C++
#include <ros/ros.h>			//这个是必须的
#include <std_msgs/Float64.h>	//这里取决于程序中使用的功能

int main(int argc,char **argv){
	ros::init(argc,argv,"节点名称");
	ros::NodeHandle n;
}
~~~

（3）在CMakeList.txt中添加内容

~~~c++
finde_package(catkin REQUIRED COMPONENTS
roscpp
std_msgs	//这里需要添加所有要用的库
)

include_directories(
${catkin_INCLUDE_DIRS}
)

add_executable([可执行文件名] src/[对应的C++文件])	//使用时要把[...]带括号整个替换

target_link_libraries([可执行文件名] ${catkin_LIBRARIES})	//这一部是把可知性文件和库链接起来，到时候才能找到可执行文件执行
~~~



## 1.2消息(topic)
### 1.2.1自定义消息
自定义消息是需要以下几类操作
（1）添加.msg文件，在其中输入自定义消息相关内容。
（2）编辑包中的package.xml文件，添加
		<build_depend>message_generation</build_depend>

​		注意观察，如果文档开头是<package format="2">，则添加以下代码

​		<exec_depend>message_runtime</exec_depend>

​		如若不是，可能需要添加以下代码

​		<run_depend>message_runtime</run_depend>

（3）编辑包中的CMakeLists.txt文件，根据注释在对应位置添加

​		find_package(.... message_generation)

​		add_message_files(FILES 名字.msg)

​		generate_messages(DEPENDENCIES std_msgs)

​		catkin_package(CATKIN_DEPENDS message_runtime)

（4）之后直接编译包（**注意把CMakeLists.txt中的其他使用这个自定义消息的节点文件先注释了，以免干扰**）。

（5）之后在工作空间的 ~/devel/include/包名/ 下可以找到这个自定义消息对应的头文件，之后包含这个头文件就可以了。

​		#include <包名/名字.h>
### 1.2.2自定义变长度消息
在.msg文件中使用类似`float64[] dbl_vec`的语句，其余和普通自定义消息类似。

定义的可变长度消息是一个类对象，其中有以下成员函数/方法：

（1）`msg.dbl_vec[0]`：访问第一个元素

（2）`msg.dbl_vec.resize(n)`：重新定义消息长度

（3）`msg.dbl_vec.push_back(value)`：向队尾添加新的变量

（4）还有很多成员函数，大致功能和C++中的向量类似

## 1.3服务(service)
相比于话题（topic），是一对一、双向的通信。**因为客户端请求服务时会暂停直至应答返回，因此服务中的通信应该快速、简洁**。
### 1.3.1自定义服务

（1）添加`.srv`文件，在其中输入自定义消息相关内容。
~~~ros
string name
---
bool on_the_list
bool good_guy
int32 age
string nickname
~~~
其中`---`起分割作用，上边表示请求结构，下边表示响应结构。

（2）编辑包中的package.xml文件，添加
		<build_depend>message_generation</build_depend>

​		注意观察，如果文档开头是<package format="2">，则添加以下代码

​		<exec_depend>message_runtime</exec_depend>

​		如若不是，可能需要添加以下代码

​		<run_depend>message_runtime</run_depend>

（3）编辑包中的CMakeLists.txt文件，根据注释在对应位置添加

​		find_package(.... message_generation)

​		add_message_files(FILES 名字.srv)

​		generate_messages(DEPENDENCIES std_msgs)

​		catkin_package(CATKIN_DEPENDS message_runtime)

（4）之后直接编译包（**注意把CMakeLists.txt中的其他使用这个自定义消息的节点文件先注释了，以免干扰**）。

（5）之后在工作空间的 ~/devel/include/包名/ 下可以找到这个自定义消息对应的头文件，之后包含这个头文件就可以了。

​		#include <包名/名字.h>

​		**特别的是，相比于话题，服务会生成三个头文件，其中有两个是“Request”和“Response”，用的时候只需要包含总的那个头文件即可**。

### 1.3.2在新的包中使用已有的话题和服务
（1）直接包含已有话题和服务的头文件
（2）新包的`package.xml`文件需要包含对旧包的依赖，使用如下语句`<build_depend>旧包名</build_depend>`
### 1.3.3 服务节点
（1）包含对应的头文件，一般为如下格式`#include <包名/头文件名.h>

（2）`ros::ServiceServer my_SrvServer_object = n.advertiseService("service_mySrv",callBack);`==注意这里没有模板，不需要传入服务的数据类型==

（3）写回调函数
~~~C++
bool callBack(example_ros_msg::example_SrvMsgsRequest& request,
                example_ros_msg::example_SrvMsgsResponse& response){

    string in_name(request.name);   //request.name中存放传入请求的内容，自己定义的

    response.on_the_list = false;
    if(in_name.compare("Bob")==0){
        ROS_INFO("asked about Bob");
        response.age=32;
        response.good_guy=false;
        response.nickname="boomber";
        response.on_the_list=true;
    }
    return true;
}
~~~



# 2. 常用函数及命令
## 2.1 常用函数
（1）`ros::spin()`：一旦进入spin函数，它就不会返回了，也不继续往后执行了，相当于它在自己的函数里面死循环了（直到ctrl+c　或者程序终止的时候才退出）。**所以一般在书函数末尾加**

（2）`ROS_INFO()`：
+ ROS_INFO_STREAM(“Hello ROS”),输出字符串
+ ROS_INFO(“s%”, msg.data.c_str()),输出一个字符串变量
+ ROS_INFO(“I heard: [s%]”, msg.data.c_str()),输出一个字符串变量，这里的中括号不是必须的，输出时会直接显示这个中括号
+ ROS_INFO(“I heard: [s%]”, msg->data.c_str()),输出一个指针变量
+ ROS_INFO(“Publish Person Info: name:%s age:%d sex:%d”,person_msg.name.c_str(), person_msg.age, person_msg.sex)，按数据类型输出


## 2.2 常用命令
`rostopic list`：打印当前所有话题
`rostopic echo 话题路径名`：打印话题内容到窗口
`rosservice list`：打印当前所有服务
`rosservice info 服务名`：打印对应服务的信息
`rosmsg show 包名/消息名`：打印这个消息类型的数据构成结构
`check_urdf links.urdf`：检查机器人描述文件urdf的语法格式是否正确和信息

# 3. 仿真、感知、可视化
## 3.1 统一的机器人描述格式（URDF）
描述一个用以仿真的机器人，需要定义四个方面的信息：运动学模型、动力学模型、碰撞模型、视觉模型。即一个`link`字段下包含`collision`、`visual`、`inertial`，`joint`字段写在`link`字段外面。

运行指令`check_urdf links.urdf`可检查urdf文件语法和信息。

~~~xml
<?xml version="1.0"?>
<robot  name="one_DOF_robot">

<!-- Used for fixing robot to Gazebo 'base_link' -->
  <link name="world"/>

  <joint name="glue_robot_to_world" type="fixed">
    <parent link="world"/>
    <child link="link1"/>
  </joint>

 <!-- Base Link -->
  <link name="link1">
    <collision>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
       <box size="0.2 0.2 0.7"/>
      </geometry>
    </collision>

    <visual>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <box size="0.2 0.2 1"/>
      </geometry>
    </visual>

    <inertial>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia
        ixx="1.0" ixy="0.0" ixz="0.0"
        iyy="1.0" iyz="0.0"
        izz="1.0"/>
    </inertial>
  </link>

<!-- Moveable Link -->
  <link name="link2">
    <collision>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
        <!--box size="0.15 0.15 0.8"-->
      </geometry>
    </collision>

    <visual>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <geometry>
        <cylinder length="1" radius="0.1"/>
      </geometry>
    </visual>

    <inertial>
      <origin xyz="0 0 0.5" rpy="0 0 0"/>
      <mass value="1"/>
      <inertia
        ixx="0.1" ixy="0.0" ixz="0.0"
        iyy="0.1" iyz="0.0"
        izz="0.005"/>
    </inertial>
  </link>

  <joint name="joint1" type="continuous">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="0 0 1" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
  </joint>
</robot>
~~~
### 3.1.1 运动学模型：joint字段
link1是基础坐标系，link2是link1的子级。两个杆通过joint1链接。

joint字段中origin是描述了关节坐标系的信息。关节坐标系和父杆link1相关联，xyz描述了关节坐标系相对于link1参考坐标系的位移;rpy用欧拉角（俯仰偏航滚转）描述了关节坐标系相对于link1参考坐标系的旋转。

`<axis xyz="0 1 0"/>`指定了关节轴与关节坐标系的y轴共线。

定义关节`"glue_robot_to_world"`时，因为是令link1和世界坐标系静态链接，因此省略了xyz和rpy的描述。

### 3.1.2 视觉模型：visual字段
在`visual`字段中`<origin xyz="0 0 0.5" rpy="0 0 0"/>`指定了视觉模型在link1坐标系中的位置，这里相当于让图形远点在link11坐标系上方0.5m处。

`<box size="0.2 0.2 1"/>`指定了视觉图形是一个长宽高为对应值的长方体。

`<cylinder length="1" radius="0.1"/>`指定了一个对应长度和半径的圆柱体。

### 3.1.3 动力学模型：inertial字段
在`inertial`字段中`<origin xyz="0 0 0.5" rpy="0 0 0"/>`指定了质心的位置。`inertia`指定了转动惯量的值。`mass`指定了质量。

注意：`mass`和`inertia`中的ixx\iyy\izz不能等于0，不然仿真时会出现除以0的情况。

### 3.1.4 碰撞模型：collision字段
`collision`字段中`<box size="0.2 0.2 0.7"/>`指定了碰撞模型的大小。`<origin xyz="0 0 0.5" rpy="0 0 0"/>`指定了碰撞模型的位置。

一般来说碰撞模型的`box`和`origin`是和视觉模型一样的，但是如果关节处不留有间隙，则会一直计算两个部件之间的碰撞。代码中让`link1`的碰撞模型稍短了一些就是避免`link1`和`link2`之间在关节处的频繁碰撞，因为此处关节碰撞不是关键。

并不是所有情况都可以缩小碰撞模型，比如做一些精致的仿生手模型时，指头关节之间的碰撞是必须仔细考虑的。

## 3.2 Gazebo仿真
`roslaunch gazebo_ros enpty_world.launch`指令启动一个空的世界环境。

### 3.2.1 导入模型
`rosrun gazebo_ros spawn_model ...`gazebo_ros包中spawn_model可以导入模型到gazebo中。也可以**编写launch文件批量导入**。

------
导入一个urdf模型
~~~shell
rosrun gazebo_ros spawn_model -urdf -file minimal_robot.urdf -model one_DOF
~~~
指令传入的参数主要包含三部分：1）模型文件为urdf的声明。2）加载文件路径。3）模型加载后在gazebo中名称。

------
也可以通过Gazebo GUI中的Insert来手动插入模型。

### 3.2.2 最小关节控制
核心在于使用**关节执行器命令**和**关节位移传感器**来施加控制。
可以用专门的Gazebo的控制插件，没有必要一定编写控制节点(node)来实现控制。

------
编写node节点文件控制，在节点中使能两个服务客户端：1）/gazebo/apply_joint_effort。2）/gazebo/get_joint_properties。

------
Gazebo控制插件使用步骤：
**（1）在`joint`字段添加力矩和关节限制**
每个关节都要添加

~~~xml
<joint name="joint1" type="revolute">
	...
	...
	<limit effort="10.0" lower="0.0" upper="2.0" velocity="0.5"/>
	<dynamics damping="1.0"/>
</joint>
~~~
这里joint的参数`revolute`相比于之前的`continuous`更适合机械臂，因为机械关节基本都有运动范围限制。

`<limit effort="10.0" lower="0.0" upper="2.0" velocity="0.5"/>`：分别表示1）力矩限制10Nm。2）运动限制的上下关节范围。3）速度限制0.5rad/s。

`<dynamics damping="1.0"/>`：表示线性摩擦阻尼，1.0(Nm)/(rad/s)

**（2）添加transmission块**
每一个受控关节都要有对应的transmission块。
~~~xml
<transmission name="tran1">
    <type>transmission_interface/SimpleTransmission</type>
    <joint name="joint1">
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </joint>
    <actuator name="moter1">
      <mechanicalReduction>1.0</mechanicalReduction>
      <hardwareInterface>hardware_interface/PositionJointInterface</hardwareInterface>
    </actuator>
</transmission>
~~~
`<joint name="joint1">`：这里关节名必须和前面定义的关节相同。

`<type>transmission_interface/SimpleTransmission</type>`和`<hardwareInterface>hardware_interface/PositionJointInterface</hardwareIn`不用更改

`<mechanicalReduction>1.0</mechanicalReduction>`：设置传动率为单位1。机器人动力学中这个值在100到1000都常见。

**（3）使能Gazebo插件控制器**
~~~xml
<gazebo>
    <plugin name="gazebo_ros_control" filename="libgazebo_ros_control.so">
      <robotNamespace>/one_DOF_robot</robotNamespace>
    </plugin>
</gazebo>
~~~
引入插件库`libgazebo_ros_control.so`。

`<robotNamespace>/one_DOF_robot</robotNamespace>`：仿真器的namespace，对于不同的机器人最好分到不同的命名空间。

**（4）控制增益文件yaml**
~~~yaml
one_DOF_robot:	# 必须和之前urdf文件中机器人的名字一样
  # Publish all joint states -----------------------------------
  joint_state_controller:
    type: joint_state_controller/JointStateController
    publish_rate: 50  
  
  # Position Controllers ---------------------------------------
  joint1_position_controller:
    type: effort_controllers/JointPositionController
    joint: joint1	#这段代码每个关节都需要复制写一遍，这里名字和urdf中关节名相同
    pid: {p: 10.0, i: 10.0, d: 10.0, i_clamp_min: -10.0, i_clamp_max: 10.0} #最后两个参数抗积分饱和
~~~

**（5）launch文件**
~~~xml
<launch>
  <!-- Load joint controller configurations from YAML file to parameter server -->
  <rosparam file="$(find ctrl_test)/src/control_params/one_dof_ctl_params.yaml"command="load"/>
  <param name="robot_description" 
     textfile="$(find ctrl_test)/model/minimal_robot_description/minimal_robot_description.urdf"/>

  <!-- Start Gazebo -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch" />
  
  <!-- Spawn a robot into Gazebo -->
  <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" 
     args="-param robot_description -urdf -model one_DOF_robot" />

  <!--start up the controller plug-ins via the controller manager -->
  <node name="controller_spawner" pkg="controller_manager" type="spawner" respawn="false"
    output="screen" ns="/one_DOF_robot" args="joint_state_controller joint1_position_controller"/>
    
</launch>
~~~

# 4. 移动机器人
四元数讲解：https://www.3dgep.com/understanding-quaternions/

三个核心要素：
+ 期望状态生成器（路径规划）：生成可行的状态序列引导机器人通过期望路径。
+ 机器人状态估计器：估计机器人状态与期望状态之间偏差。
+ 转向控制器：根据期望与机器人状态之间偏差纠正机器人动作。

