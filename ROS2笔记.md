# 参考文档
https://fishros.com/d2lros2/#/
# 1. 基础
## 1.1 基础概念
节点之间通过四种手段交互：
+ 话题-topics
+ 服务-services
+ 动作-Action
+ 参数-parameters

## 1.2 常用指令
### 1.2.1 基础指令
启动一个节点：

有可能需要`source install/setup.bash`

~~~shell
ros2 run <package_name> <executable_name>
~~~
指令意义：启动 包下的 中的节点。

------
查看节点列表(常用)：
~~~shell
ros2 node list
~~~

------
查看节点信息(常用)：
~~~shell
ros2 node info <node_name>
~~~

------
重映射节点名称
~~~shell
ros2 run turtlesim turtlesim_node --ros-args --remap __node:=my_turtle
~~~

------
运行节点时设置参数
~~~shell
ros2 run example_parameters_rclcpp parameters_basic --ros-args -p rcl_log_level:=10
~~~
### 1.2.2 和功能包相关指令
创建功能包
~~~shell
ros2 pkg create <package-name>  --build-type  {cmake,ament_cmake,ament_python}  --dependencies <依赖名字>
~~~

------
列出可执行文件
~~~shell
ros2 pkg executables #列出所有
ros2 pkg executables turtlesim	#列出turtlesim功能包下所有
~~~

------
列出所有的包
~~~shell
ros2 pkg list
~~~

------
输出某个包所在路径的前缀
~~~shell
ros2 pkg prefix  <package-name>
~~~

### 1.2.3 和接口相关指令
查看接口列表
~~~
ros2 interface list
~~~

------
查看某一个接口详细的内容
~~~
ros2 interface show std_msgs/msg/String
~~~

------
详细查看一个参数的信息
~~~
ros2 param describe <node_name> <param_name>
~~~

------
获取参数的值
~~~
ros2 param get <node_name> <param_name>
~~~

## 1.3 Colcon构建工作空间
创建一个文件夹，在文件夹下创建`src`文件夹。**使用`colcon build`指令编译工程**后，会在`src`同级目录下生成如下文件夹：
+ `build` 目录存储的是中间文件。对于每个包，将创建一个子文件夹，在其中调用例如CMake。
+ `install` 目录是每个软件包将安装到的位置。默认情况下，每个包都将安装到单独的子目录中。因此需要==`source install/setup.bash`==才能找到编译后的资源。
+ `log` 目录包含有关每个colcon调用的各种日志信息。

-----
只编译一个包
~~~shell
colcon build --packages-select YOUR_PKG_NAME 
~~~

------
不编译测试单元
~~~shell
colcon build --packages-select YOUR_PKG_NAME  --cmake-args -DBUILD_TESTING=0
~~~

------
运行编译的包的测试
~~~shell
colcon test
~~~

------
**允许通过更改src下的部分文件来改变install（重要）**
每次调整 python 脚本时都不必重新build了

~~~shell
colcon build --symlink-install
~~~

## 1.4 创建功能包
-------
**使用RCLCPP**
创建example_cpp功能包，使用ament-cmake作为编译类型，并为其添加rclcpp依赖。

~~~shell
cd 工作空间/src
ros2 pkg create example_cpp --build-type ament_cmake --dependencies rclcpp
~~~
+ `pkg create` 是创建包的意思
+ `--build-type` 用来指定该包的编译类型，一共有三个可选项ament_python、ament_cmake、cmake
+ `--dependencies` 指的是这个功能包的依赖，这里小鱼给了一个ros2的C++客户端接口rclcpp

------
**使用RCLPY**
~~~shell
cd 工作空间/src
ros2 pkg create example_py  --build-type ament_python --dependencies rclpy
~~~

## 1.5 创建节点
-------
**使用RCLCPP**

~~~C++
#include "rclcpp/rclcpp.hpp"

/*
    创建一个类节点，名字叫做Node03,继承自Node.
*/
class Node03 : public rclcpp::Node
{

public:
    // 构造函数,有一个参数为节点名称
    Node03(std::string name) : Node(name)
    {
        // 打印一句
        RCLCPP_INFO(this->get_logger(), "大家好，我是%s.",name.c_str());
    }

private:
   
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*产生一个node_03的节点*/
    auto node = std::make_shared<Node03>("node_03");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
~~~
修改一下CMakeLists.txt,将其添加为可执行文件，并使用install指令将其安装到install目录。

在CmakeLists.txt最后一行加入下面两行代码。
~~~cmake
#先使用find_package寻找需要依赖的包，需要几个找几个
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(可执行文件名 src/需要编译的文件.cpp)
ament_target_dependencies(可执行文件名 rclcpp 需要依赖的包2 ...)#这里需要看当前这个可执行文件需要依赖哪几个包

#奖可执行文件安装到install目录下
install(TARGETS
  可执行文件名	
  DESTINATION lib/${PROJECT_NAME}
)
~~~

-------
**使用RCLPY**
~~~python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class Node04(Node):
    """
    创建一个Node04节点，并在初始化时输出一个话
    """
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)


def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = Node04("node_04")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy
~~~
接着修改`setup.py`
~~~python
entry_points={
        'console_scripts': [
            "node_02 = example_py.node_02:main",
            "node_04 = example_py.node_04:main"
        ],
    },
~~~
其中`example_py`是功能包的名字

## 1.6 运行节点
无论cpp还是py都要先编译一下
~~~shell
cd 工作空间/
colcon build
~~~
source环境，也可以直接修改bashrc文件
~~~shell
source install/setup.bash
~~~
运行节点
~~~shell
ros2 run <package_name> <executable_name>
~~~
### 1.6.1 launch启动
首先在功能包中新建一个文件夹`launch/`，然后再这个文件夹下新家文件`文件名.launch.py`

------
我们需要导入两个库，一个叫做LaunchDescription，用于对launch文件内容进行描述，一个是Node，用于声明节点所在的位置。
> 注意这里要定一个名字叫做`generate_launch_description`的函数，ROS2会对该函数名字做识别。

~~~python
# 导入库
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """launch内容描述函数，由ros2 launch 扫描调用"""
    action_robot_01 = Node(
        package="example_action_rclcpp",# 功能包
        executable="action_robot_01",# 可执行文件（节点）
        arguments=['-d', 'ndgh', ...],# ros2 run 指令的参数
        namespace = 'my_robot',		# 再节点名称前加一个前缀，可以将不得不同名的节点放在不同的命名空间防止重名
        name = 'robot1',			# 这里可以定义节点的名字，使得同一个源代码可以生成多个不同名节点
        parameters=[{'rcl_log_level': 40}],# 提供参数
        remappings=[
            ('input/pose','turtle/pose'),# 将节点中的话题进行重命名
            ('output/cmd','turtle/cmd'),]
        )
    action_control_01 = Node(
        package="example_action_rclcpp",# 并不是一定需要提供所有的元素，只有这两个是必须的
        executable="action_control_01"
    )
    # 创建LaunchDescription对象launch_description,用于描述launch文件
    launch_description = LaunchDescription(
        [action_robot_01, action_control_01])
    # 返回让ROS2根据launch描述执行节点
    return launch_description
~~~

-------
launch文件的彼此嵌套
~~~python
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import Node

def generate_launch_description():
    # 包含其他的launch文件
    parameter_yaml=IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        	get_package_share_directory('包名'),'文件夹名，一般为launch'),  
        	'/文件名.launch.py'])
    	)
    
    # 将包含的launch文件加一个指定的命名空间，防止某些名称重复
    parameter_yaml_with_namespace = GroupAction(
        actions=[
            PushRosNamespace('新的命名空间'),
        	parameter_yaml]
    	)

    launch_description = LaunchDescription(
        [parameter_yaml_with_namespace, 
        ....,
        ....])

    return launch_description
~~~

-------
**将launch文件拷贝到安装目录**

如果功能包是`ament_cmake`类型功能包，修改文件`CMakelist.txt`，添加如下内容
~~~cmake
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})
~~~

如果功能包是`ament_python`类型功能包，修改文件`setup.py`，添加如下内容
~~~python
from setuptools import setup
from glob import glob		#添加
import os					#添加

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),#添加
    ],
)
~~~

# 2. 通信手段
## 2.1 话题 
话题通讯可以有多种组合方式：
+ 1对1
+ 1对多
+ 多对1
+ 多对多
+ 自发自收

### 2.1.1 RCLCPP实现
#### 2.1.1.1 导入消息接口
消息接口是ROS2通信时必须的一部分，通过消息接口ROS2才能完成消息的序列化和反序列化。ROS2为我们定义好了常用的消息接口，并生成了C++和Python的依赖文件，我们可以直接在程序中进行导入。

ament_cmake类型功能包导入消息接口分为三步：

+ 在CMakeLists.txt中导入，具体是先find_packages再ament_target_dependencies。
+ 在packages.xml中导入，具体是添加depend标签并将消息接口写入。
+ 在代码中导入，C++中是#include"消息功能包/xxx/xxx.hpp"。

我们依次做完这三步后文件内容如下：

CMakeLists.txt
~~~cmake
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(topic_publisher_01 src/topic_publisher_01.cpp)
add_executable(topic_subscribe_01 src/topic_subscribe_01.cpp)

ament_target_dependencies(topic_publisher_01 rclcpp std_msgs)
ament_target_dependencies(topic_subscribe_01 rclcpp std_msgs)

install(TARGETS
topic_publisher_01
topic_subscribe_01
  DESTINATION lib/${PROJECT_NAME}
)
~~~
packages.xml
~~~xml

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
~~~
#### 2.1.1.2 发布者
需要提供消息接口、话题名称和服务质量Qos。
+ 消息接口上面我们已经导入了，是std_msgs/msg/string.h。
+ 话题名称（topic_name），我们就用control_command。
+ Qos，Qos支持直接指定一个数字，这个数字对应的是KeepLast队列长度。一般设置成10，即如果一次性有100条消息，默认保留最新的10个，其余的都扔掉。
+ 需要声明一个定时器，去按照指定周期发布数据

~~~C++
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TopicPublisher01 : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    TopicPublisher01(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "大家好，我是%s.", name.c_str());
        // 创建发布者
        command_publisher_ = this->create_publisher<std_msgs::msg::String>("command", 10);
        // 创建定时器，500ms为周期，定时发布
        timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&TopicPublisher01::timer_callback, this));
    }

private:
    void timer_callback()
    {
        // 创建消息
        std_msgs::msg::String message;
        message.data = "forward";
        // 日志打印
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        // 发布消息
        command_publisher_->publish(message);
    }
    // 声名定时器指针
    rclcpp::TimerBase::SharedPtr timer_;
    // 声明话题发布者
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<TopicPublisher01>("topic_publisher_01");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
~~~
#### 2.1.1.3 订阅者
~~~C++
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class TopicSubscribe01 : public rclcpp::Node
{
public:
    // 构造函数,有一个参数为节点名称
    TopicSubscribe01(std::string name) : Node(name)
    {
        RCLCPP_INFO(this->get_logger(), "大家好，我是%s.", name.c_str());
        // 创建一个订阅者订阅话题
        command_subscribe_ = this->create_subscription<std_msgs::msg::String>("command", 10, std::bind(&TopicSubscribe01::command_callback, this, std::placeholders::_1));
    }

private:
     // 声明一个订阅者
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_subscribe_;
    // 收到话题数据的回调函数
    void command_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        double speed = 0.0f;
        if(msg->data == "forward")
        {
            speed = 0.2f;
        }
        RCLCPP_INFO(this->get_logger(), "收到[%s]指令，发送速度 %f", msg->data.c_str(),speed);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    /*创建对应节点的共享指针对象*/
    auto node = std::make_shared<TopicSubscribe01>("topic_subscribe_01");
    /* 运行节点，并检测退出信号*/
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
~~~
### 2.1.2 RCLPY实现
#### 2.1.2.1 发布者
~~~python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NodePublisher02(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)
        self.command_publisher_ = self.create_publisher(String,"command", 10) 
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        # 定时器回调函数
        msg = String()
        msg.data = 'backup'
        self.command_publisher_.publish(msg) 
        self.get_logger().info(f'发布了指令：{msg.data}')    #打印一下发布的数据

def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = NodePublisher02("topic_publisher_02")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    node.destroy_node()
    rclpy.shutdown() # 关闭rclpy
~~~
#### 2.1.2.2 订阅者
~~~python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NodeSubscribe02(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)
         # 创建订阅者
        self.command_subscribe_ = self.create_subscription(String,"command",self.command_callback,10)

    def command_callback(self,msg):
        speed = 0.0
        if msg.data=="backup":
            speed = -0.2
        self.get_logger().info(f'收到[{msg.data}]命令，发送速度{speed}')

def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = NodeSubscribe02("topic_subscribe_02")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    node.destroy_node()
    rclpy.shutdown() # 关闭rclpy
~~~
#### 2.1.2.3 setup.py
修改文件中如下部分
~~~python
entry_points={
        'console_scripts': [
            "topic_publisher_02 = conmunication_py.topic_publisher_02:main",
            "topic_subscribe_02 = conmunication_py.topic_subscribe_02:main"
        ],
~~~


## 2.2 服务
客户端发送请求给服务端，服务端可以根据客户端的请求做一些处理，然后返回结果给客户端。所以服务-客户端模型，也可以成为请求-响应模型。

要注意服务的一下特点：
+ 同一个服务只能由一个节点提供
+ 同一个服务可以被多个不同客户端调用
+ 相比于话题，服务是双向通信的

### 2.2.1 RCLCPP实现
#### 2.2.1.1 导入服务接口
+ 在CMakeLists.txt中导入，具体是先find_packages再ament_target_dependencies。
+ 在packages.xml中导入，具体是添加depend标签并将消息接口写入。
+ 在代码中导入，C++中是#include"消息功能包/xxx/xxx.hpp"。

CMakeLists.txt
~~~cmake
find_package(example_interfaces REQUIRED)#这个是实例的服务，实际项目中用不到

add_executable(service_server_01  src/service_server.cpp)
ament_target_dependencies(service_server_01  rclcpp example_interfaces)

add_executable(service_client_01  src/service_client.cpp)
ament_target_dependencies(service_client_01  rclcpp example_interfaces)

install(TARGETS
  service_server_01	
  service_client_01
  DESTINATION lib/${PROJECT_NAME}
)
~~~

packages.xml
~~~xml
<depend>example_interfaces</depend> #这个是实例的服务，实际项目中用不到
~~~

代码
~~~C++
#include "example_interfaces/srv/add_two_ints.hpp"	//这个是实例的服务，实际项目中用不到
~~~
#### 2.2.1.2 服务端
~~~C++
#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

class ServiceServer01 : public rclcpp::Node {
public:
  ServiceServer01(std::string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
    // 创建服务
    add_ints_server_ =
      this->create_service<example_interfaces::srv::AddTwoInts>(
        "add_two_ints_srv",
        std::bind(&ServiceServer01::handle_add_two_ints, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

private:
  // 声明一个服务
  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr
    add_ints_server_;

  // 收到请求的处理函数
  void handle_add_two_ints(
    const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
    std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response) {
    RCLCPP_INFO(this->get_logger(), "收到a: %ld b: %ld", request->a,
                request->b);
    response->sum = request->a + request->b;
  };
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  /*创建对应节点的共享指针对象*/
  auto node = std::make_shared<ServiceServer01>("service_server_01");
  /* 运行节点，并检测退出信号*/
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
~~~
#### 2.2.1.3 客户端
~~~C++
#include "example_interfaces/srv/add_two_ints.hpp"

class ServiceClient01 : public rclcpp::Node {
public:
  // 构造函数,有一个参数为节点名称
  ServiceClient01(std::string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
    // 创建客户端
    client_ = this->create_client<example_interfaces::srv::AddTwoInts>("add_two_ints_srv");
  }

  void send_request(int a, int b) {
    RCLCPP_INFO(this->get_logger(), "计算%d+%d", a, b);

    // 1.等待服务端上线
    while (!client_->wait_for_service(std::chrono::seconds(1))) {
      //等待时检测rclcpp的状态
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(this->get_logger(), "等待服务的过程中被打断...");
        return;
      }
      RCLCPP_INFO(this->get_logger(), "等待服务端上线中");
    }

    // 2.构造请求的
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts_Request>();
    request->a = a;
    request->b = b;

    // 3.发送异步请求，然后等待返回，返回时调用回调函数
    client_->async_send_request(
      request, std::bind(&ServiceClient01::result_callback_, this,
                         std::placeholders::_1));
  };

private:
  // 声明客户端
  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;

  void result_callback_(
    rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture result_future) {
    auto response = result_future.get();
    RCLCPP_INFO(this->get_logger(), "计算结果：%ld", response->sum);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  /*创建对应节点的共享指针对象*/
  auto node = std::make_shared<ServiceClient01>("service_client_01");
  /* 运行节点，并检测退出信号*/
  //增加这一行，node->send_request(5, 6);，计算5+6结果
  node->send_request(5, 6);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
~~~
### 2.2.2 RCLPY实现
#### 2.2.2.1 服务端
~~~python
import rclpy
from rclpy.node import Node
# 导入接口
from example_interfaces.srv import AddTwoInts

class ServiceServer02(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("节点已启动：%s!" % name)
        self.add_ints_server_ = self.create_service(AddTwoInts,"add_two_ints_srv", self.handle_add_two_ints) 

    def handle_add_two_ints(self,request, response):
        self.get_logger().info(f"收到请求，计算{request.a} + {request.b}")
        response.sum = request.a + request.b
        return response

def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = ServiceServer02("service_server_02")  # 新建一个节点
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    node.destroy_node()
    rclpy.shutdown() # 关闭rclpy
~~~

#### 2.2.2.2 客户端
~~~python
import rclpy
from rclpy.node import Node
# 导入接口
from example_interfaces.srv import AddTwoInts

class ServiceClient02(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info('节点已启动！')
        self.client_ = self.create_client(AddTwoInts,"add_two_ints_srv") 
    
    def send_request(self, a, b):
        while rclpy.ok() and self.client_.wait_for_service(1)==False:
            self.get_logger().info(f"等待服务端上线....")
            
        request = AddTwoInts.Request()
        request.a = a
        request.b = b
        self.client_.call_async(request).add_done_callback(self.result_callback_)

    def result_callback_(self, result_future):
        response = result_future.result()
        self.get_logger().info(f"收到返回结果：{response.sum}")

def main(args=None):
    rclpy.init(args=args) # 初始化rclpy
    node = ServiceClient02("service_client_02")  # 新建一个节点
    # 调用函数发送请求
    node.send_request(3,4)
    rclpy.spin(node) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    node.destroy_node()
    rclpy.shutdown() # 关闭rclpy
~~~
#### 2.2.2.3 setup.py
和之前一样，在setup.py中添加入口位置
~~~python
entry_points={
        'console_scripts': [
            ......,
            ......,
            "service_client_02 = conmunication_py.service_client_02:main",
            "service_server_02 = conmunication_py.service_server_02:main"
        ],
~~~
## 2.3 参数
### 2.3.1 参数组成成分
ROS2参数是由**键值对**组成的，键值对指的是就是名字和数值，比方说

+ 名字：李四写小说周期，值：5s
+ 名字：显示器亮度,值：60%

名字的数据类型小鱼不多说肯定是字符串了，值的数据类型呢？我们这里用到的是5是整形数据，显然只有一个整形是不够用的，ROS2支持的参数值的类型如下：

+ bool 和bool[]，布尔类型用来表示开关，比如我们可以控制雷达控制节点，开始扫描和停止扫描。
+ int64 和int64[]，整形表示一个数字，含义可以自己来定义，这里我们可以用来表示李四节点写小说的周期值
+ float64 和float64[]，浮点型，可以表示小数类型的参数值
+ string 和string[]，字符串，可以用来表示雷达控制节点中真实雷达的ip地址
+ byte[]，字节数组，这个可以用来表示图片，点云数据等信息

### 2.3.2 RCLCPP实现

### 2.3.2 RCLPY实现
和普通写节点区别不大，主要介绍两个`Node`类方法来设置和获取参数
~~~python
self.declare_parameter('rcl_log_level', 0)	#定义并初始化参数
log_level = self.get_parameter("rcl_log_level").value	#获取参数值
~~~

## 2.4 动作
Action的三大组成部分：目标、反馈和结果。
+ 目标：即Action客户端告诉服务端要做什么，服务端针对该目标要有响应。解决了不能确认服务端接收并处理目标问题
+ 反馈：即Action服务端告诉客户端此时做的进度如何（类似与工作汇报）。解决执行过程中没有反馈问题
+ 结果：即Action服务端最终告诉客户端其执行结果，结果最后返回，用于表示任务最终执行情况。

> 参数是由服务构建出来了，而Action是由话题和服务共同构建出来的（一个Action = 三个服务+两个话题） 三个服务分别是：1.目标传递服务 2.结果传递服务 3.取消执行服务 两个话题：1.反馈话题（服务发布，客户端订阅） 2.状态话题（服务端发布，客户端订阅）

**因为ros2没有自带的动作接口，所以只能自己创建**

### 2.4.1 action代码结构解析

### 2.4.2 动作RCLCPP实现

### 2.4.3 动作RCLPY实现
首先需要导入包
~~~python
import time	# 需要定时
# 导入rclpy相关库
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
# 导入接口，自定义的那个接口
from robot_control_interfaces.action import MoveRobot
~~~
然后编写action服务端的内容，主体是虽然找不到三个回调函数，但是三部分的功能在函数`execute_callback`里均有体现
~~~python
class ActionRobot02(Node):
    def __init__(self,name):
            super().__init__(name)
            self.get_logger().info(f"节点已启动：{name}!")
            self.action_server_=ActionServer(
                self,
                MoveRobot,	#自定义action接口
                'move_robot',	#action的名称
                self.execute_callback,)
   
# execute_callback函数是action所有内容，需要考虑用time.sleep()来保证不会阻塞其他进程
def execute_callback(self, goal_handle):
    feedback_msg = MoveRobot.Feedback()	#定义action中反馈那部分数据对象
    goal_handle.publish_feedback(feedback_msg)	#反馈action中反馈那部分二点数据对象
    
    if goal_handle.is_cancel_requested:	#如果接收到停止action的要求
        result = MoveRobot.Result()	#定义一个action中结果那部分数据对象
        result.finish = False	#这部分泛指对result的操作，具体和自己定义的action接口有关
        return result	#把结果返回
           
    goal_handle.succeed()	#执行此句表示action执行成功
    result = MoveRobot.Result()	#定义一个action中结果那部分数据对象
    result.finish = True	#这部分泛指对result的操作，具体和自己定义的action接口有关
	return result	#把结果返回
~~~

然后编写action客户端的内容
~~~python
class ActionControl02(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"节点已启动：{name}!")
        self.action_client_ = ActionClient(self, MoveRobot, 'move_robot')
        
    def send_goal(self):
        self.action_client_.wait_for_server()	#等待服务端启动
        
        goal_msg = MoveRobot.Goal()	# 定义action中目标那部分的数据对象
        self.send_goal_ = self.action_client_.send_goal_async(	# 发送目标请求
        	goal_msg,									# 传入的Goal数据对象
        	feedback_callback=self.feedback_callback)	# 处理周期反馈的回调函数
        
        self.send_goal_.add_done_callback(self.goal_response_callback)#对目标的反馈回调函数
        
    def goal_response_callback(self, future):#对目标的反馈回调函数
        goal_handle = future.result()
        if not goal_handle.accepted:	#如果被拒绝执行目标
            self.get_logger().info('Goal rejected :(')
            return
        
        self._get_result_future = goal_handle.get_result_async()# 异步获取执行结果
        self._get_result_future.add_done_callback(self.get_result_callback)# 执行有结果后的回调函数
        
    def get_result_callback(self, future):#对执行反馈结果的回调函数
        result = future.result().result	#获取反馈结果
    
	def feedback_callback(self, feedback_msg):#处理周期反馈的回调函数
        feedback = feedback_msg.feedback	#获取反馈结果
~~~



## 2.5 ROS2接口介绍
interface，即接口。**接口其实是一种规范**

除了参数之外，话题、服务和动作(Action)都支持自定义接口，所定义的接口也被分为话题接口、服务接口、动作接口三种。

通过ROS2的IDL模块 产生了头文件，有了头文件，我们就可以在程序里导入并使用这个消息模块。

```mermaid
graph LR
	A[msg,srv,action]-->
	B[ROS2-IDL转换器]-->
	C[Python的py,C++的.h头文件]
```
### 2.5.1 ros2自带接口
使用指令`ros2 interface package sensor_msgs`查看自带的接口
### 2.5.2 接口文件内容
话题接口格式：`xxx.msg`
~~~
int64 num_static=100  可以赋值
int64 num
~~~
服务接口格式：`xxx.srv`
~~~
int64 a	#服务的请求
int64 b
---
int64 sum	#服务的反馈
~~~
动作接口格式：`xxx.action`
~~~
int32 order		#定义动作目标
---
int32[] sequence	#定义动作的结果
---
int32[] partial_sequence	#定义动作反馈
~~~
### 2.5.3 接口数据类型
基础类型有（==同时后面加上[]可形成数组==，**注意[]加载数据类型后，和C语言不一样**）
~~~
bool
byte
char
float32,float64
int8,uint8
int16,uint16
int32,uint32
int64,uint64
string
~~~

包装类型，即在已有的接口类型上进行包含，比如
~~~
uint32 id
string image_name
sensor_msgs/Image
~~~
### 2.5.4 自定义接口实践
-----
（1）编写对应的接口文件
话题接口放到msg文件夹下，以.msg结尾。服务接口放到srv文件夹下，以srv结尾。动作接口放到action文件夹下，以action结尾。

-----
（2）创建接口功能包编接口
~~~shell
ros2 pkg create example_ros2_interfaces --build-type ament_cmake --dependencies rosidl_default_generators geometry_msgs
~~~
**注意功能包类型必须为：ament_cmake**，因此如果之后要用python写的话需要新建包，并在py文件中imprt包`from 包名(指的是ros包).msg import 接口名`

依赖rosidl_default_generators必须添加，geometry_msgs视内容情况添加（比如如果使用geometry_msgs/Pose pose就要添加）。

接着修改`CMakeLists.txt`
~~~cmake
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED) #这部分视情况添加
# 添加下面的内容
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/对应文件名.msg"
  "action/对应文件名.msg"
  "srv/对应文件名.srv"
  DEPENDENCIES geometry_msgs
)
~~~
==千万注意，`rosidl_generate_interfaces`必须放在`ament_package()`前==

接着修改`package.xml`

~~~xml
<buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rosidl_default_generators</depend>
  <depend>geometry_msgs</depend>#视情况而定，取决于是否使用geometry_msgs提供的现成接口
  
  <member_of_group>rosidl_interface_packages</member_of_group> #添加这一行

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
~~~
保存即可编译
~~~shell
colcon build --packages-select example_ros2_interfaces
~~~
编译完成后在`工作空间/install/包名/include`下你应该可以看到C++的头文件。在`工作空间/install/包名/local/lib/python3.10/dist-packages`下应该可以看到Python版本的头文件。

# 3. 机器人学
https://fishros.com/d2lros2/#/humble/chapt6/basic/1.%E7%9F%A9%E9%98%B5%E4%B8%8E%E7%9F%A9%E9%98%B5%E8%BF%90%E7%AE%97