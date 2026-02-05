/*
 需求：编写一个ROS2节点（C++）
 流程：
     1.包含头文件
     2.初始化ROS2客户端
     3.自定义节点类
     4.调用spin函数，传入自定义类对象指针
     5.释放资源
*/

// 1.包含头文件
#include "rclcpp/rclcpp.hpp"

// 3.自定义节点类
class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("my_node_cpp","t1_ns")
    {
        RCLCPP_INFO(this->get_logger(), "my_node_cpp 节点已启动（C++）!");
    }
};

int main(int argc, char **argv)
{
    // 2.初始化ROS2客户端
    rclcpp::init(argc, argv);
    // 4.调用spin函数，传入自定义类对象指针
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    // 5.释放资源
    rclcpp::shutdown();
    return 0;
}

// p151