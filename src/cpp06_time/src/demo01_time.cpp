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

using namespace std::chrono_literals;

// 3.自定义节点类
class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("time_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "time_node_cpp 节点已启动（C++）!");
        demo_rate();
    }

private:
    //演示 Rate 的使用
    void demo_rate()
    {
      // 1.创建 Rate 对象
      rclcpp::Rate rate1(500ms);//设置休眠时间
      rclcpp::Rate rate2(1.0);//设置执行频率
      // 2.调用 Rate 的 sleep 函数
      while (rclcpp::ok())
      {
        RCLCPP_INFO(this->get_logger(),"------------");
        // rate1.sleep();
        rate2.sleep();
      }
      
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