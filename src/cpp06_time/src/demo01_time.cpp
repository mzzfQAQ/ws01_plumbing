/*
    问题：Time与Duration 的区别：
    答：
      1.二者只是API使用类似；
      2.二者有本质区别：
        rclcpp::Time t2(2,500000000L);---指的是一个具体时刻
        rclcpp::Duration du2(2,500000000);---指的是一个时间段，持续2.5s
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
        // demo_rate();
        // demo_time();
        demo_duration();
    }

private:
    // 演示 Duration 的使用
    void demo_duration()
    {
      //1.创建 Duration 对象
      rclcpp::Duration du1(1s);
      rclcpp::Duration du2(2,500000000);
      //2.调用函数
      RCLCPP_INFO(this->get_logger(),"s = %.2f, ns = %ld",du1.seconds(),du1.nanoseconds());
      RCLCPP_INFO(this->get_logger(),"s = %.2f, ns = %ld",du2.seconds(),du2.nanoseconds());
    }

    // 演示 Time 的使用
    void demo_time()
    {
      // 1.创建 Time 对象
      rclcpp::Time t1(500000000L);
      rclcpp::Time t2(2,500000000L);
      // rclcpp::Time right_now = this->get_clock()->now();
      rclcpp::Time right_now = this->now();

      // 2.调用 Time 对象的函数
      RCLCPP_INFO(this->get_logger(),"s = %.2f, ns = %ld",t1.seconds(),t1.nanoseconds());
      RCLCPP_INFO(this->get_logger(),"s = %.2f, ns = %ld",t2.seconds(),t2.nanoseconds());
      RCLCPP_INFO(this->get_logger(),"s = %.2f, ns = %ld",right_now.seconds(),right_now.nanoseconds());
    }

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