/*
 需求：订阅乌龟1的位姿信息，解析出线速度和角速度，生成并发布控制乌龟2运动的速度指令
 明确：
    订阅话题：/turtle1/pose
    订阅消息：/turtle1/msg/Pose

    发布话题：/t2/turtle1/cmd_vel
    发布消息：gemotry_msgs/msg/Twist
 流程：
     1.包含头文件
     2.初始化ROS2客户端
     3.自定义节点类
        3-1.创建发布方
        3-2.创建订阅方（订阅乌龟1位姿，解析速度）
        3-3.订阅方的回调函数要处理速度，生成并发布控制乌龟2的速度指令
     4.调用spin函数，传入自定义类对象指针
     5.释放资源

bug描述：
    乌龟1后退时，乌龟2仍然继续前进

bug原因：
    1.和乌龟位姿发布有关，当乌龟实际速度为负数时，位姿中的速度仍是正数
    2.发布的乌龟2的速度，与位姿中的线速度一致
    
bug修复：
    修改源码，将位姿中的线速度计算修改为直接等于 x 方向速度
*/

// 1.包含头文件
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"

// 3.自定义节点类
class Exer01PubSub : public rclcpp::Node
{
public:
    Exer01PubSub() : Node("exer01_pub_sub_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "exer01_pub_sub_node_cpp 节点已启动（C++）!");
        // 3-1.创建发布方
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/t2/turtle1/cmd_vel",10);
        // 3-2.创建订阅方（订阅乌龟1位姿，解析速度）
        sub_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose",10,std::bind(&Exer01PubSub::pose_cb,this,std::placeholders::_1));
        
    }

private:
    void pose_cb(const turtlesim::msg::Pose &pose){
        // 3-3.订阅方的回调函数要处理速度，生成并发布控制乌龟2的速度指令
        // 创建新的速度指令
        geometry_msgs::msg::Twist twist;
        twist.linear.x = pose.linear_velocity;
        twist.angular.z = -pose.angular_velocity;

        // 发布
        pub_->publish(twist);

    }
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr sub_;
};

int main(int argc, char **argv)
{
    // 2.初始化ROS2客户端
    rclcpp::init(argc, argv);
    // 4.调用spin函数，传入自定义类对象指针
    auto node = std::make_shared<Exer01PubSub>();
    rclcpp::spin(node);
    // 5.释放资源
    rclcpp::shutdown();
    return 0;
}