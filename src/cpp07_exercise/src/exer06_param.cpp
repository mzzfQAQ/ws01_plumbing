/*
 需求：修改 turtlesim_node 的背景色
 流程：
     1.包含头文件
     2.初始化ROS2客户端
     3.自定义节点类
        3-1.创建参数客户端
        3-2.连接参数服务端
        3-3.更新参数
     4.创建节点对象指针，并调用其函数
     5.释放资源
*/

// 1.包含头文件
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

// 3.自定义节点类
class Exer06Param : public rclcpp::Node
{
public:
    Exer06Param() : Node("exer06_param_node_cpp")
    {
        RCLCPP_INFO(this->get_logger(), "exer06_param_node_cpp 节点已启动（C++）!");
        // 3-1.创建参数客户端
        client_ = std::make_shared<rclcpp::SyncParametersClient>(this,"/turtlesim");
    }
    // 3-2.连接参数服务端
    bool connect_server(){
        while (!client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"强制退出客户端");
                return false;
            }
            

            RCLCPP_INFO(this->get_logger(),"服务连接中....");
        }
        
        return true;
    }
    // 3-3.更新参数
    void update_param(){
        // 1.获取参数
        int red = client_->get_parameter<int>("background_r");
        // 2.编写循环，修改参数（通过休眠控制修改频率）
        rclcpp::Rate rate(30.0);
        int count = red;
        while (rclcpp::ok())
        {
            // red += 5;
            count <= 255 ? red += 5:red -= 5;
            count += 5;
            if(count >= 511) count = 0;
            

            // 修改服务端参数
            client_->set_parameters({rclcpp::Parameter("background_r",red)});

            rate.sleep();
        }
        
    }
    
private:    
    rclcpp::SyncParametersClient::SharedPtr client_;
};

int main(int argc, char **argv)
{
    // 2.初始化ROS2客户端
    rclcpp::init(argc, argv);
    // 4.调用spin函数，传入自定义类对象指针
    auto node = std::make_shared<Exer06Param>();
    if (!node->connect_server())
    {
        return 1;
    }
    
    // 调用其函数
    node->update_param();
    // rclcpp::spin(node);
    // 5.释放资源
    rclcpp::shutdown();
    return 0;
}