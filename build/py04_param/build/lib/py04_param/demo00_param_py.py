# 需求：编写一个ROS2节点（Python）
# 流程：
#     1.导包
#     2.初始化ROS2客户端
#     3.自定义节点类
        # 3-1.创建参数对象
        # 3-2.解析参数
#     4.调用spin函数，传入自定义类对象
#     5.释放资源

# 1.导包
import rclpy
from rclpy.node import Node

# 3.自定义节点类
class MyParam(Node):
    def __init__(self):
        super().__init__("my_param_node_py")
        self.get_logger().info("my_param_node_py 节点已启动（python）!")
        # 3-1.创建参数对象
        p1 = rclpy.Parameter("car_name",value="Tiger")
        p2 = rclpy.Parameter("width",value=1.5)
        p3 = rclpy.Parameter("wheels",value=2)
        # 3-2.解析参数
        self.get_logger().info("car_name = %s" % p1.value)
        self.get_logger().info("width = %.2f" % p2.value)
        self.get_logger().info("wheels = %d" % p3.value)

        self.get_logger().info("key = %s" %p1.name)

def main():
    # 2.初始化ROS2客户端
    rclpy.init()
    # 4.调用spin函数，传入自定义类对象
    rclpy.spin(MyParam())
    # 5.释放资源
    rclpy.shutdown()

if __name__ == '__main__':
    main()

