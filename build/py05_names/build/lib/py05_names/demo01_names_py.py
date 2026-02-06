# 需求：编写一个ROS2节点（Python）
# 流程：
#     1.导包
#     2.初始化ROS2客户端
#     3.自定义节点类
#     4.调用spin函数，传入自定义类对象
#     5.释放资源

# 1.导包
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# 3.自定义节点类
class MyNode(Node):
    def __init__(self):
        super().__init__("zhenkeng_py",namespace="zuoxie_py")
        self.get_logger().info("my_node_py 节点已启动（python）!")
        # 全局话题：和命名空间，节点名称无关系
        # self.pub = self.create_publisher(String,"/shi",10)
        # 相对话题：和命名空间有关系，节点名称无关系
        # self.pub = self.create_publisher(String,"kaihui",10)
        # 私有话题：和命名空间，节点名称有关系  
        self.pub = self.create_publisher(String,"~/vip",10)

def main():
    # 2.初始化ROS2客户端
    rclpy.init()
    # 4.调用spin函数，传入自定义类对象
    rclpy.spin(MyNode())
    # 5.释放资源
    rclpy.shutdown()

if __name__ == '__main__':
    main()