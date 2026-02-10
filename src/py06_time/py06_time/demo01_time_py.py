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
import threading
from rclpy.time import Time
from rclpy.duration import Duration

# 3.自定义节点类
class MyNode(Node):
    def __init__(self):
        super().__init__("time_node_py")
        self.get_logger().info("time_node_py节点已启动（python）!")
        # self.demo_rate()
        # self.demo_time()
        self.demo_duration()

    def demo_duration(self):
        # 1.创建 Duration 对象
        du1 = Duration(seconds=10,nanoseconds=800000000)
        # 2.调用函数
        self.get_logger().info("ns = %d" % (du1.nanoseconds))

    def demo_time(self):
        # 1.创建 Time 对象
        t1 = Time(seconds= 5,nanoseconds= 500000000)
        right_now = self.get_clock().now()
        # 2.调用 time 对象的函数
        self.get_logger().info("s = %.2f,ns = %d" % (t1.seconds_nanoseconds()[0],t1.seconds_nanoseconds()[1]))
        self.get_logger().info("s = %.2f,ns = %d" % (right_now.seconds_nanoseconds()[0],right_now.seconds_nanoseconds()[1]))

    def demo_rate(self):
        # 1.创建 Rate 对象
        self.rate = self.create_rate(1.0)
        # 2.调用 sleep 函数 --- 会导致程序阻塞
        # 解决方案1:使用time休眠
        # while rclpy.ok():
        #     self.get_logger().info("++++++++++++++++")
        #     self.rate.sleep()

        # 解决方案2:创建子线程实现运行频率控制
        thread = threading.Thread(target=self.do_some)
        thread.start()

    def do_some(self):
        while rclpy.ok():
            self.get_logger().info("++++++++++++++++")
            self.rate.sleep()

def main():
    # 2.初始化ROS2客户端
    rclpy.init()
    # 4.调用spin函数，传入自定义类对象
    rclpy.spin(MyNode())
    # 5.释放资源
    rclpy.shutdown()

if __name__ == '__main__':
    main()