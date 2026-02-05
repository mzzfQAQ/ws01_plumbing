# 需求：编写动作服务端，需要解析客户端提交的数据，遍历该数字并累加求和，最终结果需要响应回客户端，
    #   且请求响应过程中需要生成连续反馈
# 流程：
#     1.导包
#     2.初始化ROS2客户端
#     3.自定义节点类
        # 3-1.创建动作服务对象
        # 3-2.处理提交的目标值（回调函数）---默认实现
        # 3-3.生成连续反馈与最终响应（回调函数）
        # 3-4.处理取消请求（回调函数）---默认实现
#     4.调用spin函数，传入自定义类对象
#     5.释放资源

# 1.导包
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from base_interfaces_demo.action import Progress
from rclpy.action import CancelResponse
import time

# 3.自定义节点类
class ProgressActionServer(Node):
    def __init__(self):
        super().__init__("progress_action_server_node_py")
        self.get_logger().info("progress_action_server_node_py 节点已启动（python）!")
        # 3-1.创建动作服务对象
        self.server = ActionServer(
            self,
            Progress,
            "get_sum",
            self.execute_callback,
            cancel_callback=self.cancel_callback
            )
        # 3-3.生成连续反馈与最终响应（回调函数）
    def execute_callback(self, goal_handle):
        num = goal_handle.request.num
        sum = 0
        # --- 修正：先实例化结果对象，确保取消逻辑能访问到它 ---
        result = Progress.Result() 
        
        self.get_logger().info("开始执行任务...")

        for i in range(1, num + 1):
            # 核心：取消请求处理逻辑
            if goal_handle.is_cancel_requested:
                goal_handle.canceled() 
                result.sum = sum       
                self.get_logger().info("任务被取消了！")
                return result          
            
            sum += i
            feedback = Progress.Feedback()
            feedback.progress = i / num
            goal_handle.publish_feedback(feedback)
            self.get_logger().info("连续反馈：%.2f" % feedback.progress)
            time.sleep(1.0)

        # 2. 响应最终结果
        goal_handle.succeed()
        result.sum = sum # 正常完成时更新最终结果
        self.get_logger().info("最终计算结果：%d" % result.sum)

        return result
    
    def cancel_callback(self, cancel_request):
    # """当客户端请求取消时，服务端决定是否允许取消"""
        self.get_logger().info('接收到取消请求，正在处理...')
        
        return CancelResponse.ACCEPT

    
        

def main():
    # 2.初始化ROS2客户端
    rclpy.init()
    # 4.调用spin函数，传入自定义类对象
    rclpy.spin(ProgressActionServer())
    # 5.释放资源
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    