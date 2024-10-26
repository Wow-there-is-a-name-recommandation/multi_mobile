import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

class InitialCmdVelPublisher(Node):

    def __init__(self):
        super().__init__('initial_cmd_vel_publisher')
        self.publisher_1 = self.create_publisher(Twist, '/my_bot1/cmd_vel', 10)
        self.publisher_2 = self.create_publisher(Twist, '/my_bot2/cmd_vel', 10)
        self.publisher_3 = self.create_publisher(Twist, '/my_bot3/cmd_vel', 10)
        
        # 초기 메시지 발행 후 종료할 타이머 (0.5초 후)
        self.timer = self.create_timer(0.5, self.shutdown_node)
        
        # 초기화 메시지 발행
        self.publish_initial_cmd()

    def publish_initial_cmd(self):
        msg = Twist()
        msg.linear.x = 0.0  # 초기 선형 속도
        msg.angular.z = 0.0  # 초기 각속도

        # 각 로봇에 초기 명령 발행
        self.publisher_1.publish(msg)
        self.publisher_2.publish(msg)
        self.publisher_3.publish(msg)
        
        self.get_logger().info('Initial cmd_vel published, node shutting down')

    def shutdown_node(self):
        # 노드를 안전하게 종료
        self.get_logger().info('Shutting down InitialCmdVelPublisher node...')
        rclpy.shutdown() 

def main(args=None):
    rclpy.init(args=args)

    node = InitialCmdVelPublisher()

    rclpy.spin(node)

if __name__ == '__main__':
    main()
