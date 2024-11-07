import rclpy
import sys
from rclpy.node import Node
from nav_msgs.msg import Odometry
from custom_interfaces.msg import RobotState
from rclpy.executors import MultiThreadedExecutor

class OdomSubscriber(Node):
    def __init__(self, robot_id):
        super().__init__(f'odom_subscriber_{robot_id}')
        self.robot_id = robot_id

        # 로봇 네임스페이스에 따른 오도메트리 토픽 구독
        odom_topic = f'/{robot_id}/odom'

        # Odometry 메시지를 구독하여 위치와 각도 데이터를 받음
        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,  # Gazebo에서 발행하는 자신의 위치
            self.odom_callback,
            10)
        
        # 위치 발행: /robot_id/my_odom 및 /odom_net
        self.my_odom_publisher = self.create_publisher(RobotState, f'/{robot_id}/my_odom', 10)
        self.odom_net_publisher = self.create_publisher(RobotState, '/odom_net', 10)

    def odom_callback(self, msg):
        # 위치 정보
        position = msg.pose.pose.position
        x = position.x
        y = position.y

        # 각도 정보
        orientation = msg.pose.pose.orientation
        z = orientation.z

        # 로깅을 통해 확인
        #self.get_logger().info(f"Robot {self.robot_id} Position: x={x}, y={y}")
        #self.get_logger().info(f"Robot {self.robot_id} Orientation (z): {z}")

        # 커스텀 메시지 생성 및 발행
        robot_state_msg = RobotState()
        robot_state_msg.robot_id = self.robot_id
        robot_state_msg.state = [x, y, z]  # 상태 정보를 커스텀 메시지에 넣음

        # /robot_id/my_odom 토픽으로 발행
        self.my_odom_publisher.publish(robot_state_msg)

        # /odom_net 토픽으로 발행
        self.odom_net_publisher.publish(robot_state_msg)

def main(args=None):
    rclpy.init(args=args)

    # 런치 파일에서 전달된 인자를 사용하여 로봇 이름을 설정
    robot_id = sys.argv[1]

    # 각각의 로봇 이름에 맞는 오도메트리 구독자 생성
    robot_subscriber = OdomSubscriber(robot_id)

    rclpy.spin(robot_subscriber)

    robot_subscriber.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
