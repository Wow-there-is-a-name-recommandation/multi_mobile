import rclpy
import sys
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from custom_interfaces.msg import RobotState
import ast
import math
import numpy as np

class LaplacianAlgorithm(Node):

    def __init__(self, robot_id, target):

        super().__init__(f'laplacian_algorithm_{robot_id}')
        self.robot_id = robot_id
        self.target = ast.literal_eval(target)
        self.compensate_x = 0.0
        self.compensate_y = 0.0
        self.compensate_theta = 0.0

        # 자신의 위치 정보 구독 (my_odom)
        self.odom_subscriber = self.create_subscription(
            #topic에 msg가 발행될 때마다 자동 반복
            RobotState,             # 토픽에서 받아올 이웃 노드의 자료형
            f'/{robot_id}/my_odom', # 자신의 오도메트리 정보
            self.odom_callback,
            10
        )

        # 다른 로봇들의 위치 정보 구독 (odom_net)
        self.neighbor_subscriber = self.create_subscription(
            RobotState,
            '/odom_net',            # 이웃 로봇들의 상태가 발행되는 토픽 이름
            self.neighbor_callback,
            10
        )

        # 자신의 타겟을 퍼블리싱 (target_net)
        self.target_publisher = self.create_publisher(RobotState, '/target_net', 10)
        
        # 다른 로봇들의 타겟 정보 구독 (target_net)
        self.target_subscriber = self.create_subscription(
            RobotState,
            '/target_net',          # 이웃 로봇들의 목표가 발행되는 토픽
            self.target_callback,
            10
        )

        self.cmd_vel_publisher = self.create_publisher(Twist, f'/{robot_id}/cmd_vel', 10)

        self.state = [0.0, 0.0, 0.0]    # 현재 로봇의 상태저장공간 (x, y, theta(rad))

        self.neighbor_states = {}       # 이웃 로봇들의 상태저장공간
        self.neighbor_targets = {}  # 이웃 로봇들의 목표값 저장

        self.publish_target()  # 타겟 퍼블리싱 함수 호출

        self.timer_laplacian = self.create_timer(0.5, self.consensus_loop)  # 100ms마다 제어 입력 계산
        self.timer_target = self.create_timer(0.1, self.publish_target)  # 100ms마다 통신

    def odom_callback(self, msg):
        # 자신의 위치 정보를 업데이트
        self.state = msg.state
        #self.get_logger().info(f'Updated self position: {self.state}')

    def neighbor_callback(self, msg):
        # 이웃 로봇들의 상태를 robot_id를 기준으로 저장
        self.neighbor_states[msg.robot_id] = msg.state
        #self.get_logger().info(f'Received neighbor position for {msg.robot_id}: {msg.state}')

    def target_callback(self, msg):
        # 이웃 로봇들의 타겟 정보를 robot_id를 기준으로 저장
        self.neighbor_targets[msg.robot_id] = msg.state
        #self.get_logger().info(f'Received neighbor target for {msg.robot_id}: {msg.state}')

    def publish_target(self):
        # 자신의 목표 위치를 퍼블리싱
        target_msg = RobotState()
        target_msg.robot_id = self.robot_id
        target_msg.state = self.target
        self.target_publisher.publish(target_msg)
        #self.get_logger().info(f'Published target: {self.target}')

    def consensus_loop(self):
        if len(self.neighbor_targets) == 0:
            return  # 이웃 로봇들의 상태 정보가 없을 때는 제어하지 않음

        # 상태 차이를 계산하고 제어 입력을 생성
        # 상태 차이를 저장(dot x)
        target_input_x = 0.0
        target_input_y = 0.0
        target_input_theta = 0.0


        for robot_id, neighbor_target in self.neighbor_targets.items():
            neighbor_x, neighbor_y, neighbor_theta = neighbor_target
            target_input_x += (neighbor_x - self.target[0])
            target_input_y += (neighbor_y - self.target[1])
            target_input_theta += (neighbor_theta - self.target[2])

        #self.get_logger().info(f'Target Change: {target_input_x}, {target_input_y}, {target_input_theta}')

        self.target[0] += 0.3*target_input_x #+ self.compensate_x
        self.target[1] += 0.3*target_input_y #+ self.compensate_y
        self.target[2] += 0.3*target_input_theta #+ self.compensate_theta
        
        self.compensate_x += -4*self.compensate_x-target_input_x
        self.compensate_y = -4*self.compensate_y-target_input_y
        self.compensate_theta = -4*self.compensate_theta-target_input_theta



def main(args=None):
    rclpy.init(args=args)

    # 런치 파일에서 전달된 인자를 사용하여 로봇 이름을 설정
    robot_id = sys.argv[1]
    target = sys.argv[2]

    # 각각의 로봇 이름에 맞는 컨트롤러 노드 생성
    laplacian = LaplacianAlgorithm(robot_id, target)

    rclpy.spin(laplacian)

    # Destroy the node explicitly
    laplacian.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
