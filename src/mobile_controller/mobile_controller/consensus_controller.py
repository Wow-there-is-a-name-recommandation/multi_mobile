import rclpy
import sys
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from custom_interfaces.msg import RobotState
import ast
import math
import numpy as np

class MobileController(Node):

    def __init__(self, robot_id, target):

        super().__init__(f'consensus_controller_{robot_id}')
        self.robot_id = robot_id
        self.target = ast.literal_eval(target)
        self.compensate_x = 0.0
        self.compensate_y = 0.0
        self.compensate_theta = 0.0
        self.neighbor_states = {}       # 이웃 로봇들의 상태저장공간

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
        
        # 다른 로봇들의 타겟 정보 구독 (target_net)
        self.target_subscriber = self.create_subscription(
            RobotState,
            '/target_net',          # 이웃 로봇들의 목표가 발행되는 토픽
            self.target_callback,
            10
        )

        self.cmd_vel_publisher = self.create_publisher(Twist, f'/{robot_id}/cmd_vel', 10)

        self.state = [0.0, 0.0, 0.0]    # 현재 로봇의 상태저장공간 (x, y, theta(rad))

        self.timer_laplacian = self.create_timer(0.5, self.control_loop)  # 100ms마다 제어 입력 계산

    def odom_callback(self, msg):
        # 자신의 위치 정보를 업데이트
        self.state = msg.state
        #self.get_logger().info(f'Updated self position: {self.state}')

    def neighbor_callback(self, msg):
        # 이웃 로봇들의 상태를 robot_id를 기준으로 저장
        neighbor_position = msg.state[:2]
        self_position = self.state[:2]
        distance = math.sqrt((neighbor_position[0] - self_position[0]) ** 2 + (neighbor_position[1] - self_position[1]) ** 2)
        if distance <= 5.0:
            self.neighbor_states[msg.robot_id] = msg.state
        #self.get_logger().info(f'Received neighbor position for {msg.robot_id}: {msg.state}')

    def target_callback(self, msg):
        # 이웃 로봇들의 타겟 정보를 robot_id를 기준으로 저장
        if (msg.robot_id == self.robot_id):
            self.target = msg.state
        self.get_logger().info(f'Received target: {msg.state}')

    def control_loop(self):

        # 제어 입력을 적용하여 로봇의 속도를 업데이트
        # twist는 로봇 기준 속도 입력 메시지 생성
        cmd_vel = Twist()
        # 목표 방향 계산 (타겟 방향)
        target_direction = math.atan2(self.target[1] - self.state[1], self.target[0] - self.state[0])
    
        # 목표 방향과 현재 방향 간의 각도 차이 계산
        angle_to_target = target_direction - self.state[2]
        angle_to_target = math.atan2(math.sin(angle_to_target), math.cos(angle_to_target))  # 각도 정규화

        # 직진 거리 계산
        distance_to_target = math.sqrt((self.target[0] - self.state[0]) ** 2 + (self.target[1] - self.state[1]) ** 2)

        attractive_force = [0.0, 0.0]
        repulsive_force = [0.0, 0.0]

        # 인력 계산 (목표 지점 방향으로)
        attraction_strength = np.exp(-distance_to_target)  # 거리가 가까워질수록 인력이 약해짐
        attractive_force[0] = (self.target[0] - self.state[0]) * attraction_strength
        attractive_force[1] = (self.target[1] - self.state[1]) * attraction_strength

        # 이웃 로봇에 대한 척력(밀림) 계산
        for neighbor_id, neighbor_state in self.neighbor_states.items():
            dx = self.state[0] - neighbor_state[0]
            dy = self.state[1] - neighbor_state[1]
            distance = math.sqrt(dx ** 2 + dy ** 2)

            if distance < 5.0 and distance > 0.1:  # 너무 가깝거나 멀리 있는 로봇은 고려하지 않음
                repulsion_strength = min(5.0, 1.0 / distance ** 2)  # 거리가 가까울수록 큰 척력 적용
                repulsive_force[0] += repulsion_strength * dx / distance
                repulsive_force[1] += repulsion_strength * dy / distance

        # 최종 제어 입력 계산
        total_force = [
            attractive_force[0] + repulsive_force[0],
            attractive_force[1] + repulsive_force[1]
        ]

        # 방향과 속도 설정
        target_direction = math.atan2(total_force[1], total_force[0])
        angle_to_target = target_direction - self.state[2]
        angle_to_target = math.atan2(math.sin(angle_to_target), math.cos(angle_to_target))

        linear_velocity = math.sqrt(total_force[0] ** 2 + total_force[1] ** 2)
        angular_velocity = angle_to_target

        # 목표 지점에 도달했을 때 멈춤
        if distance_to_target <= 0.1:
            cmd_vel.linear.x = 0.0
            cmd_vel.angular.z = 0.0
        else:
            cmd_vel.linear.x = min(0.5, linear_velocity)
            cmd_vel.angular.z = min(2.0, angular_velocity)

        self.cmd_vel_publisher.publish(cmd_vel)
        
        #self.get_logger().info(f'Publishing control x={self.state[0]}, y={self.state[1]}, dis={target_direction}')


def main(args=None):
    rclpy.init(args=args)

    # 런치 파일에서 전달된 인자를 사용하여 로봇 이름을 설정
    robot_id = sys.argv[1]
    target = sys.argv[2]

    # 각각의 로봇 이름에 맞는 컨트롤러 노드 생성
    controller = MobileController(robot_id, target)

    rclpy.spin(controller)

    # Destroy the node explicitly
    controller.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
