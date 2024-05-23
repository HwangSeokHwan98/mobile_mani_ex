#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Point
from math import sqrt


from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Quaternion
import math
import numpy as np



class InitialPoseAndGoalPublisher:
    def __init__(self):
        rospy.init_node('initial_pose_and_goal_publisher_node', anonymous=True)
        self.pub_initial_pose = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        ##추가
        self.pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz
        self.initial_pose_msg = None  # 초기값은 None으로 초기화



        # /amcl_pose 토픽을 구독하여 초기값 수신
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.amcl_pose_callback)

    def amcl_pose_callback(self, data):
        # /amcl_pose 토픽에서 받은 초기값을 저장
        self.initial_pose_msg = data

    def publish_initial_pose(self):
        # 초기값을 받은 후에만 초기 위치를 발행
        if self.initial_pose_msg is not None:
            self.pub_initial_pose.publish(self.initial_pose_msg)

    def publish_goal(self, goal_x, goal_y, goal_z):

        goal_pose_msg = PoseStamped()
        goal_pose_msg.header.frame_id = "map"
        goal_pose_msg.pose.position.x = goal_x
        goal_pose_msg.pose.position.y = goal_y
        goal_pose_msg.pose.position.z = goal_z
        goal_pose_msg.pose.orientation.x = 0.0
        goal_pose_msg.pose.orientation.y = 0.0
        goal_pose_msg.pose.orientation.z = 0.0
        goal_pose_msg.pose.orientation.w = 1.0

        distance_to_goal = sqrt((self.initial_pose_msg.pose.pose.position.x - goal_x)**2 + (self.initial_pose_msg.pose.pose.position.y - goal_y)**2)

        if distance_to_goal > 0.75:
            direction_vector = Point((goal_x - self.initial_pose_msg.pose.pose.position.x), (goal_y - self.initial_pose_msg.pose.pose.position.y), 0.0)
            direction_length = sqrt(direction_vector.x**2 + direction_vector.y**2)
            scaled_direction_vector = Point((direction_vector.x / direction_length) * 0.75, (direction_vector.y / direction_length) * 0.75, 0.0)
            goal_pose_msg.pose.position.x = goal_x - scaled_direction_vector.x
            goal_pose_msg.pose.position.y = goal_y - scaled_direction_vector.y

            self.pub_goal.publish(goal_pose_msg)
            print("목표로부터 0.5m 전의 좌표로 이동 명령을 보냈습니다.")
            # print("goals: ",goal_x,goal_y,goal_z)
            # print("goal_pose_msgs:", goal_pose_msg.pose.position.x,goal_pose_msg.pose.position.y)

        
        ##matrix about world -> base
        angle_w_b = math.atan2(goal_pose_msg.pose.position.y , goal_pose_msg.pose.position.x)
        under_matrix = np.array([[0,0,0,1]])
        
        R_matrix_w_b = np.array([[math.cos(angle_w_b), -math.sin(angle_w_b), 0],
                                 [math.sin(angle_w_b), math.cos(angle_w_b), 0],
                                 [0, 0, 1]])
        
        P_matrix_w_b = np.array([[goal_pose_msg.pose.position.x],
                                [goal_pose_msg.pose.position.y],
                                [0]])
        
        T_matrix_w_b = np.hstack((R_matrix_w_b, P_matrix_w_b))
        T_matrix_w_b = np.vstack((T_matrix_w_b, under_matrix))


        ##matrix about base -> world
        R_matrix_b_w = np.linalg.inv(R_matrix_w_b)
        P_matrix_b_w = -np.dot(R_matrix_b_w, P_matrix_w_b)
        T_matrix_b_w = np.hstack((R_matrix_b_w, R_matrix_b_w.dot(P_matrix_w_b)))
        T_matrix_b_w = np.vstack((T_matrix_b_w, under_matrix))

        
        ##matrix about base -> manipulator
        R_matrix_b_m = np.array([[0, 1, 0],
                                [-1, 0, 0],
                                [0, 0, 1]])
        
        P_matrix_b_m = np.array([[0.331200],
                                [0.00],
                                [0.403400]])
        
        T_matrix_b_m = np.hstack((R_matrix_b_m, P_matrix_b_m))
        T_matrix_b_m = np.vstack((T_matrix_b_m, under_matrix))
        
        
        
        ##matrix about manipulator -> base

        R_matrix_m_b = np.linalg.inv(R_matrix_b_m)
        P_matrix_m_b = -np.dot(R_matrix_m_b, P_matrix_b_m)
        
        T_matrix_m_b = np.hstack((R_matrix_m_b, P_matrix_m_b))
        T_matrix_m_b = np.vstack((T_matrix_m_b, under_matrix))


        ##matrix about world->manipulator
        T_matrix_w_m = np.dot(T_matrix_w_b, T_matrix_b_m)

        ##matrix about world to desired
        T_matrix_w_d = np.array([[1, 0, 0, goal_x,],
                                [0, 1, 0, goal_y],
                                [0, 0, 1, goal_z],
                                [0, 0, 0, 1]])
        
        P_matrix_w_d = np.array([[goal_x],
                                [goal_y],
                                [goal_z],
                                [1]])
        
        ## desired point frame of manipulator
        P_matrix_m_d = np.dot(T_matrix_m_b, np.dot(T_matrix_b_w, P_matrix_w_d))
        # print(T_matrix_m_b)
        # print(T_matrix_b_w)
        # print(P_matrix_w_d)
        # print(P_matrix_m_d)

        
        desired_mani_x = float(P_matrix_m_d[0])
        desired_mani_y = float(P_matrix_m_d[1])
        desired_mani_z = float(P_matrix_m_d[2])

        # print("goals: ",goal_x,goal_y,goal_z)
        # print("goal_pose_msgs:", (goal_x - scaled_direction_vector.x), (goal_y - scaled_direction_vector.y))
        # print("desired_manis : ", desired_mani_x, desired_mani_y, desired_mani_z)

        self.calculate_joint_positions(desired_mani_x, desired_mani_y, desired_mani_z)

        traj_msg = JointTrajectory()
        traj_msg.header.stamp = rospy.Time.now()
        traj_msg.joint_names = ["elbow_joint", "shoulder_lift_joint", "shoulder_pan_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

        point = JointTrajectoryPoint()
        point.positions = self.calculate_joint_positions(desired_mani_x, desired_mani_y, desired_mani_z)



        
        print("point.positions :", point.positions)
        point.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # Assuming zero velocities
        point.time_from_start = rospy.Duration(1)  # Example duration
        traj_msg.points.append(point)

        while not rospy.is_shutdown():
            traj_msg.header.stamp = rospy.Time.now()  # Update timestamp
            self.pub.publish(traj_msg)
            self.rate.sleep()

        
        
    def calculate_joint_positions(self, x, y, z):

        my_chain = Chain.from_urdf_file("/home/hsh/catkin_ws/src/ur_ikfast/ur3/ur3.urdf")

        desired_joints = my_chain.inverse_kinematics([x, y, z])
        print("desired_joints:", desired_joints)  # 추가된 부분
        
        joint_positions = [
            float(desired_joints[0]),
            float(desired_joints[1]),
            float(desired_joints[2]),
            float(desired_joints[3]),
            float(desired_joints[4]),
            float(desired_joints[5]),
        ]

        print("joint_positions : ", joint_positions)
        
        return joint_positions

if __name__ == '__main__':
    try:
        publisher = InitialPoseAndGoalPublisher()
        while publisher.initial_pose_msg is None:  # 초기값을 받을 때까지 대기
            rospy.sleep(0.1)
        publisher.publish_initial_pose()

        goal_x = float(input("목표 x 좌표를 입력하세요: "))
        goal_y = float(input("목표 y 좌표를 입력하세요: "))
        goal_z = float(input("목표 z 좌표를 입력하세요: "))
        publisher.publish_goal(goal_x, goal_y, goal_z)

    except rospy.ROSInterruptException:
        pass
