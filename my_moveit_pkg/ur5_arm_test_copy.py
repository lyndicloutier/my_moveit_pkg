# Copyright 2022 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Denis Štogl, Lovro Ivanov
#

# pylint: disable=missing-function-docstring, missing-class-docstring, missing-module-docstring, line-too-long, trailing-whitespace, consider-using-f-string

import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile
from builtin_interfaces.msg import Duration
from rcl_interfaces.msg import ParameterDescriptor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
# receiving position values from PoseStamped (geometry.msgs)
from geometry_msgs.msg import PoseStamped

class UR5ArmTest(Node):
    def __init__(self):
        super().__init__("ur5_arm_test_copy")

        self._allow_undeclared_parameters = True

        # self.get_logger().info(f'"{self._parameters} {self._parameter_overrides}"')

        self.declare_parameters(namespace = "", 
                                parameters = [("controller_name", rclpy.Parameter.Type.STRING),
                                                ("wait_sec_between_publish", rclpy.Parameter.Type.INTEGER), 
                                                ("goal_names", rclpy.Parameter.Type.STRING_ARRAY), 
                                                ("joints", rclpy.Parameter.Type.STRING_ARRAY), 
                                                ("check_starting_point", rclpy.Parameter.Type.BOOL)])

        # Read parameters
        controller_name = self.get_parameter("controller_name").value
        wait_sec_between_publish = self.get_parameter("wait_sec_between_publish").value
        goal_names = self.get_parameter("goal_names").value
        self.joints = self.get_parameter("joints").value
        self.check_starting_point = self.get_parameter("check_starting_point").value
        self.starting_point = {}

        # if joints is empty
        if self.joints is None or len(self.joints) == 0:
            raise ValueError('"joints" parameter is not set!')

        # starting point stuff
        if self.check_starting_point:
            # declare nested params
            for name in self.joints:
                param_name_tmp = "starting_point_limits" + "." + name
                self.declare_parameter(param_name_tmp, [-2 * 3.14159, 2 * 3.14159])
                self.starting_point[name] = self.get_parameter(param_name_tmp).value

            for name in self.joints:
                if len(self.starting_point[name]) != 2:
                    raise ValueError('"starting_point" parameter is not set correctly!')
            self.joint_state_sub = self.create_subscription(JointState, "joint_states", self.joint_state_callback, 10)

        # initialize starting point status
        self.starting_point_ok = not self.check_starting_point

        self.joint_state_msg_received = False

        # Read all positions from parameters
        self.goals = []  # List of JointTrajectoryPoint
        for goal_name in goal_names:
            self.declare_parameter(goal_name, descriptor=ParameterDescriptor(dynamic_typing=True))
            goal = self.get_parameter(goal_name).value
            
            # remove this "if" part in ROS Iron
            if isinstance(goal, list):
                self.get_logger().warn(
                    f'Goal "{goal_name}" is defined as a list. This is deprecated. '
                    "Use the following structure:\n<goal_name>:\n  "
                    "positions: [joint1, joint2, joint3, ...]\n  "
                    "velocities: [v_joint1, v_joint2, ...]\n  "
                    "accelerations: [a_joint1, a_joint2, ...]\n  "
                    "effort: [eff_joint1, eff_joint2, ...]"
                )

                if goal is None or len(goal) == 0:
                    raise ValueError(f'Values for goal "{goal_name}" not set!')

                float_goal = [float(value) for value in goal]

                point = JointTrajectoryPoint()
                point.positions = float_goal
                point.time_from_start = Duration(sec=4)

                self.goals.append(point)

            else:
                point = JointTrajectoryPoint()

                def get_sub_param(sub_param, name):
                    param_name = name + "." + sub_param
                    self.declare_parameter(param_name, [float()])
                    param_value = self.get_parameter(param_name).value

                    float_values = []

                    if len(param_value) != len(self.joints):
                        return [False, float_values]

                    float_values = [float(value) for value in param_value]

                    return [True, float_values]
                
                # CHECKING FOR POSITION, VELOCITY, ACCELERATION, AND EFFORT VALUES

                one_ok = False

                [ok, values] = get_sub_param("positions", goal_name)
                if ok:
                    point.positions = values
                    one_ok = True

                [ok, values] = get_sub_param("velocities", goal_name)
                if ok:
                    point.velocities = values
                    one_ok = True

                [ok, values] = get_sub_param("accelerations", goal_name)
                if ok:
                    point.accelerations = values
                    one_ok = True

                [ok, values] = get_sub_param("effort", goal_name)
                if ok:
                    point.effort = values
                    one_ok = True

                if one_ok:
                    point.time_from_start = Duration(sec=4)
                    self.goals.append(point)
                    self.get_logger().info(f'Goal "{goal_name}" has definition {point}')

                else:
                    self.get_logger().warn(
                        f'Goal "{goal_name}" definition is wrong. This goal will not be used. '
                        "Use the following structure: \n<goal_name>:\n  "
                        "positions: [joint1, joint2, joint3, ...]\n  "
                        "velocities: [v_joint1, v_joint2, ...]\n  "
                        "accelerations: [a_joint1, a_joint2, ...]\n  "
                        "effort: [eff_joint1, eff_joint2, ...]"
                    )

                # END CHECK FOR VALUES

        if len(self.goals) < 1:
            self.get_logger().error("No valid goal found. Exiting...")
            exit(1)

        publish_topic = "/" + controller_name + "/" + "joint_trajectory"

        self.get_logger().info(
            f"Publishing goal on topic '{publish_topic}' every "
            f"{wait_sec_between_publish} s"
        )

        self.publisher_ = self.create_publisher(JointTrajectory, publish_topic, 1)

        # NEW CODE
        self.subscriber_ = self.create_subscription(PoseStamped, '/posestamped', self.position_callback, QoSProfile(depth = 10, reliability = ReliabilityPolicy.RELIABLE))

        self.timer = self.create_timer(wait_sec_between_publish, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        if self.starting_point_ok:

            self.get_logger().info(f"Sending goal {self.goals[self.i]}.")

            traj = JointTrajectory()
            traj.joint_names = self.joints
            traj.points.append(self.goals[self.i])
            self.publisher_.publish(traj)

            self.i += 1
            self.i %= len(self.goals)

        elif self.check_starting_point and not self.joint_state_msg_received:
            self.get_logger().warn(
                'Start configuration could not be checked! Check "joint_state" topic!'
            )
        else:
            self.get_logger().warn("Start configuration is not within configured limits!")

    def joint_state_callback(self, msg):
        if not self.joint_state_msg_received:

            # check start state
            limit_exceeded = [False] * len(msg.name)
            for idx, enum in enumerate(msg.name):
                if (msg.position[idx] < self.starting_point[enum][0]) or (
                    msg.position[idx] > self.starting_point[enum][1]
                ):
                    self.get_logger().warn(f"Starting point limits exceeded for joint {enum} !")
                    limit_exceeded[idx] = True

            if any(limit_exceeded):
                self.starting_point_ok = False
            else:
                self.starting_point_ok = True

            self.joint_state_msg_received = True
        else:
            return
    
    # function used for subscriber / NEW CODE
    def position_callback(self, msg):
        new_msg = PoseStamped()
        new_msg.pose.position.x = msg.pose.position.x
        new_msg.pose.position.y = msg.pose.position.y
        new_msg.pose.position.z = msg.pose.position.z
        new_msg.pose.orientation.x = msg.pose.orientation.x
        new_msg.pose.orientation.y = msg.pose.orientation.y
        new_msg.pose.orientation.z = msg.pose.orientation.z
        new_msg.header.frame_id =msg.header.frame_id
        self.get_logger().info("Received: position = (" + str(msg.pose.position.x) + ", " + str(msg.pose.position.y) 
                               + ", " + str(msg.pose.position.z) + ", " + str(msg.pose.orientation.x) 
                               + ", " + str(msg.pose.orientation.y) + ", " + str(msg.pose.orientation.z) 
                               + ") and frame_id = " + msg.header.frame_id + ", Publishing: position = (" + str(new_msg.pose.position.x) + ", " + str(new_msg.pose.position.y) 
                               + ", " + str(new_msg.pose.position.z) + ", " + str(new_msg.pose.orientation.x) 
                               + ", " + str(new_msg.pose.orientation.y) + ", " + str(new_msg.pose.orientation.z) 
                               + ") and frame_id = " + new_msg.header.frame_id)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    ur5_arm = UR5ArmTest()
    rclpy.spin(ur5_arm)
    ur5_arm.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
