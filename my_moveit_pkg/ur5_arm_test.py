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

# pylint: disable=unused-import, unused-argument, missing-function-docstring, global-statement, no-name-in-module, invalid-name, missing-class-docstring, missing-module-docstring, line-too-long, trailing-whitespace, consider-using-f-string, unused-variable, unspecified-encoding, missing-final-newline

# imports used
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit.planning import MoveItPy
from ur5_arm_msgs.srv import UR5ServiceMessage


# class for the node
class UR5ArmTest(Node):

    # declare variables for goal position
    frame_id = None
    orientation_w = None
    position_x = None
    position_y = None
    position_z = None

    # function to initialize the node
    def __init__(self):
        # initialize the ur5_arm_test node
        super().__init__("ur5_arm_test")

        self._allow_undeclared_parameters = True

        logger = rclpy.logging.get_logger("ur5_arm_test")

        self.declare_parameters(namespace = "robot_description_kinematics", 
                                parameters = [("kinematics_solver", rclpy.Parameter.Type.STRING),
                                                ("kinematics_solver_search_resolution", rclpy.Parameter.Type.DOUBLE), 
                                                ("kinematics_solver_timeout", rclpy.Parameter.Type.DOUBLE), 
                                                ("kinematics_solver_attempts", rclpy.Parameter.Type.INTEGER)])

        # initialize the MoveItPy node
        self.get_logger().info('before MoveItPy')
        ur5 = MoveItPy(node_name = "ur5")
        self.get_logger().info('after MoveItPy')

        # get planning component
        ur_manipulator = ur5.get_planning_component("ur_manipulator")

        logger.info("MoveItPy instance created")

        # create the service server object
        self.service = self.create_service(UR5ServiceMessage, '/posestamped', self.listener_callback)

        # set plan start state to current state
        ur_manipulator.set_start_state_to_current_state()

        # set pose goal with PoseStamped message
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = self.frame_id
        pose_goal.pose.orientation.w = self.orientation_w
        pose_goal.pose.position.x = self.position_x
        pose_goal.pose.position.y = self.position_y
        pose_goal.pose.position.z = self.position_z

        # set the goal state to the goal positions that were previously set
        ur_manipulator.set_goal_state(pose_stamped_msg = pose_goal, pose_link = "base_link")

        # plan to goal
        plan_and_execute(ur5, ur_manipulator, logger)

    # function to receive the goal position from the service provider
    def listener_callback(self, request : UR5ServiceMessage.Request, response : UR5ServiceMessage.Response):
        # print the log info in the terminal
        self.get_logger().info("Received request\n")

        # set goal position values to the values from the service provider
        self.frame_id = request.position.header.frame_id
        self.orientation_w = request.position.pose.orientation.w
        self.position_x = request.position.pose.position.x
        self.position_y = request.position.pose.position.y
        self.position_z = request.position.pose.position.z

        return response

# function to plan and execute the trajectory of the robot
def plan_and_execute(robot, planning_component, logger, single_plan_parameters = None, multi_plan_parameters = None):
    logger.info("Planning trajectory")
    # if using multiplan parameters
    if multi_plan_parameters is not None:
        # plan trajectory with multiplan paramters
        plan_result = planning_component.plan(multi_plan_parameters = multi_plan_parameters)
    # if using single plan parameters
    elif single_plan_parameters is not None:
        # plan trajectory with single plan paramters
        plan_result = planning_component.plan(single_plan_parameters = single_plan_parameters)
    else:
        # plan normally
        plan_result = planning_component.plan()
    
    if plan_result:
        # execute the robot trajectory
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers = [])
    else:
        # planning failed
        logger.error("Planning failed")

def main(args=None):
    rclpy.init(args=args)

    # create the node
    ur5_arm_test = UR5ArmTest()

    # spin the node
    rclpy.spin(ur5_arm_test)

    # shut the node down
    ur5_arm_test.get_logger().info("Shutting down\n")
    rclpy.shutdown()

if __name__ == "__main__":
    main()