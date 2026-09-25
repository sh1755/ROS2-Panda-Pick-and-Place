#!/usr/bin/env python3

import time
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint


class PandaPickPlace(Node):

    def __init__(self):
        super().__init__('panda_pick_place')

        self.get_logger().info(
            'Starting Panda Pick-and-Place Demo'
        )

        # Connect to MoveIt 2
        self.move_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        self.get_logger().info(
            'Waiting for MoveIt /move_action...'
        )

        self.move_client.wait_for_server()

        self.get_logger().info(
            'MoveIt connected successfully.'
        )

        # Panda 7-DOF joints
        self.joint_names = [
            'panda_joint1',
            'panda_joint2',
            'panda_joint3',
            'panda_joint4',
            'panda_joint5',
            'panda_joint6',
            'panda_joint7'
        ]

    # =========================================================
    # MOVE PANDA ARM
    # =========================================================

    def move_joints(self, positions, target_name):

        self.get_logger().info(
            f'Moving to: {target_name}'
        )

        goal = MoveGroup.Goal()

        # MoveIt planning group
        goal.request.group_name = 'panda_arm'

        # Planning parameters
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0

        # Slow speed for demonstration
        goal.request.max_velocity_scaling_factor = 0.20
        goal.request.max_acceleration_scaling_factor = 0.20

        # Create joint constraints
        constraints = Constraints()

        for joint_name, position in zip(
            self.joint_names,
            positions
        ):

            joint_constraint = JointConstraint()

            joint_constraint.joint_name = joint_name
            joint_constraint.position = float(position)

            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01

            joint_constraint.weight = 1.0

            constraints.joint_constraints.append(
                joint_constraint
            )

        goal.request.goal_constraints.append(
            constraints
        )

        # Plan and execute
        goal.planning_options.plan_only = False

        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        # Send goal to MoveIt
        send_goal_future = (
            self.move_client.send_goal_async(goal)
        )

        rclpy.spin_until_future_complete(
            self,
            send_goal_future
        )

        goal_handle = send_goal_future.result()

        if goal_handle is None:

            self.get_logger().error(
                f'No goal handle: {target_name}'
            )

            return False

        if not goal_handle.accepted:

            self.get_logger().error(
                f'Goal rejected: {target_name}'
            )

            return False

        self.get_logger().info(
            f'Goal accepted: {target_name}'
        )

        # Wait for execution
        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        wrapped_result = result_future.result()

        if wrapped_result is None:

            self.get_logger().error(
                f'No result: {target_name}'
            )

            return False

        result = wrapped_result.result

        error_code = result.error_code.val

        if error_code == 1:

            self.get_logger().info(
                f'SUCCESS: {target_name}'
            )

            return True

        else:

            self.get_logger().error(
                f'MoveIt failed at {target_name}. '
                f'Error code = {error_code}'
            )

            return False

    # =========================================================
    # GRIPPER
    # =========================================================

    def open_gripper(self):

        self.get_logger().info(
            'GRIPPER -> OPEN'
        )

        # Temporary placeholder.
        # Actual panda_hand_controller will be added next.

        time.sleep(1)

    def close_gripper(self):

        self.get_logger().info(
            'GRIPPER -> CLOSE'
        )

        # Temporary placeholder.

        time.sleep(1)

    # =========================================================
    # PICK AND PLACE
    # =========================================================

    def run_task(self):

        self.get_logger().info(
            '==================================='
        )

        self.get_logger().info(
            'PANDA PICK AND PLACE STARTING'
        )

        self.get_logger().info(
            '==================================='
        )

        # -----------------------------------------------------
        # HOME POSITION
        # -----------------------------------------------------

        home = [
            0.0,
            -0.785,
            0.0,
            -2.356,
            0.0,
            1.571,
            0.785
        ]

        # -----------------------------------------------------
        # PRE-GRASP
        # -----------------------------------------------------

        pre_grasp = [
            0.20,
            -0.60,
            0.10,
            -2.10,
            0.00,
            1.55,
            0.90
        ]

        # -----------------------------------------------------
        # GRASP
        # -----------------------------------------------------

        grasp = [
            0.20,
            -0.35,
            0.10,
            -1.90,
            0.00,
            1.60,
            0.90
        ]

        # -----------------------------------------------------
        # LIFT
        # -----------------------------------------------------

        lift = [
            0.20,
            -0.65,
            0.10,
            -2.00,
            0.00,
            1.50,
            0.90
        ]

        # -----------------------------------------------------
        # PRE-PLACE
        # -----------------------------------------------------

        pre_place = [
            -0.45,
            -0.55,
            0.10,
            -2.05,
            0.00,
            1.55,
            0.55
        ]

        # -----------------------------------------------------
        # PLACE
        # -----------------------------------------------------

        place = [
            -0.45,
            -0.30,
            0.10,
            -1.85,
            0.00,
            1.55,
            0.55
        ]

        # =====================================================
        # TASK SEQUENCE
        # =====================================================

        # STEP 1
        self.open_gripper()

        # STEP 2
        if not self.move_joints(
            home,
            'HOME'
        ):
            return

        time.sleep(1)

        # STEP 3
        if not self.move_joints(
            pre_grasp,
            'PRE-GRASP'
        ):
            return

        time.sleep(1)

        # STEP 4
        if not self.move_joints(
            grasp,
            'GRASP POSITION'
        ):
            return

        time.sleep(1)

        # STEP 5
        self.close_gripper()

        # STEP 6
        if not self.move_joints(
            lift,
            'LIFT'
        ):
            return

        time.sleep(1)

        # STEP 7
        if not self.move_joints(
            pre_place,
            'PRE-PLACE'
        ):
            return

        time.sleep(1)

        # STEP 8
        if not self.move_joints(
            place,
            'PLACE'
        ):
            return

        time.sleep(1)

        # STEP 9
        self.open_gripper()

        # STEP 10
        if not self.move_joints(
            pre_place,
            'RETREAT'
        ):
            return

        time.sleep(1)

        # STEP 11
        if not self.move_joints(
            home,
            'HOME'
        ):
            return

        self.get_logger().info(
            '==================================='
        )

        self.get_logger().info(
            'PICK AND PLACE COMPLETE'
        )

        self.get_logger().info(
            '==================================='
        )


# =============================================================
# MAIN
# =============================================================

def main(args=None):

    rclpy.init(args=args)

    node = PandaPickPlace()

    try:

        node.run_task()

    except KeyboardInterrupt:

        node.get_logger().info(
            'Pick-and-place interrupted.'
        )

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
