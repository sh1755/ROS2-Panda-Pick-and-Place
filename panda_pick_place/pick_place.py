#!/usr/bin/env python3

import time
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import Pose

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    PlanningScene,
    CollisionObject,
    AttachedCollisionObject,
)

from moveit_msgs.srv import ApplyPlanningScene

from shape_msgs.msg import SolidPrimitive
from control_msgs.action import GripperCommand


class PandaPickPlace(Node):

    def __init__(self):

        super().__init__('panda_pick_place')

        # ==================================================
        # MOVEIT ARM
        # ==================================================

        self.move_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        # ==================================================
        # PANDA GRIPPER
        # ==================================================

        self.gripper_client = ActionClient(
            self,
            GripperCommand,
            '/panda_hand_controller/gripper_cmd'
        )

        # ==================================================
        # PLANNING SCENE
        # ==================================================

        self.scene_client = self.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene'
        )

        self.group_name = 'panda_arm'
        self.base_frame = 'panda_link0'
        self.ee_link = 'panda_hand'

        # ==================================================
        # TABLE
        # ==================================================

        self.table_x = 0.40
        self.table_y = 0.00
        self.table_z = 0.10

        self.table_length = 0.70
        self.table_width = 0.70
        self.table_height = 0.05

        # ==================================================
        # CUBE
        # ==================================================

        self.cube_x = 0.45
        self.cube_y = 0.00

        self.cube_size = 0.05

        # Table top:
        # 0.10 + 0.025 = 0.125
        #
        # Cube centre:
        # 0.125 + 0.025 = 0.150

        self.cube_z = 0.15

        # ==================================================
        # PLACE POSITION
        # ==================================================

        self.place_x = 0.35
        self.place_y = -0.25
        self.place_z = 0.15

    # ======================================================
    # MOVE ROBOT
    # ======================================================

    def move_to_pose(
        self,
        x,
        y,
        z,
        name='POSE'
    ):

        self.get_logger().info(
            f'Moving to {name}: '
            f'x={x:.3f}, '
            f'y={y:.3f}, '
            f'z={z:.3f}'
        )

        self.move_client.wait_for_server()

        goal = MoveGroup.Goal()

        goal.request.group_name = (
            self.group_name
        )

        goal.request.num_planning_attempts = 20

        goal.request.allowed_planning_time = 10.0

        # ==================================================
        # POSITION CONSTRAINT
        # ==================================================

        position_constraint = PositionConstraint()

        position_constraint.header.frame_id = (
            self.base_frame
        )

        position_constraint.link_name = (
            self.ee_link
        )

        bounding_volume = BoundingVolume()

        sphere = SolidPrimitive()

        sphere.type = SolidPrimitive.SPHERE

        # Position tolerance
        sphere.dimensions = [0.02]

        target_pose = Pose()

        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z

        target_pose.orientation.w = 1.0

        bounding_volume.primitives.append(
            sphere
        )

        bounding_volume.primitive_poses.append(
            target_pose
        )

        position_constraint.constraint_region = (
            bounding_volume
        )

        position_constraint.weight = 1.0

        # ==================================================
        # ORIENTATION
        # ==================================================

        orientation_constraint = OrientationConstraint()

        orientation_constraint.header.frame_id = (
            self.base_frame
        )

        orientation_constraint.link_name = (
            self.ee_link
        )

        # Downward-facing Panda hand
        orientation_constraint.orientation.x = 1.0
        orientation_constraint.orientation.y = 0.0
        orientation_constraint.orientation.z = 0.0
        orientation_constraint.orientation.w = 0.0

        orientation_constraint.absolute_x_axis_tolerance = 0.30
        orientation_constraint.absolute_y_axis_tolerance = 0.30
        orientation_constraint.absolute_z_axis_tolerance = 0.30

        orientation_constraint.weight = 1.0

        # ==================================================
        # GOAL CONSTRAINT
        # ==================================================

        constraints = Constraints()

        constraints.position_constraints.append(
            position_constraint
        )

        constraints.orientation_constraints.append(
            orientation_constraint
        )

        goal.request.goal_constraints.append(
            constraints
        )

        # ==================================================
        # SEND GOAL
        # ==================================================

        future = self.move_client.send_goal_async(
            goal
        )

        rclpy.spin_until_future_complete(
            self,
            future
        )

        goal_handle = future.result()

        if goal_handle is None:

            self.get_logger().error(
                f'{name}: No goal handle'
            )

            return False

        if not goal_handle.accepted:

            self.get_logger().error(
                f'{name}: Goal rejected'
            )

            return False

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        result = (
            result_future.result().result
        )

        if result.error_code.val == 1:

            self.get_logger().info(
                f'SUCCESS: {name}'
            )

            return True

        self.get_logger().error(
            f'FAILED: {name} '
            f'error={result.error_code.val}'
        )

        return False

    # ======================================================
    # GRIPPER
    # ======================================================

    def command_gripper(
        self,
        position,
        name
    ):

        self.get_logger().info(
            f'{name} GRIPPER'
        )

        self.gripper_client.wait_for_server()

        goal = GripperCommand.Goal()

        goal.command.position = position
        goal.command.max_effort = 20.0

        future = (
            self.gripper_client.send_goal_async(
                goal
            )
        )

        rclpy.spin_until_future_complete(
            self,
            future
        )

        goal_handle = future.result()

        if goal_handle is None:

            self.get_logger().error(
                'No gripper goal handle'
            )

            return False

        if not goal_handle.accepted:

            self.get_logger().error(
                'Gripper command rejected'
            )

            return False

        result_future = (
            goal_handle.get_result_async()
        )

        rclpy.spin_until_future_complete(
            self,
            result_future
        )

        result = (
            result_future.result().result
        )

        self.get_logger().info(
            f'Gripper position: '
            f'{result.position:.3f}'
        )

        time.sleep(0.5)

        return True

    def open_gripper(self):

        return self.command_gripper(
            0.04,
            'OPENING'
        )

    def close_gripper(self):

        return self.command_gripper(
            0.00,
            'CLOSING'
        )

    # ======================================================
    # ADD TABLE + CUBE
    # ======================================================

    def add_scene(self):

        self.get_logger().info(
            'ADDING TABLE AND CUBE'
        )

        self.scene_client.wait_for_service()

        scene = PlanningScene()
        scene.is_diff = True

        # ==================================================
        # TABLE
        # ==================================================

        table = CollisionObject()

        table.header.frame_id = (
            self.base_frame
        )

        table.id = 'table'

        table_box = SolidPrimitive()

        table_box.type = SolidPrimitive.BOX

        table_box.dimensions = [
            self.table_length,
            self.table_width,
            self.table_height
        ]

        table_pose = Pose()

        table_pose.position.x = (
            self.table_x
        )

        table_pose.position.y = (
            self.table_y
        )

        table_pose.position.z = (
            self.table_z
        )

        table_pose.orientation.w = 1.0

        table.primitives.append(
            table_box
        )

        table.primitive_poses.append(
            table_pose
        )

        table.operation = (
            CollisionObject.ADD
        )

        scene.world.collision_objects.append(
            table
        )

        # ==================================================
        # CUBE
        # ==================================================

        cube = CollisionObject()

        cube.header.frame_id = (
            self.base_frame
        )

        cube.id = 'pick_cube'

        cube_box = SolidPrimitive()

        cube_box.type = SolidPrimitive.BOX

        cube_box.dimensions = [
            self.cube_size,
            self.cube_size,
            self.cube_size
        ]

        cube_pose = Pose()

        cube_pose.position.x = (
            self.cube_x
        )

        cube_pose.position.y = (
            self.cube_y
        )

        cube_pose.position.z = (
            self.cube_z
        )

        cube_pose.orientation.w = 1.0

        cube.primitives.append(
            cube_box
        )

        cube.primitive_poses.append(
            cube_pose
        )

        cube.operation = (
            CollisionObject.ADD
        )

        scene.world.collision_objects.append(
            cube
        )

        # ==================================================
        # APPLY SCENE
        # ==================================================

        request = (
            ApplyPlanningScene.Request()
        )

        request.scene = scene

        future = (
            self.scene_client.call_async(
                request
            )
        )

        rclpy.spin_until_future_complete(
            self,
            future
        )

        self.get_logger().info(
            'TABLE AND CUBE ADDED'
        )

        time.sleep(2.0)

    # ======================================================
    # ATTACH CUBE
    # ======================================================

    def attach_cube(self):

        self.get_logger().info(
            'ATTACHING CUBE'
        )

        scene = PlanningScene()
        scene.is_diff = True

        # ==================================================
        # REMOVE FROM WORLD
        # ==================================================

        remove_cube = CollisionObject()

        remove_cube.header.frame_id = (
            self.base_frame
        )

        remove_cube.id = 'pick_cube'

        remove_cube.operation = (
            CollisionObject.REMOVE
        )

        scene.world.collision_objects.append(
            remove_cube
        )

        # ==================================================
        # ATTACH TO HAND
        # ==================================================

        attached = AttachedCollisionObject()

        attached.link_name = (
            self.ee_link
        )

        attached.object.header.frame_id = (
            self.ee_link
        )

        attached.object.id = (
            'pick_cube'
        )

        primitive = SolidPrimitive()

        primitive.type = SolidPrimitive.BOX

        primitive.dimensions = [
            self.cube_size,
            self.cube_size,
            self.cube_size
        ]

        attached.object.primitives.append(
            primitive
        )

        attach_pose = Pose()

        attach_pose.position.x = 0.0
        attach_pose.position.y = 0.0

        # Cube near finger centre
        attach_pose.position.z = 0.06

        attach_pose.orientation.w = 1.0

        attached.object.primitive_poses.append(
            attach_pose
        )

        attached.object.operation = (
            CollisionObject.ADD
        )

        attached.touch_links = [
            'panda_hand',
            'panda_leftfinger',
            'panda_rightfinger'
        ]

        scene.robot_state.is_diff = True

        scene.robot_state.attached_collision_objects.append(
            attached
        )

        request = (
            ApplyPlanningScene.Request()
        )

        request.scene = scene

        future = (
            self.scene_client.call_async(
                request
            )
        )

        rclpy.spin_until_future_complete(
            self,
            future
        )

        self.get_logger().info(
            'CUBE ATTACHED'
        )

        time.sleep(1.0)

    # ======================================================
    # DETACH CUBE
    # ======================================================

    def detach_cube(self):

        self.get_logger().info(
            'DETACHING CUBE'
        )

        scene = PlanningScene()
        scene.is_diff = True

        # ==================================================
        # REMOVE ATTACHED CUBE
        # ==================================================

        attached = AttachedCollisionObject()

        attached.link_name = (
            self.ee_link
        )

        attached.object.id = (
            'pick_cube'
        )

        attached.object.operation = (
            CollisionObject.REMOVE
        )

        scene.robot_state.is_diff = True

        scene.robot_state.attached_collision_objects.append(
            attached
        )

        # ==================================================
        # ADD CUBE TO WORLD AT PLACE POSITION
        # ==================================================

        cube = CollisionObject()

        cube.header.frame_id = (
            self.base_frame
        )

        cube.id = 'pick_cube'

        cube_box = SolidPrimitive()

        cube_box.type = SolidPrimitive.BOX

        cube_box.dimensions = [
            self.cube_size,
            self.cube_size,
            self.cube_size
        ]

        cube_pose = Pose()

        cube_pose.position.x = (
            self.place_x
        )

        cube_pose.position.y = (
            self.place_y
        )

        cube_pose.position.z = (
            self.place_z
        )

        cube_pose.orientation.w = 1.0

        cube.primitives.append(
            cube_box
        )

        cube.primitive_poses.append(
            cube_pose
        )

        cube.operation = (
            CollisionObject.ADD
        )

        scene.world.collision_objects.append(
            cube
        )

        request = (
            ApplyPlanningScene.Request()
        )

        request.scene = scene

        future = (
            self.scene_client.call_async(
                request
            )
        )

        rclpy.spin_until_future_complete(
            self,
            future
        )

        self.get_logger().info(
            'CUBE PLACED'
        )

        time.sleep(1.0)

    # ======================================================
    # PICK AND PLACE TASK
    # ======================================================

    def run(self):

        self.get_logger().info(
            '================================'
        )

        self.get_logger().info(
            'PANDA PICK AND PLACE STARTED'
        )

        self.get_logger().info(
            '================================'
        )

        # ==================================================
        # 1. CREATE ENVIRONMENT
        # ==================================================

        self.add_scene()

        # ==================================================
        # 2. OPEN GRIPPER
        # ==================================================

        if not self.open_gripper():
            return

        # ==================================================
        # 3. PRE-GRASP
        # ==================================================

        if not self.move_to_pose(
            self.cube_x,
            self.cube_y,
            self.cube_z + 0.20,
            'PRE-GRASP'
        ):
            return

        # ==================================================
        # 4. APPROACH CUBE
        # ==================================================

        if not self.move_to_pose(
            self.cube_x,
            self.cube_y,
            self.cube_z + 0.14,
            'GRASP-DOWN'
        ):
            return

        # ==================================================
        # 5. CLOSE REAL SIMULATED FINGERS
        # ==================================================

        if not self.close_gripper():
            return

        # ==================================================
        # 6. ATTACH CUBE
        # ==================================================

        self.attach_cube()

        # ==================================================
        # 7. LIFT
        # ==================================================

        if not self.move_to_pose(
            self.cube_x,
            self.cube_y,
            self.cube_z + 0.35,
            'LIFT-UP'
        ):
            return

        # ==================================================
        # 8. MOVE ABOVE PLACE
        # ==================================================

        if not self.move_to_pose(
            self.place_x,
            self.place_y,
            self.place_z + 0.35,
            'PRE-PLACE'
        ):
            return

        # ==================================================
        # 9. LOWER
        # ==================================================

        if not self.move_to_pose(
            self.place_x,
            self.place_y,
            self.place_z + 0.14,
            'PLACE-DOWN'
        ):
            return

        # ==================================================
        # 10. OPEN FINGERS
        # ==================================================

        if not self.open_gripper():
            return

        # ==================================================
        # 11. DETACH
        # ==================================================

        self.detach_cube()

        # ==================================================
        # 12. RETREAT
        # ==================================================

        retreat_ok = self.move_to_pose(
            self.place_x,
            self.place_y,
            self.place_z + 0.25,
            'RETREAT-UP'
        )

        if not retreat_ok:

            self.get_logger().warn(
                'Retreat failed, but cube was placed.'
            )

        self.get_logger().info(
            '================================'
        )

        self.get_logger().info(
            'PICK AND PLACE COMPLETE'
        )

        self.get_logger().info(
            '================================'
        )


# ==========================================================
# MAIN
# ==========================================================

def main(args=None):

    rclpy.init(args=args)

    node = PandaPickPlace()

    try:

        time.sleep(2.0)

        node.run()

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
