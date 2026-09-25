#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose


class AddPandaScene(Node):

    def __init__(self):
        super().__init__('add_panda_scene')

        self.publisher = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )

        # Give MoveIt time to connect
        self.timer = self.create_timer(2.0, self.add_objects)
        self.scene_added = False

    def add_objects(self):

        if self.scene_added:
            return

        scene = PlanningScene()
        scene.is_diff = True

        # ==========================================
        # TABLE
        # ==========================================

        table = CollisionObject()

        table.header.frame_id = 'panda_link0'
        table.id = 'table'

        table_box = SolidPrimitive()
        table_box.type = SolidPrimitive.BOX

        # X, Y, Z dimensions
        table_box.dimensions = [
            1.0,
            1.0,
            0.10
        ]

        table_pose = Pose()

        table_pose.position.x = 0.50
        table_pose.position.y = 0.0
        table_pose.position.z = -0.05

        table_pose.orientation.w = 1.0

        table.primitives.append(table_box)
        table.primitive_poses.append(table_pose)

        table.operation = CollisionObject.ADD

        # ==========================================
        # CUBE / BLOCK
        # ==========================================

        cube = CollisionObject()

        cube.header.frame_id = 'panda_link0'
        cube.id = 'pick_cube'

        cube_box = SolidPrimitive()
        cube_box.type = SolidPrimitive.BOX

        # 5 cm cube
        cube_box.dimensions = [
            0.05,
            0.05,
            0.05
        ]

        cube_pose = Pose()

        # In front of Panda
        cube_pose.position.x = 0.45
        cube_pose.position.y = 0.0
        cube_pose.position.z = 0.025

        cube_pose.orientation.w = 1.0

        cube.primitives.append(cube_box)
        cube.primitive_poses.append(cube_pose)

        cube.operation = CollisionObject.ADD

        # ==========================================
        # ADD TO PLANNING SCENE
        # ==========================================

        scene.world.collision_objects.append(table)
        scene.world.collision_objects.append(cube)

        self.publisher.publish(scene)

        self.scene_added = True

        self.get_logger().info(
            'Table and pick cube added to MoveIt planning scene.'
        )


def main(args=None):

    rclpy.init(args=args)

    node = AddPandaScene()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
