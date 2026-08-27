#!/usr/bin/env python3
"""将 Gazebo 中的简化风机障碍物同步为 MoveIt 碰撞物."""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from shape_msgs.msg import SolidPrimitive


class WindTurbineScene(Node):
    def __init__(self):
        super().__init__('wind_turbine_scene')
        self.publisher = self.create_publisher(PlanningScene, '/planning_scene', 10)
        self.timer = self.create_timer(1.0, self.publish_scene)
        self.publish_scene()

    @staticmethod
    def primitive(object_id, primitive_type, dimensions, position):
        obj = CollisionObject()
        obj.id = object_id
        obj.header.frame_id = 'base_footprint'
        primitive = SolidPrimitive()
        primitive.type = primitive_type
        primitive.dimensions = dimensions
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = position
        pose.orientation.w = 1.0
        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD
        return obj

    def publish_scene(self):
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [
            self.primitive(
                'turbine_tower', SolidPrimitive.CYLINDER,
                [1.6, 0.12], (0.45, 0.0, 0.8)),
            self.primitive(
                'turbine_blade', SolidPrimitive.BOX,
                [1.1, 0.10, 0.08], (0.72, 0.0, 1.35)),
        ]
        self.publisher.publish(scene)


def main(args=None):
    rclpy.init(args=args)
    node = WindTurbineScene()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
