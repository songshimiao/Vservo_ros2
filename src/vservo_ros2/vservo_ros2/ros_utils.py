import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


def param(node, name, default):
    return node.declare_parameter(name, default).value


def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def fresh(node, stamp, max_age):
    age = node.get_clock().now().nanoseconds * 1e-9 - stamp_s(stamp)
    return 0 <= age <= max_age


def board_from_params(node):
    from .board import Board
    return Board(param(node, 'squares_x', 7), param(node, 'squares_y', 5),
                 param(node, 'square_m', 0.015), param(node, 'marker_m', 0.011),
                 param(node, 'dictionary', 'DICT_4X4_50'))


def spin(factory, args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = factory()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
