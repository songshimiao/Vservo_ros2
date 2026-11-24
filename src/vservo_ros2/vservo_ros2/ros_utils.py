# ROS 公共小工具：参数、时间戳、锁存 QoS、标定板参数和节点生命周期。
# LATCHED 用于内参/元数据等低频状态，使晚加入的订阅者也能收到最后一份数据。
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

LATCHED = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)


# 声明参数并取值；调用位置给出的 default 可由 launch/YAML 覆盖。
def param(node, name, default):
    return node.declare_parameter(name, default).value


# 把 ROS 的整数秒/纳秒转成浮点秒，便于做时效比较。
def stamp_s(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


# 只接受年龄位于 [0, max_age] 的消息，未来时间戳也视为无效。
def fresh(node, stamp, max_age):
    age = node.get_clock().now().nanoseconds * 1e-9 - stamp_s(stamp)
    return 0 <= age <= max_age


# 按节点参数构造 ChArUco 模型，所有检测和采集应使用同一实物尺寸。
def board_from_params(node):
    from .board import Board
    return Board(param(node, 'squares_x', 7), param(node, 'squares_y', 5),
                 param(node, 'square_m', 0.015), param(node, 'marker_m', 0.011),
                 param(node, 'dictionary', 'DICT_4X4_50'))


# 统一创建、运行、销毁节点；即使初始化或运行异常，也清理已创建节点和 ROS 上下文。
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
