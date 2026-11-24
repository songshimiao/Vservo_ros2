# 未来西瓜/瓜蒂识别的插件接口，目前尚无可执行的 YOLO 米制位姿实现。
"""Extension contract for future YOLO + metric pose backends.

Implement detect(bgr, K, D) -> board.Observation in an external plugin.
Use aligned depth / known CAD PnP / calibrated plane to recover metric 6D pose.
Pixel boxes alone are insufficient; do not invent depth or orientation.
Both camera plugins must return the same physical target ID and target axes.
"""


# 插件须与 Board/ArucoMarker 一样返回 Observation，并统一双相机目标 ID 和轴向。
class YoloPoseAdapter:
    # 输入 BGR 图像、K 和 D；实现者需用深度、已知 CAD 或标定平面恢复米制 6D 位姿。
    def detect(self, bgr, K, D):
        raise NotImplementedError('Install a YOLO + metric 6D backend; bounding boxes cannot drive PBVS')
