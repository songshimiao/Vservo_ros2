"""Extension contract for future YOLO + metric pose backends.

Implement detect(bgr, K, D) -> board.Observation in an external plugin.
Use aligned depth / known CAD PnP / calibrated plane to recover metric 6D pose.
Pixel boxes alone are insufficient; do not invent depth or orientation.
Both camera plugins must return the same physical target ID and target axes.
"""


class YoloPoseAdapter:
    def detect(self, bgr, K, D):
        raise NotImplementedError('Install a YOLO + metric 6D backend; bounding boxes cannot drive PBVS')
