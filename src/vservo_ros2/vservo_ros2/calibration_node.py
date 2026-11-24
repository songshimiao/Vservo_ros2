from collections import deque
import json
from pathlib import Path
import cv2
import numpy as np
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from cv_bridge import CvBridge
from .geometry import read_json, write_json, tf_matrix, error
from .calibration import intrinsic, handeye
from .ros_utils import param, spin, fresh, LATCHED, board_from_params


class CalibrationNode(Node):
    """Manual capture and solve services. Never connects to robot RPC or moves it."""
    def __init__(self):
        super().__init__('calibration')
        self.role = param(self, 'role', 'eih')
        self.mode = param(self, 'mode', 'intrinsic')
        if self.role not in ('eih', 'eth') or self.mode not in ('intrinsic', 'handeye'):
            raise ValueError('Invalid role / calibration mode')
        self.directory = Path(param(self, 'session_dir', '/tmp/vservo_session')).expanduser()
        self.output = Path(param(self, 'output', '/tmp/vservo_result.json')).expanduser()
        self.protect_output = param(self, 'protect_output', False)
        self.directory.mkdir(parents=True, exist_ok=True)
        saved = self.directory / 'samples.json'
        resume = param(self, 'resume', False)
        if saved.exists() and not resume:
            raise ValueError('Session already contains samples; choose a NEW session_dir')
        self.board = board_from_params(self)
        self.max_px = param(self, 'max_rms_px', 1.0)
        self.max_m = param(self, 'max_consistency_m', 0.01)
        self.max_deg = param(self, 'max_consistency_deg', 2.0)
        self.still_s = param(self, 'stillness_s', 0.5)
        self.bridge = CvBridge()
        self.image = self.info = self.meta = None
        self.recent_images = deque(maxlen=12)
        self.samples, self.observations = [], []
        self.last_stamp = None
        self.session_identity = None
        if saved.exists():
            manifest = read_json(saved)
            if (manifest['role'], manifest['mode'], manifest['board']) != (self.role, self.mode, self.board.spec):
                raise ValueError('Saved session role, mode or board configuration mismatch')
            self.samples = manifest['samples']
            self.observations = [(s['object_points'], s['image_points']) for s in self.samples]
            self.last_stamp = max((s['stamp_ns'] for s in self.samples), default=None)
            intrinsics = manifest['intrinsics']
            self.session_identity = (manifest['camera']['serial'], *manifest['image_size'],
                                     tuple(np.ravel(intrinsics['camera_matrix'])), tuple(intrinsics['dist_coeffs']))
        self.tf = Buffer(cache_time=Duration(seconds=30))
        self.listener = TransformListener(self.tf, self)
        self.create_subscription(Image, f'/{self.role}/image_raw', self.on_image, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, f'/{self.role}/camera_info', lambda m: setattr(self, 'info', m), LATCHED)
        self.create_subscription(String, f'/{self.role}/metadata', lambda m: setattr(self, 'meta', json.loads(m.data)), LATCHED)
        self.create_service(Trigger, '~/capture', self.capture)
        self.create_service(Trigger, '~/solve', self.solve)
        self.preview_status = self.create_publisher(String, '~/preview_status', 1)
        self.preview_image = self.create_publisher(CompressedImage, '~/preview_image', qos_profile_sensor_data)
        self.create_timer(0.5, self.preview)

    def on_image(self, msg):
        self.image = msg
        self.recent_images.append(msg)

    def synchronized_image(self):
        if self.mode == 'intrinsic' or self.image is None:
            return self.image
        # TF typically arrives after the corresponding image. Use the newest frame
        # bracketed by TF at both ends of the stillness interval.
        for msg in reversed(self.recent_images):
            if not fresh(self, msg.header.stamp, 0.3):
                continue
            stamp = Time.from_msg(msg.header.stamp)
            if (self.tf.can_transform('base', 'flange', stamp)
                    and self.tf.can_transform('base', 'flange', stamp-Duration(seconds=self.still_s))):
                return msg
        raise ValueError('等待图像时间戳及静止窗口对应的法兰 TF')

    def prepare_sample(self, preview=None):
        msg = self.synchronized_image()
        if msg is None or self.info is None or self.meta is None or not fresh(self, msg.header.stamp, 0.3):
            raise ValueError('Need fresh image, CameraInfo and camera metadata')
        if msg.header.frame_id != f'{self.role}_optical_frame':
            raise ValueError('Unexpected image frame')
        stamp = Time.from_msg(msg.header.stamp)
        if self.last_stamp is not None and stamp.nanoseconds <= self.last_stamp and preview is None:
            raise ValueError('This image was already captured')
        if preview is not None:
            preview['stamp_ns'] = stamp.nanoseconds
        image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if (image.shape[1], image.shape[0]) != (self.info.width, self.info.height):
            raise ValueError('Image and CameraInfo dimensions differ')
        expected_role = 'eye_in_hand' if self.role == 'eih' else 'eye_to_hand'
        if self.meta['camera']['role'] != expected_role:
            raise ValueError('Camera mounting role mismatch')
        identity = (self.meta['camera']['serial'], self.info.width, self.info.height,
                    tuple(self.info.k), tuple(self.info.d))
        if self.session_identity is not None and identity != self.session_identity:
            raise ValueError('Camera identity / intrinsics changed during session; create a new session')
        K = np.array(self.info.k).reshape(3, 3)
        D = np.array(self.info.d)
        obj, pix, ids = self.board.corners(image, K if self.mode == 'handeye' else None,
                                            D if self.mode == 'handeye' else None)
        if preview is not None:
            preview.update(corners=len(pix), pixels=pix.tolist(), detected=True)
        sample = dict(stamp_ns=stamp.nanoseconds, filename=f'{len(self.samples):04d}.png',
                      object_points=obj.tolist(), image_points=pix.tolist(), corner_ids=ids.reshape(-1).tolist())
        if self.mode == 'handeye':
            if not self.meta.get('validated') or self.meta.get('source') != 'charuco_calibrated':
                raise ValueError('Hand-eye requires validated ChArUco intrinsics; restart camera with intrinsics file')
            if preview is not None:
                preview['still'] = None
            current = tf_matrix(self.tf.lookup_transform('base', 'flange', stamp))
            previous = tf_matrix(self.tf.lookup_transform('base', 'flange', stamp-Duration(seconds=self.still_s)))
            translation, angle = error(previous, current)
            still = translation <= 0.001 and angle <= np.deg2rad(0.3)
            if preview is not None:
                preview.update(still=bool(still), flange_mm=translation*1000, flange_deg=float(np.degrees(angle)))
            if not still:
                raise ValueError('Robot was moving: wait >=1s after stopping, then capture')
            observation = self.board.detect(image, K, D)
            sample.update(base_T_flange=current.tolist(), camera_T_board=observation.camera_T_target.tolist(),
                          reprojection_px=observation.reprojection_px)
            if self.samples:
                differences = [error(np.array(s['base_T_flange']), current) for s in self.samples]
                if any(t < 0.005 and a < np.deg2rad(5) for t, a in differences):
                    raise ValueError('Duplicate robot pose: translate / rotate to a new sample')
        if self.last_stamp is not None and stamp.nanoseconds <= self.last_stamp:
            raise ValueError('This image was already captured')
        return image, sample, (obj, pix)

    def preview(self):
        if not self.preview_status.get_subscription_count():
            return
        status = dict(role=self.role, mode=self.mode, session_dir=str(self.directory.resolve()),
                      output=str(self.output.resolve()), protect_output=self.protect_output,
                      board=self.board.spec, sample_count=len(self.samples), detected=False, corners=0, still=None)
        try:
            self.prepare_sample(status)
            status['ready'] = True
        except Exception as exc:
            status.update(ready=False, reason=str(exc))
        msg = next((m for m in reversed(self.recent_images)
                    if Time.from_msg(m.header.stamp).nanoseconds == status.get('stamp_ns')), self.image)
        if msg is not None and fresh(self, msg.header.stamp, 0.3):
            try:
                image = self.bridge.imgmsg_to_cv2(msg, 'bgr8').copy()
                gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
                status['stamp_ns'] = Time.from_msg(msg.header.stamp).nanoseconds
                status['sharpness'] = round(float(cv2.Laplacian(gray, cv2.CV_64F).var()), 1)
                status['bright_fraction'] = round(float(np.mean(gray >= 250)), 3)
                pixels = status.pop('pixels', [])
                if not pixels and not status['detected']:
                    try:
                        K = np.array(self.info.k).reshape(3, 3) if self.mode == 'handeye' and self.info else None
                        D = np.array(self.info.d) if K is not None else None
                        points, _ = self.board.charuco(image, K, D)
                        if points is not None:
                            pixels = points.reshape(-1, 2).tolist()
                            status['corners'] = len(pixels)
                    except (ValueError, cv2.error):
                        pass
                for x, y in pixels:
                    cv2.circle(image, (round(x), round(y)), 4, (30, 235, 255), 2, cv2.LINE_AA)
                if self.preview_image.get_subscription_count():
                    ok, jpeg = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 78])
                    if ok:
                        self.preview_image.publish(CompressedImage(header=msg.header, format='jpeg', data=jpeg.tobytes()))
            except (ValueError, cv2.error, RuntimeError) as exc:
                status['preview_error'] = str(exc)
        self.preview_status.publish(String(data=json.dumps(status, allow_nan=False)))

    def capture(self, request, response):
        try:
            image, sample, observation = self.prepare_sample()
            path = self.directory / sample['filename']
            if not cv2.imwrite(str(path), image):
                raise IOError('Cannot save sample image')
            try:
                write_json(self.directory / 'samples.json', dict(self.manifest(), samples=[*self.samples, sample]))
            except Exception:
                path.unlink(missing_ok=True)
                raise
            self.samples.append(sample)
            self.observations.append(observation)
            self.last_stamp = sample['stamp_ns']
            self.session_identity = (self.meta['camera']['serial'], self.info.width, self.info.height,
                                     tuple(self.info.k), tuple(self.info.d))
            response.success, response.message = True, f'Saved {len(self.samples)} samples in {self.directory}'
        except Exception as exc:
            response.success, response.message = False, str(exc)
        return response

    def manifest(self):
        return dict(schema_version=1, role=self.role, mode=self.mode, board=self.board.spec,
                    camera=self.meta['camera'], intrinsics=self.meta,
                    image_size=[self.info.width, self.info.height], samples=self.samples,
                    timestamp_convention='host_receive_time; stationary capture required')

    def solve(self, request, response):
        try:
            if self.mode == 'intrinsic':
                result = intrinsic(self.observations, [self.info.width, self.info.height], self.max_px)
            else:
                result = handeye([s['base_T_flange'] for s in self.samples],
                                 [s['camera_T_board'] for s in self.samples], self.role, self.max_m, self.max_deg)
            result.update(camera=self.meta['camera'], board=self.board.spec,
                          session_dir=str(self.directory), sample_count=len(self.samples), schema_version=1)
            write_json(self.output, result, replace=not self.protect_output)
            response.success, response.message = True, f'Validated result saved: {self.output}'
        except Exception as exc:
            response.success, response.message = False, str(exc)
        return response


def main(args=None):
    spin(CalibrationNode, args)
