# /root/ros2_ws/src/blue_segmentation/blue_segmentation/blue_filter_node.py
import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Header
from cv_bridge import CvBridge

import cv2
import numpy as np
import time
import os

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def segment_blue(img_bgr, h_min=90, h_max=140, s_min=50, v_min=50, kernel=5, fill_holes=False):
    """
    BGR 이미지에서 파란색 마스크와 세그먼트 이미지를 반환.
    HSV 임계값은 OpenCV 범위(H:0~179, S:0~255, V:0~255)
    """
    hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    # V 채널(밝기)에 대해 히스토그램 평활화를 적용하여 조명 변화에 강인하게 만듭니다.
    h, s, v = cv2.split(hsv)
    v = cv2.equalizeHist(v)
    hsv = cv2.merge([h, s, v])

    lower = np.array([h_min, s_min, v_min], dtype=np.uint8)
    upper = np.array([h_max, 255, 255], dtype=np.uint8)
    mask = cv2.inRange(hsv, lower, upper)

    if kernel and kernel > 1:
        k = np.ones((kernel, kernel), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)

    if fill_holes:
        # 가장 큰 contour를 찾아 내부를 채워서 구멍을 제거
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            cv2.drawContours(mask, [largest_contour], -1, 255, thickness=cv2.FILLED)

    seg = cv2.bitwise_and(img_bgr, img_bgr, mask=mask)
    return mask, seg


def segment_blue_lab(img_bgr, l_max=200, b_max=120, kernel=5, fill_holes=False):
    """
    BGR 이미지를 LAB 색상 공간으로 변환하여 파란색 마스크와 세그먼트 이미지를 반환.
    이 방법은 조명 변화에 더 강인합니다.
    OpenCV LAB 범위 (L: 0-255, a: 0-255, b: 0-255)
    파란색은 낮은 b 채널 값을 가집니다.
    """
    lab = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2Lab)
    l, a, b = cv2.split(lab)

    # L(밝기) 채널에 대해 CLAHE(Contrast Limited Adaptive Histogram Equalization)를 적용
    # 전역 히스토그램 평활화보다 국소적인 대비를 향상시켜 조명 변화에 더 잘 대응
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    l = clahe.apply(l)
    lab = cv2.merge([l, a, b])

    # 파란색을 위한 LAB 임계값
    # L: 너무 어둡거나 너무 밝은 영역 제외
    # a: 녹색-빨강 축, 중간 범위
    # b: 파랑-노랑 축, 파란색은 낮은 값을 가짐 (128이 0점)
    lower = np.array([0, 0, 0], dtype=np.uint8)
    upper = np.array([l_max, 255, b_max], dtype=np.uint8)
    mask = cv2.inRange(lab, lower, upper)

    if kernel and kernel > 1:
        k = np.ones((kernel, kernel), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=1)

    if fill_holes:
        # 가장 큰 contour를 찾아 내부를 채워서 구멍을 제거
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            cv2.drawContours(mask, [largest_contour], -1, 255, thickness=cv2.FILLED)

    seg = cv2.bitwise_and(img_bgr, img_bgr, mask=mask)
    return mask, seg


class BlueFilterNode(Node):
    def __init__(self):
        super().__init__('blue_filter_node')
        self.bridge = CvBridge()

        # 파라미터
        self.declare_parameter('segmentation_method', 'hsv') # 'hsv' 또는 'lab'
        self.declare_parameter('image_topic', '/camera/camera/color/image_raw/compressed') # << 수정됨
        self.declare_parameter('use_compressed', False)   # True면 CompressedImage 구독
        # --- HSV 파라미터
        self.declare_parameter('h_min', 90)
        self.declare_parameter('h_max', 140)
        self.declare_parameter('s_min', 50)
        self.declare_parameter('v_min', 50)
        # --- LAB 파라미터
        self.declare_parameter('lab_l_max', 200) # LAB: 최대 밝기 (너무 밝은 영역 제외)
        self.declare_parameter('lab_b_max', 120) # LAB: b 채널 최대값 (파란색 범위)
        # --- 공용 파라미터
        self.declare_parameter('fill_holes', True)        # 반사광 등으로 인한 구멍 채우기
        self.declare_parameter('kernel_size', 5)
        self.declare_parameter('publish_overlay', True)
        self.declare_parameter('process_every_n', 1)      # N프레임에 1번 처리 (부하 감소용)
        self.declare_parameter('log_fps', True)           # 초당 처리 FPS 로그
        self.declare_parameter('save_segmented_image', True) # 세그먼트 이미지 저장 여부
        self.declare_parameter('save_path', '/tmp/blue_segmented_images') # 저장 경로
        self.declare_parameter('save_format', 'jpg')      # 저장 포맷
        self.declare_parameter('test_image_path', '')     # 테스트 이미지 경로. 지정하면 1회 실행(테스트 모드)
        self.declare_parameter('test_mode_interval_sec', 3.0) # 테스트 모드 처리 간격 (초)

        self.params = {p: self.get_parameter(p).value for p in
            ['segmentation_method',
             'h_min','h_max','s_min','v_min',
             'lab_l_max', 'lab_b_max',
             'fill_holes', 'kernel_size', 'publish_overlay','use_compressed','process_every_n','log_fps',
             'save_segmented_image', 'save_path', 'save_format', 'test_image_path', 'test_mode_interval_sec']}

        # 런타임 파라미터 변경 반영
        self.add_on_set_parameters_callback(self._on_set_params)

        # 퍼블리셔 (Test 모드에서도 사용 가능하도록 앞으로 이동)
        self.pub_mask    = self.create_publisher(Image, 'blue_mask', 10)
        self.pub_seg     = self.create_publisher(Image, 'blue_segmented', 10)
        self.pub_overlay = self.create_publisher(Image, 'blue_overlay', 10)

        # FPS 로깅용
        self._counter = 0
        self._t_last = time.time()

        test_image_path = self.params['test_image_path']
        if test_image_path:
            self._run_test_mode(test_image_path)
        else:
            self._setup_subscriber()

    def _run_test_mode(self, path):
        """단일 이미지 또는 이미지 시퀀스 테스트 모드 실행"""
        self.get_logger().info("--- Running in test mode ---")

        if not os.path.exists(path):
            self.get_logger().error(f"Test path does not exist: {path}")
            return

        # 테스트 시에는 이미지 저장을 강제 활성화
        if not bool(self.params['save_segmented_image']):
            self.get_logger().info("Forcing 'save_segmented_image' to True for test mode.")
            self.params['save_segmented_image'] = True
        self.get_logger().info(f"Output will be saved to: {self.params['save_path']}")

        image_files = []
        if os.path.isdir(path):
            self.get_logger().info(f"Processing image sequence from directory: {path}")
            supported_formats = ['.jpg', '.jpeg', '.png', '.bmp']
            for f in sorted(os.listdir(path)):
                if any(f.lower().endswith(ext) for ext in supported_formats):
                    image_files.append(os.path.join(path, f))
            if not image_files:
                self.get_logger().error(f"No supported image files found in directory: {path}")
                return
        elif os.path.isfile(path):
            self.get_logger().info(f"Processing single image: {path}")
            image_files.append(path)
        
        # Calculate delay based on FPS
        fps = float(self.params.get('test_mode_fps', 10.0))
        if fps <= 0:
            self.get_logger().warn("test_mode_fps must be positive, defaulting to 10.0")
            fps = 10.0
        delay = 1.0 / fps

        for image_path in image_files:
            frame_bgr = cv2.imread(image_path)
            if frame_bgr is None:
                self.get_logger().warn(f"Failed to read image from {image_path}, skipping.")
                continue

            self.get_logger().info(f"Processing: {os.path.basename(image_path)}")
            # 가상 헤더 생성
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "test_image_sequence"

            self._process_and_publish(frame_bgr, header, original_filename=os.path.basename(image_path))
            
            # 다음 이미지 처리를 위한 지연 (스트림 시뮬레이션)
            delay = float(self.params.get('test_mode_interval_sec', 3.0))
            self.get_logger().debug(f"Waiting for {delay:.2f}s ({fps} FPS)")
            time.sleep(delay)

        self.get_logger().info("--- Test mode processing finished ---")

    def _run_single_image_test(self, image_path):
        """단일 이미지 테스트 모드 실행"""
        self.get_logger().info("--- Running in single image test mode ---")
        self.get_logger().info(f"Input image: {image_path}")

        # 테스트 시에는 이미지 저장을 강제 활성화
        if not bool(self.params['save_segmented_image']):
            self.get_logger().info("Forcing 'save_segmented_image' to True for test mode.")
            self.params['save_segmented_image'] = True
        
        self.get_logger().info(f"Output will be saved to: {self.params['save_path']}")

        frame_bgr = cv2.imread(image_path)
        if frame_bgr is None:
            self.get_logger().error(f"Failed to read image from {image_path}")
            return

        # 가상 헤더 생성
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "test_image"

        self._process_and_publish(frame_bgr, header)
        self.get_logger().info("--- Test mode processing finished ---")

    def _setup_subscriber(self):
        """이미지 토픽 구독자 설정"""
        # Sensor-data QoS (RealSense와 궁합)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        image_topic = self.get_parameter('image_topic').value
        if bool(self.params['use_compressed']):
            # 예: /camera/.../image_raw/compressed
            print("use_compressed")
            self.sub = self.create_subscription(CompressedImage, image_topic, self.image_cb_compressed, qos)
        else:
            print("not use_compressed")
            # 예: /camera/.../image_raw
            self.sub = self.create_subscription(Image, image_topic, self.image_cb, qos)

        self.get_logger().info(f"Subscribed: {image_topic} (compressed={bool(self.params['use_compressed'])})")
        self.get_logger().info("Publishing: /blue_mask, /blue_segmented, /blue_overlay")

    # ----------------------
    # 콜백들
    # ----------------------
    def image_cb(self, msg: Image):
        """RAW sensor_msgs/Image 콜백"""
        self.get_logger().info("Image received!") # << 디버깅 로그 추가
        # RealSense는 보통 rgb8/bgr8. cv_bridge가 bgr8로 변환
        frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._process_and_publish(frame_bgr, msg.header)

    def image_cb_compressed(self, msg: CompressedImage):
        """CompressedImage 콜백 (JPEG/PNG 바이트 → BGR 복원)"""
        self.get_logger().info("Compressed Image received!") # << 디버깅 로그 추가
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame_bgr is None:
            self.get_logger().warn("Failed to decode compressed image")
            return
        self._process_and_publish(frame_bgr, msg.header)

    # ----------------------
    # 내부 처리/퍼블리시
    # ----------------------
    def _process_and_publish(self, frame_bgr, header, original_filename=None):
        # 프레임 드롭(부하 조절)
        self._counter += 1
        n = max(1, int(self.params['process_every_n']))
        if (self._counter % n) != 0:
            return

        method = self.params.get('segmentation_method', 'hsv')
        fill_holes = bool(self.params['fill_holes'])

        if method == 'lab':
            mask, seg = segment_blue_lab(
                frame_bgr,
                l_max=int(self.params['lab_l_max']),
                b_max=int(self.params['lab_b_max']),
                kernel=int(self.params['kernel_size']),
                fill_holes=fill_holes
            )
        else: # 기본값 'hsv'
            mask, seg = segment_blue(
                frame_bgr,
                h_min=int(self.params['h_min']),
                h_max=int(self.params['h_max']),
                s_min=int(self.params['s_min']),
                v_min=int(self.params['v_min']),
                kernel=int(self.params['kernel_size']),
                fill_holes=fill_holes
            )

        if bool(self.params['publish_overlay']):
            overlay = frame_bgr.copy()
            overlay[mask > 0] = (255, 0, 0)  # 파란색 강조(BGR)
            vis = cv2.addWeighted(frame_bgr, 1.0, overlay, 0.4, 0.0)
        else:
            vis = seg

        # 퍼블리시 (Compressed → Image로 내보냄)
        mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
        seg_msg  = self.bridge.cv2_to_imgmsg(seg,  encoding='bgr8')
        vis_msg  = self.bridge.cv2_to_imgmsg(vis,  encoding='bgr8')
        mask_msg.header = seg_msg.header = vis_msg.header = header

        self.pub_mask.publish(mask_msg)
        self.pub_seg.publish(seg_msg)
        self.pub_overlay.publish(vis_msg)

        # 이미지 저장
        if bool(self.params['save_segmented_image']):
            self._save_image(seg, header, original_filename=original_filename)

        # FPS 로그
        if bool(self.params['log_fps']):
            now = time.time()
            dt = now - self._t_last
            if dt >= 1.0:
                fps = self._counter / dt
                self.get_logger().info(f"Proc FPS ~ {fps:.1f} (every_n={n})")
                self._counter = 0
                self._t_last = now

    ###sehun
    def _save_image(self, img, header, original_filename=None):
        """지정된 경로에 이미지를 저장"""
        save_path = self.params['save_path']
        if not os.path.exists(save_path):
            try:
                os.makedirs(save_path)
                self.get_logger().info(f"Created directory: {save_path}")
            except OSError as e:
                self.get_logger().error(f"Failed to create save directory {save_path}: {e}")
                return

        if original_filename:
            # 테스트 모드에서 원본 파일명 사용
            filename = os.path.join(save_path, original_filename)
        else:
            # 실시간 모드에서 타임스탬프 사용
            img_format = self.params['save_format'].lower()
            ts = f"{header.stamp.sec}_{header.stamp.nanosec // 1000000:03d}"
            filename = os.path.join(save_path, f"segmented_{ts}.{img_format}")
        
        try:
            cv2.imwrite(filename, img)
            # 너무 자주 로그가 찍히는 것을 방지하기 위해 debug 레벨 사용
            self.get_logger().debug(f"Saved image to {filename}")
        except Exception as e:
            self.get_logger().error(f"Failed to save image to {filename}: {e}")

    # ----------------------
    # 파라미터 콜백
    # ----------------------
    def _on_set_params(self, params):
        for p in params:
            if p.name in self.params:
                self.params[p.name] = p.value
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    node = BlueFilterNode()

    # 테스트 모드인 경우, 처리는 __init__에서 이미 끝났으므로 spin하지 않음
    if not node.get_parameter('test_image_path').value:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
    else:
        # 테스트 모드에서는 발행 후 잠시 대기하여 메시지 전송을 보장
        node.get_logger().info("Test mode: Published topics. Waiting 1s for delivery before shutdown...")
        time.sleep(1.0)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
