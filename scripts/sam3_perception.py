#!/usr/bin/env python3
"""Text-prompt SAM 3 segmentation of the shared ROS camera stream."""
import copy
from dataclasses import dataclass
import json
from pathlib import Path
import threading
import time

import numpy as np
from PIL import Image as PILImage, ImageDraw
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


@dataclass
class Prediction:
    masks: np.ndarray  # N,H,W, boolean, original image resolution
    boxes: np.ndarray  # N,4, xyxy in original image pixels
    scores: np.ndarray  # N


class Sam3Backend:
    def __init__(self, checkpoint, device, threshold):
        import torch
        from sam3.model_builder import build_sam3_image_model
        from sam3.model.sam3_image_processor import Sam3Processor

        self.torch = torch
        self.device = torch.device(device)
        if self.device.type not in ('cpu', 'cuda'):
            raise ValueError('device must be cpu or cuda[:index]')
        if self.device.type == 'cuda':
            if not torch.cuda.is_available():
                raise RuntimeError('CUDA unavailable; configure a supported GPU or device:=cpu')
            if self.device.index is None:
                self.device = torch.device('cuda', torch.cuda.current_device())
            torch.cuda.set_device(self.device)
        if checkpoint:
            checkpoint = str(Path(checkpoint).expanduser().resolve(strict=True))
        else:
            # Resolve weights before allocating the model, failing promptly when
            # access is missing. An explicit path supports fully offline operation.
            from huggingface_hub import hf_hub_download
            checkpoint = hf_hub_download('facebook/sam3', 'sam3.pt')
        model = build_sam3_image_model(
            checkpoint_path=checkpoint, load_from_HF=False, device='cpu', eval_mode=True,
        ).to(self.device)
        self.processor = Sam3Processor(model, device=str(self.device), confidence_threshold=threshold)
        self.dtype = (torch.bfloat16 if self.device.type == 'cuda' and
                      torch.cuda.is_bf16_supported() else torch.float16)

    def predict(self, rgb, prompt):
        torch = self.torch
        with torch.inference_mode(), torch.autocast(
                device_type=self.device.type, dtype=self.dtype, enabled=self.device.type == 'cuda'):
            state = self.processor.set_image(PILImage.fromarray(rgb))
            output = self.processor.set_text_prompt(state=state, prompt=prompt)
            return Prediction(
                output['masks'].squeeze(1).detach().cpu().numpy(),
                output['boxes'].detach().float().cpu().numpy(),
                output['scores'].detach().float().cpu().numpy(),
            )


def compose_outputs(rgb, prediction, prompt, threshold, max_instances):
    """Resolve overlaps by confidence; return labels, overlay, and detections."""
    height, width = rgb.shape[:2]
    masks, boxes, scores = prediction.masks, prediction.boxes, prediction.scores
    if masks.shape != (len(scores), height, width) or boxes.shape != (len(scores), 4):
        raise ValueError('SAM 3 result shapes do not match the source image')
    labels = np.zeros((height, width), dtype=np.uint16)
    overlay = rgb.copy()
    records = []
    for index in np.argsort(-scores, kind='stable'):
        score = float(scores[index])
        if not np.isfinite(score) or not threshold <= score <= 1:
            continue
        if not np.isfinite(boxes[index]).all():
            continue
        mask = masks[index].astype(bool)
        if not mask.any():
            continue
        x0, y0, x1, y1 = map(float, boxes[index])
        x0, x1 = np.clip([x0, x1], 0, width)
        y0, y1 = np.clip([y0, y1], 0, height)
        if x1 <= x0 or y1 <= y0:
            continue
        instance_id = len(records) + 1
        visible = mask & (labels == 0)
        labels[visible] = instance_id
        color = np.array([(instance_id * 67) % 192 + 64,
                          (instance_id * 113) % 192 + 64,
                          (instance_id * 157) % 192 + 64], dtype=np.uint8)
        overlay[visible] = (rgb[visible].astype(np.float32) * 0.5 + color * 0.5).astype(np.uint8)
        records.append((instance_id, score, (float(x0), float(y0), float(x1), float(y1)), color))
        if len(records) == max_instances:
            break
    image = PILImage.fromarray(overlay)
    draw = ImageDraw.Draw(image)
    for instance_id, score, box, color in records:
        draw.rectangle(box, outline=tuple(map(int, color)), width=2)
        draw.text((box[0], max(0, box[1] - 12)), f'{instance_id}: {prompt} {score:.2f}',
                  fill=(255, 255, 255))
    return labels, np.asarray(image), records


class Sam3Perception(Node):
    def __init__(self, backend_factory=Sam3Backend, **kwargs):
        super().__init__('sam3_perception', **kwargs)
        defaults = dict(image_topic='/camera/color/image_raw', prompt='chair', checkpoint='',
                        device='cuda', confidence_threshold=0.5, max_rate_hz=2.0, max_instances=100)
        params = {}
        for name, default in defaults.items():
            params[name] = self.declare_parameter(
                name, default, ParameterDescriptor(read_only=True)).value
        if not 0 <= params['confidence_threshold'] <= 1:
            raise ValueError('confidence_threshold must be in [0,1]')
        if not np.isfinite(params['max_rate_hz']) or params['max_rate_hz'] <= 0:
            raise ValueError('max_rate_hz must be finite and positive')
        if not 1 <= params['max_instances'] <= 65535:
            raise ValueError('max_instances must be in [1,65535]')
        self.params = params
        self.bridge = CvBridge()
        self.condition = threading.Condition()
        self.pending = None
        self.prompt = params['prompt'].strip()
        self.generation = 0
        self.stopping = False
        self.fatal_error = None
        image_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.overlay_pub = self.create_publisher(Image, '/sam3/overlay', image_qos)
        self.labels_pub = self.create_publisher(Image, '/sam3/instance_labels', image_qos)
        self.detections_pub = self.create_publisher(Detection2DArray, '/sam3/detections', image_qos)
        self.status_pub = self.create_publisher(String, '/sam3/status', QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Image, params['image_topic'], self.on_image, image_qos)
        self.create_subscription(String, '/sam3/prompt', self.on_prompt, 1)
        self.worker = threading.Thread(target=self.work, args=(backend_factory,), daemon=True)
        self.worker.start()

    def status(self, state, **details):
        if rclpy.ok(context=self.context):
            self.status_pub.publish(String(data=json.dumps(dict(state=state, **details))))

    def on_image(self, msg):
        # Replace the pending frame; conversion and GPU work never run in callbacks.
        with self.condition:
            self.pending = msg
            self.condition.notify()

    def on_prompt(self, msg):
        with self.condition:
            self.prompt = msg.data.strip()
            self.generation += 1
            self.condition.notify()
        self.get_logger().info(f'SAM 3 prompt: {self.prompt!r}')

    def work(self, backend_factory):
        try:
            self.status('loading')
            backend = backend_factory(self.params['checkpoint'], self.params['device'],
                                      self.params['confidence_threshold'])
            self.get_logger().info('SAM 3 ready; waiting for RGB images')
            self.status('ready')
        except Exception as error:
            self.fatal_error = f'SAM 3 startup failed: {error}'
            self.get_logger().error(self.fatal_error)
            self.status('error', message=self.fatal_error)
            return
        next_frame = 0.0
        failures = 0
        while True:
            with self.condition:
                while not self.stopping:
                    delay = next_frame - time.monotonic()
                    if self.pending is not None and delay <= 0:
                        break
                    self.condition.wait(timeout=max(0.001, delay) if self.pending is not None else None)
                if self.stopping:
                    return
                msg, self.pending = self.pending, None
                prompt, generation = self.prompt, self.generation
            started = time.monotonic()
            next_frame = started + 1.0 / self.params['max_rate_hz']
            try:
                rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
                if prompt:
                    prediction = backend.predict(rgb, prompt)
                else:
                    prediction = Prediction(np.empty((0, *rgb.shape[:2]), dtype=bool),
                                            np.empty((0, 4)), np.empty(0))
                labels, overlay, records = compose_outputs(
                    rgb, prediction, prompt, self.params['confidence_threshold'],
                    self.params['max_instances'])
                overlay_msg = self.bridge.cv2_to_imgmsg(overlay, encoding='rgb8')
                labels_msg = self.bridge.cv2_to_imgmsg(labels, encoding='16UC1')
                detections = Detection2DArray()
                for output in (overlay_msg, labels_msg, detections):
                    output.header = copy.deepcopy(msg.header)
                for instance_id, score, (x0, y0, x1, y1), _ in records:
                    detection = Detection2D()
                    detection.header = copy.deepcopy(msg.header)
                    detection.id = str(instance_id)  # Per-frame label, NOT a tracking ID.
                    detection.bbox.center.position.x = (x0 + x1) / 2
                    detection.bbox.center.position.y = (y0 + y1) / 2
                    detection.bbox.size_x = x1 - x0
                    detection.bbox.size_y = y1 - y0
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = prompt
                    hypothesis.hypothesis.score = score
                    detection.results = [hypothesis]
                    detections.detections.append(detection)
                with self.condition:
                    # A changed prompt invalidates in-flight results before publication.
                    if self.stopping or not rclpy.ok(context=self.context):
                        return
                    if generation != self.generation:
                        continue
                    self.overlay_pub.publish(overlay_msg)
                    self.labels_pub.publish(labels_msg)
                    self.detections_pub.publish(detections)
                    self.status('running' if prompt else 'idle', prompt=prompt,
                                instances=len(records), inference_ms=(time.monotonic()-started)*1000,
                                frame_id=msg.header.frame_id,
                                stamp={'sec': msg.header.stamp.sec, 'nanosec': msg.header.stamp.nanosec})
                failures = 0
            except Exception as error:
                failures += 1
                message = f'SAM 3 frame processing failed: {error}'
                self.get_logger().error(message)
                self.status('error', message=message)
                if failures >= 5:
                    self.fatal_error = message
                    return

    def stop(self):
        with self.condition:
            self.stopping = True
            self.condition.notify_all()
        # Keep ROS resources alive until the in-flight GPU call returns.
        self.worker.join()


def main():
    rclpy.init()
    node = None
    try:
        node = Sam3Perception()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.fatal_error:
                raise RuntimeError(node.fatal_error)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
