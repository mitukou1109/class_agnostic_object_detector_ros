import copy
import io
import threading
import warnings

import cv2
import cv_bridge
import geometry_msgs.msg
import mvits_for_class_agnostic_od.inference.infer as mvits_inference
import mvits_for_class_agnostic_od.models.model as mvits_model
import numpy as np
import numpy.typing as npt
import rclpy
import rclpy.callback_groups
import rclpy.node
import sensor_msgs.msg
import std_msgs.msg
import vision_msgs.msg


class ObjectDetector(rclpy.node.Node):
    class Result:
        def __init__(
            self,
            boxes: list[list[int]],
            scores: list[np.float32],
            image: npt.NDArray[np.uint8],
        ):
            self.boxes = boxes
            self.scores = scores
            self.image = image

    def __init__(self):
        super().__init__("class_agnostic_object_detector")

        checkpoint_file = (
            self.declare_parameter("checkpoint_file", "")
            .get_parameter_value()
            .string_value
        )
        self.text_query = (
            self.declare_parameter("text_query", "all objects")
            .get_parameter_value()
            .string_value
        )
        self.score_threshold = (
            self.declare_parameter("score_threshold", 0.5)
            .get_parameter_value()
            .double_value
        )
        self.use_compressed_image = (
            self.declare_parameter("use_compressed_image", True)
            .get_parameter_value()
            .bool_value
        )
        result_visualization_rate = (
            self.declare_parameter("result_visualization_rate", 10.0)
            .get_parameter_value()
            .double_value
        )

        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            warnings.simplefilter("ignore", FutureWarning, append=True)
            self.model: mvits_inference.Inference = mvits_model.Model(
                "mdef_detr", checkpoint_file
            ).get_model()

        self.cv_bridge = cv_bridge.CvBridge()

        self.result_lock = threading.Lock()
        self.result: ObjectDetector.Result = None

        self.detections_pub = self.create_publisher(
            vision_msgs.msg.Detection2DArray,
            "~/detections",
            1,
        )

        if self.use_compressed_image:
            self.image_raw_sub = self.create_subscription(
                sensor_msgs.msg.CompressedImage,
                "image_raw/compressed",
                self.image_raw_compressed_callback,
                1,
            )
        else:
            self.image_raw_sub = self.create_subscription(
                sensor_msgs.msg.Image,
                "image_raw",
                self.image_raw_callback,
                1,
            )

        self.visualize_result_timer = self.create_timer(
            1.0 / result_visualization_rate,
            self.visualize_result_callback,
            rclpy.callback_groups.MutuallyExclusiveCallbackGroup(),
        )

    def image_raw_compressed_callback(self, msg: sensor_msgs.msg.CompressedImage):
        raw_image: npt.NDArray[np.uint8] = self.cv_bridge.compressed_imgmsg_to_cv2(
            msg, desired_encoding="rgb8"
        )

        self.detect_objects(msg.data, raw_image, msg.header)

    def image_raw_callback(self, msg: sensor_msgs.msg.Image):
        raw_image: npt.NDArray[np.uint8] = self.cv_bridge.imgmsg_to_cv2(
            msg, desired_encoding="rgb8"
        )

        success, jpg_image = cv2.imencode(".jpg", raw_image)
        if not success:
            return

        self.detect_objects(jpg_image, raw_image, msg.header)

    def visualize_result_callback(self):
        if self.result is None:
            return

        with self.result_lock:
            result = copy.deepcopy(self.result)

        for box, score in zip(result.boxes, result.scores):
            self.draw_result_on_image(result.image, box, score)

        resize_ratio = 640 / max(result.image.shape[0], result.image.shape[1])
        result_image = cv2.cvtColor(
            cv2.resize(result.image, dsize=None, fx=resize_ratio, fy=resize_ratio),
            cv2.COLOR_RGB2BGR,
        )

        cv2.imshow("Object detector", result_image)
        cv2.waitKey(1)

    def detect_objects(
        self,
        encoded_image,
        raw_image: npt.NDArray[np.uint8],
        header: std_msgs.msg.Header,
    ):
        result: tuple[list[list[int]], list[np.float32]] = self.model.infer_image(
            io.BytesIO(encoded_image), caption=self.text_query
        )

        detections_msg = vision_msgs.msg.Detection2DArray()
        detections_msg.header = header

        boxes: list[list[int]] = []
        scores: list[np.float32] = []

        for box, score in zip(*result):
            if score < self.score_threshold:
                continue

            boxes.append(box)
            scores.append(score)

            x1, y1, x2, y2 = map(float, box)

            detection = vision_msgs.msg.Detection2D()
            detection.header = header
            detection.bbox.size_x = x2 - x1
            detection.bbox.size_y = y2 - y1
            detection.bbox.center.position.x = (x1 + x2) / 2
            detection.bbox.center.position.y = (y1 + y2) / 2
            detection.results.append(
                vision_msgs.msg.ObjectHypothesisWithPose(
                    hypothesis=vision_msgs.msg.ObjectHypothesis(
                        class_id="object", score=float(score)
                    ),
                    pose=geometry_msgs.msg.PoseWithCovariance(
                        pose=geometry_msgs.msg.Pose(
                            position=geometry_msgs.msg.Point(
                                x=detection.bbox.center.position.x,
                                y=detection.bbox.center.position.y,
                                z=0.0,
                            ),
                        ),
                    ),
                )
            )
            detections_msg.detections.append(detection)

        self.detections_pub.publish(detections_msg)

        with self.result_lock:
            self.result = ObjectDetector.Result(boxes, scores, raw_image)

    def draw_result_on_image(
        self,
        image: npt.NDArray[np.uint8],
        box: list[int],
        score: float,
        *,
        box_thickness: int = 3,
        box_color: tuple[int] = (0, 255, 0),
        font_scale: float = 2.0,
        text_thickness: int = 2,
        text_color: tuple[int] = (255, 255, 255),
    ) -> None:
        x1, y1, x2, y2 = box

        cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), thickness=box_thickness)
        text = f"{score:.2f}"
        text_size, _ = cv2.getTextSize(
            text, cv2.FONT_HERSHEY_SIMPLEX, font_scale, text_thickness
        )
        text_width, text_height = text_size
        text_ys = (
            (y1 - text_height - 5, y1)
            if y1 - text_height - 5 > 0
            else (y1, y1 + text_height)
        )
        cv2.rectangle(
            image,
            (x1, text_ys[0]),
            (x1 + text_width, text_ys[1]),
            box_color,
            thickness=cv2.FILLED,
        )
        cv2.putText(
            image,
            text,
            (x1, text_ys[0] + text_height),
            cv2.FONT_HERSHEY_SIMPLEX,
            font_scale,
            text_color,
            thickness=text_thickness,
            lineType=cv2.LINE_AA,
        )


def main(args=None):
    rclpy.init(args=args)

    object_detector = ObjectDetector()

    rclpy.spin(object_detector)

    object_detector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
