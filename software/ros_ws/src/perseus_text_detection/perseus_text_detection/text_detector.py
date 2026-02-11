#!/usr/bin/env python3

import cv2
from cv_bridge import CvBridge
import numpy as np
from imutils.object_detection import non_max_suppression
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage

MODEL_PATH = "../model/frozen_east_text_detection.pb"
CONFIDENCE_THRESHOLD = 0.5
RESIZE_SIZE = 10
HEIGHT_OVER_WIDTH_RATIO = 1.0

# Overlay
CELL_COLOR = (0, 255, 0)
CELL_ALPHA = 0.8
CELL_SIZE = 32

BOUNDING_BOX_COLOR = (255, 0, 255)

# Node name
NODE_NAME = 'text_detector'

# Subscription
CAMERA_INPUT_RAW = "/camera/image_raw"
CAMERA_INPUT_COMPRESSED = "/camera/compressed"

# Publish
DETECTION_OUTPUT = "/ocr/detections"
ANNOTATED_IMAGE = "/ocr/annotated_image"

# Service
ON_DEMAND_DETECT = "/ocr/detect_text"

# Image buffer
IMAGE_BUFFER = 0


def paint_cell(image, point):
    overlay = image.copy()
    filled = -1

    cv2.rectangle(
        overlay,
        point,
        (point[0] + CELL_SIZE, point[1] + CELL_SIZE),
        CELL_COLOR,
        filled
    )

    # make transparent
    cv2.addWeighted(overlay, CELL_ALPHA, image, 1 - alpha, 0, image)


def draw_box(image, start_x, start_y, end_x, end_y):
    cv2.rectangle(
        image,
        (start_x, start_y),
        (end_x, end_y),
        BOUNDING_BOX_COLOR,
        2
    )


def east_detect(image):
    original = image.copy()

    (original_height, original_width) = original.shape[:2]

    # multiples of 32 for EAST model
    new_height = 32 * int(RESIZE_SIZE * HEIGHT_OVER_WIDTH_RATIO)
    new_width = 32 * int(RESIZE_SIZE / HEIGHT_OVER_WIDTH_RATIO)

    resize_ratio_height = original_height / float(new_height)
    resize_ratio_width = original_width / float(new_width)

    resize_ratio = (resize_ratio_width, resize_ratio_height)

    image = cv2.resize(image, (new_width, new_height))

    EAST = cv2.dnn.readNet(MODEL_PATH)

    image_blob = cv2.dnn.blobFromImage(
        image,
        1.0,
        (new_width, new_width),
        (123.68, 116.78, 103.94),
        swapRB=True,
        crop=False
    )

    EAST.setInput(image_blob)

    (scores, geometry) = EAST.forward(
        ["feature_fusion/Conv_7/Sigmoid", "feature_fusion/concat_3"])

    (number_of_rows, number_of_columns) = scores.shape[2:4]

    detected_points = []

    rectangles = []
    confidences = []

    for y in range(0, number_of_rows):
        scores_data = scores[0, 0, y]

        rectangle_vertex_1 = geometry[0, 0, y]
        rectangle_vertex_2 = geometry[0, 1, y]
        rectangle_vertex_3 = geometry[0, 2, y]
        rectangle_vertex_4 = geometry[0, 3, y]

        rectangle_angle = geometry[0, 4, y]

        for x in range(0, number_of_columns):
            if scores_data[x] < CONFIDENCE_THRESHOLD:
                continue

            angle = rectangle_angle[x]

            cosine = np.cos(angle)
            sine = np.sin(angle)

            (pixel_x, pixel_y) = map_to_pixel(resize_ratio, (x, y))

            bounding_box_height = rectangle_vertex_1[x] + rectangle_vertex_3[x]
            bounding_box_width = rectangle_vertex_2[x] + rectangle_vertex_4[x]

            end_x = int(
                pixel_x
                + (cosine * rectangle_vertex_2[x])
                + (sine * rectangle_vertex_3[x])
            )
            end_y = int(
                pixel_y
                - (sine * rectangle_vertex_2[x])
                + (cosine * rectangle_vertex_3[x])
            )
            start_x = int(end_x - bounding_box_width)
            start_y = int(end_y - bounding_box_height)

            rectangles.append((start_x, start_y, end_x, end_y))
            confidences.append(scores_data[x])

            detected_points.append((pixel_x, pixel_y))

            # feature maps are 4 times smaller than the input image
            paint_cell(
                original,
                (
                    pixel_x,
                    pixel_y
                ),
            )

    bounding_boxes = non_max_suppression(
        np.array(rectangles), probs=confidences)

    for (start_x, start_y, end_x, end_y) in bounding_boxes:
        draw_box(original, start_x, start_y, end_x, end_y)

    return (original, detected_points, bounding_boxes)


# Maps EAST model output to image pixel
def map_to_pixel(resize_factor, point):
    resize_x = resize_factor[0]
    resize_y = resize_factor[1]

    x = point[0]
    y = point[1]

    # feature maps are 4 times smaller than the input image
    return (int(x * resize_x * 4), int(y * resize_y * 4))


# TODO: Finish implementation
def collect_blobs(points):
    center_of_masses = []

    masses = [0]

    # initialise new list of tuples with default 0 mass id (no mass)
    points_with_masses = []
    for point in points:
        points_with_masses.append(
            (
                point[0],  # x
                point[1],  # y
                point[2],  # score
                0
            )
        )

    x_points = []
    y_points = []

    for point in points:
        x_points.append(point[0])
        y_points.append(point[1])

    for x in range(max(x_points)):
        print(x)

    return center_of_masses


class TextDetectionService(Node):
    def __init__(self):
        super().__init__(NODE_NAME)
        self.bridge = CvBridge()

        self.service = self.create_service(
            String,
            ON_DEMAND_DETECT,
            self.text_detection_service_callback
        )

        # TODO: get image from video feed: cv mat frame?
        self.raw_subscription = self.create_subscription(
            Image,
            CAMERA_INPUT_RAW,
            self.camera_input_raw_subscription,
            10
        )

        self.compressed_subscription = self.create_subscription(
            CompressedImage,
            CAMERA_INPUT_COMPRESSED,
            self.camera_input_compressed_subscription,
            10
        )

        self.text_publisher = self.create_publisher(
            String,
            DETECTION_OUTPUT,
            10
        )

        self.image_publisher = self.create_publisher(
            Image,
            ANNOTATED_IMAGE,
            10
        )

    def camera_input_raw_subscription(self, image):
        try:
            IMAGE_BUFFER = self.bridge.imgmsg_to_cv2(image, "bgr8")
        except Exception as e:
            self.get_logger.error(f"Could not convert image: {e}")

    def camera_input_compressed_subscription(self, image):
        try:
            IMAGE_BUFFER = self.bridge.compressed_imgmsg_to_cv2(image, "bgr8")
        except Exception as e:
            self.get_logger.error(f"Could not convert image: {e}")

    def text_detection_service_callback(self, request, response):
        output = east_detect(IMAGE_BUFFER)

        annotated_image = output[0]

        coordinates = output[1]
        bounding_boxes = output[2]

        # Publish coordinates and bounding boxes as string
        text_message = String()
        text_message.data = f"Coordinates: {str(coordinates)}; Bounding Boxes: {str(bounding_boxes)}"
        text_publisher.publish(text_message)

        # Publish annotated image
        image_publisher.publish(self.bridge.cv2_to_imgmsg(
            annotated_image, "passthrough", "Anntated image")
        )

        # return response.detection = ...
        return response


def main():
    rclpy.init()

    text_detection_service = TextDetectionService()

    rclpy.spin(text_detection_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
