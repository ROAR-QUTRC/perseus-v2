#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage

MODEL_PATH = "../model/frozen_east_text_detection.pb"
CONFIDENCE_THRESHOLD = 0.5
RESIZE_SIZE = 10
HEIGHT_OVER_WIDTH_RATIO = 1.0

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

# Image holding
IMAGE_GIVEN = 0


def paint_cell(image, point, size, color, alpha):
    overlay = image.copy()
    filled = -1

    cv2.rectangle(
        overlay,
        point,
        (point[0] + size, point[1] + size),
        color,
        filled
    )

    # make transparent
    cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0, image)


def east_detect(image):
    original = image.copy()

    (original_height, original_width) = original.shape[:2]

    # multiples of 32 for EAST model
    new_height = 32 * int(RESIZE_SIZE * HEIGHT_OVER_WIDTH_RATIO)
    new_width = 32 * int(RESIZE_SIZE / HEIGHT_OVER_WIDTH_RATIO)

    resize_ratio_height = original_height / float(new_height)
    resize_ratio_width = original_width / float(new_width)

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

    scores = EAST.forward("feature_fusion/Conv_7/Sigmoid")

    (number_of_rows, number_of_columns) = scores.shape[2:4]

    detected_points = []

    for y in range(0, number_of_rows):
        scores_data = scores[0, 0, y]

        for x in range(0, number_of_columns):
            if scores_data[x] < CONFIDENCE_THRESHOLD:
                continue

            detected_points.append((x, y, scores_data[x].item()))

            paint_cell(
                original,
                (
                    int(x * resize_ratio_width * 4),
                    int(y * resize_ratio_height * 4)
                ),
                32,
                (0, 255, 0),
                0.8
            )

    return ((resize_ratio_width, resize_ratio_height), detected_points, original)


def map_to_pixels(resize_factor, points):
    resize_x = resize_factor[0]
    resize_y = resize_factor[1]

    detected_pixels = []

    for point in points:
        x = point[0]
        y = point[1]
        score = point[2]

        # feature maps are 4 times smaller than the input image
        detected_pixels.append(
            (
                int(x * resize_x * 4),
                int(y * resize_y * 4),
                score
            )
        )

    return detected_pixels

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
        self.srv = self.create_service(
            String, ON_DEMAND_DETECT, self.text_detection_service_callback)

        # TODO: get image from image message
        self.raw_subscription = self.create_subscription(
            Image,
            CAMERA_INPUT_RAW,
            self.camera_input_raw_subscription,
            10
        )

        # TODO: handle compressed images stream
        self.compressed_subscription = self.create_subscription(
            CompressedImage,
            CAMERA_INPUT_COMPRESSED,
            self.camera_input_compressed_subscription,
            10
        )

        self.text_publisher = self.create_publisher(String, DETECTION_OUTPUT, 10)
        self.image_publisher = self.create_publisher(Image, ANNOTATED_IMAGE, 10)

    def camera_input_raw_subscription(self, image):
        IMAGE_GIVEN = image

    # TODO: handle compression
    def camera_input_compressed_subscription(self, image):
        IMAGE_GIVEN = image

    def text_detection_service_callback(self, request, response):
        # return response.detection = ...
        output = east_detect(IMAGE_GIVEN)
        # coordinates = map_to_pixels(output[0], collect_blobs(output[1]))
        coordinates = map_to_pixels(output[0], output[1])
        text_publisher.publish(coordinates) # TODO: proper string message
        image_publisher.publish(output[2]) # TODO: proper image message
        return response


def main():
    rclpy.init()

    text_detection_service = TextDetectionService()

    rclpy.spin(text_detection_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
