#!/usr/bin/env python3

import cv2
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import Image

MODEL_PATH = "../model/frozen_east_text_detection.pb"
CONFIDENCE_THRESHOLD = 0.5
RESIZE_SIZE = 10
HEIGHT_OVER_WIDTH_RATIO = 1.0

CAMERA_INPUT = "/camera/camera/color/image_raw"

PREVIOUS_DETECTION_OUTPUT = []


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

    return ((resize_ratio_width, resize_ratio_height), detected_points)


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


class TextDetectionService(Node):
    def __init__(self):
        super().__init__('text_detection_service')
        self.srv = self.create_service(
            DetectedPoints, 'text_detect', self.text_detection_service_callback)

        self.subscriptions = self.create_subscription(
            Image, CAMERA_INPUT, self.listener_callback, 10)

    def listener_callback(self, image):
        PREVIOUS_DETECTION_OUTPUT = map_to_pixels(east_detect(image))

    def text_detection_service_callback(self, request, response):
        return response.detection = PREVIOUS_DETECTION_OUTPUT


def main():
    rclpy.init()

    text_detection_service = TextDetectionService()

    rclpy.spin(text_detection_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
