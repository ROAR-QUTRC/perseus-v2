#!/usr/bin/env python3
import os
import json
from typing import Any, Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

MAP_DIR = os.environ.get("PERSEUS_MAP_DIR", "/opt/perseus/maps")

REQUEST_TOPIC_NAME = "/map_editor/request"
RESPONSE_TOPIC_NAME = "/map_editor/response"


def hue_saturation_value_ranges(
    hue: int,
    saturation: int,
    value: int,
    hue_tolerance: int,
    saturation_tolerance: int,
    value_tolerance: int,
):
    """Return one or two HSV ranges (OpenCV hue wraparound handled)."""
    hue_min, hue_max = int(hue - hue_tolerance), int(hue + hue_tolerance)
    saturation_min, saturation_max = (
        max(0, int(saturation - saturation_tolerance)),
        min(255, int(saturation + saturation_tolerance)),
    )
    value_min, value_max = (
        max(0, int(value - value_tolerance)),
        min(255, int(value + value_tolerance)),
    )

    # Hue wraps in OpenCV: 0..179
    if hue_min < 0:
        return [
            (
                np.array([0, saturation_min, value_min]),
                np.array([hue_max, saturation_max, value_max]),
            ),
            (
                np.array([179 + hue_min, saturation_min, value_min]),
                np.array([179, saturation_max, value_max]),
            ),
        ]
    if hue_max > 179:
        return [
            (
                np.array([0, saturation_min, value_min]),
                np.array([hue_max - 179, saturation_max, value_max]),
            ),
            (
                np.array([hue_min, saturation_min, value_min]),
                np.array([179, saturation_max, value_max]),
            ),
        ]

    return [
        (
            np.array([hue_min, saturation_min, value_min]),
            np.array([hue_max, saturation_max, value_max]),
        )
    ]


def contour_centroid_pixel_position(contour) -> Optional[tuple[int, int]]:
    """Return integer centroid (x, y) in pixel coordinates, or None if degenerate."""
    moments = cv2.moments(contour)
    if moments["m00"] == 0:
        return None

    centroid_x = int(round(moments["m10"] / moments["m00"]))
    centroid_y = int(round(moments["m01"] / moments["m00"]))
    return centroid_x, centroid_y


def contour_to_points(contour) -> list[list[int]]:
    """Convert OpenCV contour array to [[x, y], ...]."""
    return [[int(point[0][0]), int(point[0][1])] for point in contour]


class ExtractFeatures(Node):
    """
    Request/response over std_msgs/String JSON.

    Request:
      {
        "id": "...",
        "op": "extract_feature",
        "mode": "border" | "waypoint",
        "image_id": "foo.png",
        "sample_x": 123,
        "sample_y": 456,
        "tol_h": 10,
        "tol_s": 60,
        "tol_v": 60,
        "min_area": 50,          (optional)
        "approx_eps": 2.0,       (optional)
        "close_k": 5,            (optional)
        "median_k": 5            (optional)
      }

    Response:
      {
        "id": "...",
        "ok": true/false,
        "message": "...",
        "sample_x": ...,
        "sample_y": ...,
        "sample_hex": "#RRGGBB",
        "sample_image_hex": "#RRGGBB",   (compatibility field)
        "centroid": [x, y],
        "contour": [[x, y], ...]
      }
    """

    def __init__(self):
        super().__init__("extract_features")

        self.cache: dict[str, tuple[np.ndarray, np.ndarray, int, int]] = {}

        self.request_subscription = self.create_subscription(
            String, REQUEST_TOPIC_NAME, self.on_request, 10
        )
        self.response_publisher = self.create_publisher(String, RESPONSE_TOPIC_NAME, 10)

        self.get_logger().info(f"ExtractFeatures started. PERSEUS_MAP_DIR={MAP_DIR}")

    def load_image(self, image_id: str):
        if image_id in self.cache:
            return self.cache[image_id]

        image_path = os.path.join(MAP_DIR, image_id)
        image_bgr = cv2.imread(image_path, cv2.IMREAD_COLOR)
        if image_bgr is None:
            raise FileNotFoundError(f"Cannot read image: {image_path}")

        image_hue_saturation_value = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
        image_height, image_width = image_bgr.shape[:2]

        self.cache[image_id] = (
            image_bgr,
            image_hue_saturation_value,
            image_height,
            image_width,
        )
        return image_bgr, image_hue_saturation_value, image_height, image_width

    def reply(self, request_id: str, payload: dict[str, Any]):
        payload["id"] = request_id
        self.response_publisher.publish(String(data=json.dumps(payload)))

    @staticmethod
    def choose_contour_near_click(contours, x_pos: int, y_pos: int):
        """
        Prefer a contour that CONTAINS the click (pointPolygonTest >= 0).
        If none contain it, pick the closest contour (minimum abs(distance)).
        """
        contours_containing_click: list[tuple[float, Any]] = []
        closest_contour: Optional[tuple[float, Any]] = None  # (abs_distance, contour)

        for contour in contours:
            signed_distance = cv2.pointPolygonTest(
                contour, (float(x_pos), float(y_pos)), True
            )
            if signed_distance >= 0:
                contours_containing_click.append(
                    (signed_distance, contour)
                )  # prefer most-inside (max distance)
            else:
                absolute_distance = abs(signed_distance)
                if closest_contour is None or absolute_distance < closest_contour[0]:
                    closest_contour = (absolute_distance, contour)

        if contours_containing_click:
            contours_containing_click.sort(key=lambda item: item[0], reverse=True)
            return contours_containing_click[0][1]

        if closest_contour is not None:
            return closest_contour[1]

        return None

    def on_request(self, message: String):
        request_id = ""

        try:
            request = json.loads(message.data)
            request_id = str(request.get("id", ""))

            operation = request.get("op", "")
            if operation not in (
                "extract_feature",
                "extract_border",
                "extract_waypoint",
            ):
                self.reply(
                    request_id, {"ok": False, "message": f"Unsupported op: {operation}"}
                )
                return

            # Normalize to one handler with a mode
            mode = request.get("mode")
            if operation == "extract_border":
                mode = "border"
            elif operation == "extract_waypoint":
                mode = "waypoint"
            if mode not in ("border", "waypoint"):
                mode = "border"

            image_id = request["image_id"]
            x_pos = int(request["sample_x"])
            y_pos = int(request["sample_y"])

            hue_tolerance = int(request.get("tol_h", 10))
            saturation_tolerance = int(request.get("tol_s", 60))
            value_tolerance = int(request.get("tol_v", 60))

            min_area = float(request.get("min_area", 50))
            approximation_epsilon = float(request.get("approx_eps", 2.0))

            close_kernel_size = int(request.get("close_k", 5))
            median_kernel_size = int(request.get("median_k", 5))

            image_bgr, image_hue_saturation_value, image_height, image_width = (
                self.load_image(image_id)
            )

            if not (0 <= x_pos < image_width and 0 <= y_pos < image_height):
                self.reply(
                    request_id,
                    {
                        "ok": False,
                        "message": f"Out of bounds ({x_pos},{y_pos}) for {image_width}x{image_height}",
                        "sample_x": x_pos,
                        "sample_y": y_pos,
                    },
                )
                return

            # Sample HSV and BGR at click point
            clicked_hue, clicked_saturation, clicked_value = [
                int(channel) for channel in image_hue_saturation_value[y_pos, x_pos]
            ]
            blue, green, red = [int(channel) for channel in image_bgr[y_pos, x_pos]]
            sample_hexadecimal_color = f"#{red:02X}{green:02X}{blue:02X}"

            # Build mask for the clicked color class
            mask = None
            for lower_bound, upper_bound in hue_saturation_value_ranges(
                clicked_hue,
                clicked_saturation,
                clicked_value,
                hue_tolerance,
                saturation_tolerance,
                value_tolerance,
            ):
                range_mask = cv2.inRange(
                    image_hue_saturation_value, lower_bound, upper_bound
                )
                mask = range_mask if mask is None else cv2.bitwise_or(mask, range_mask)

            # Clean mask
            if median_kernel_size >= 3:
                if median_kernel_size % 2 == 0:
                    median_kernel_size += 1
                mask = cv2.medianBlur(mask, median_kernel_size)

            if close_kernel_size >= 3:
                kernel = np.ones((close_kernel_size, close_kernel_size), np.uint8)
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(
                mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )
            if not contours:
                self.reply(
                    request_id,
                    {
                        "ok": False,
                        "message": "No contour found (tolerance too tight?)",
                        "sample_x": x_pos,
                        "sample_y": y_pos,
                        "sample_hex": sample_hexadecimal_color,
                        "sample_image_hex": sample_hexadecimal_color,
                    },
                )
                return

            # Filter small blobs (noise)
            contours = [
                contour for contour in contours if cv2.contourArea(contour) >= min_area
            ]
            if not contours:
                self.reply(
                    request_id,
                    {
                        "ok": False,
                        "message": "Contours found but all filtered by min_area",
                        "sample_x": x_pos,
                        "sample_y": y_pos,
                        "sample_hex": sample_hexadecimal_color,
                        "sample_image_hex": sample_hexadecimal_color,
                    },
                )
                return

            chosen_contour = self.choose_contour_near_click(contours, x_pos, y_pos)
            if chosen_contour is None:
                self.reply(
                    request_id,
                    {
                        "ok": False,
                        "message": "No suitable contour near click",
                        "sample_x": x_pos,
                        "sample_y": y_pos,
                        "sample_hex": sample_hexadecimal_color,
                        "sample_image_hex": sample_hexadecimal_color,
                    },
                )
                return

            # Simplify for payload size
            approximated_contour = cv2.approxPolyDP(
                chosen_contour, epsilon=approximation_epsilon, closed=True
            )

            # Always compute centroid (useful for both modes)
            centroid = contour_centroid_pixel_position(chosen_contour)
            if centroid is None:
                bounding_x_pos, bounding_y_pos, bounding_width, bounding_height = (
                    cv2.boundingRect(chosen_contour)
                )
                centroid = (
                    int(bounding_x_pos + bounding_width / 2),
                    int(bounding_y_pos + bounding_height / 2),
                )

            payload: dict[str, Any] = {
                "ok": True,
                "message": "OK",
                "mode": mode,
                "sample_x": x_pos,
                "sample_y": y_pos,
                "sample_hex": sample_hexadecimal_color,
                "sample_image_hex": sample_hexadecimal_color,  # compatibility for frontend variants
                "sample_rgb": [red, green, blue],
                "centroid": [centroid[0], centroid[1]],
            }

            if mode == "border":
                payload["contour"] = contour_to_points(approximated_contour)

            self.reply(request_id, payload)

        except Exception as error:
            self.reply(request_id, {"ok": False, "message": f"ERROR: {error}"})


def main(arguments=None):
    rclpy.init(args=arguments)
    node = ExtractFeatures()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
