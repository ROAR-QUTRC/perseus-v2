#!/usr/bin/env python3
import os
import json
from typing import Any, Optional
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

MAP_directory = os.environ.get("PERSEUS_MAP_DIR", "/opt/perseus/maps")
YAML_DIR = Path(os.environ.get("PERSEUS_YAML_DIR", str(Path.home() / "perseus-v2/software/web_ui/static"))).resolve()
YAML_DIR.mkdir(parents=True, exist_ok=True)

REQUEST_TOPIC_NAME = "/map_editor/request"
RESPONSE_TOPIC_NAME = "/map_editor/response"


def SaveFileName(name: str) -> str:
    name = os.path.basename(name or "")
    if not name.endswith((".yaml", ".yml")):
        name += ".yaml"
    return name


def HueSaturationValueRanges(
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


def ContourCentroidPixelPosition(contour) -> Optional[tuple[int, int]]:
    moments = cv2.moments(contour)
    if moments["m00"] == 0:
        return None
    centroid_x = int(round(moments["m10"] / moments["m00"]))
    centroid_y = int(round(moments["m01"] / moments["m00"]))
    return centroid_x, centroid_y


def ContourToPoints(contour) -> list[list[int]]:
    return [[int(point[0][0]), int(point[0][1])] for point in contour]


class ExtractFeatures(Node):
    """
    Request/response over std_msgs/String JSON.

    Request:
      {
        "id": "...",
        "op": "extract_feature",
        "mode": "border" | "waypoint" | "origin",
        "image_id": "foo.png",
        "sample_x": 123,
        "sample_y": 456,
        "tol_h": 10,
        "tol_s": 60,
        "tol_v": 60,
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
            String, REQUEST_TOPIC_NAME, self.OnRequest, 10
        )
        self.response_publisher = self.create_publisher(String, RESPONSE_TOPIC_NAME, 10)

        self.get_logger().info(f"ExtractFeatures started. PERSEUS_MAP_DIR={MAP_directory}")
        self.get_logger().info(f"YAML_DIR={YAML_DIR}")

    def LoadImage(self, image_id: str):
        if image_id in self.cache:
            return self.cache[image_id]

        image_path = os.path.join(MAP_directory, image_id)
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

    ### Method that saves the yaml code as a file to a specific folder specified above 
    def SaveYaml(self, request_id: str, request: dict):
        yaml_text = request.get("yaml_text", "")
        file_name = SaveFileName(request.get("file_name", "waypoints.yaml"))

        try:
            YAML_DIR.mkdir(parents=True, exist_ok=True)
            out_path = YAML_DIR / file_name

            tmp_path = out_path.with_suffix(out_path.suffix + ".tmp")
            tmp_path.write_text(yaml_text, encoding="utf-8")
            tmp_path.replace(out_path)

            self.reply(request_id, {"ok": True, "saved_path": str(out_path)})
        except Exception as e:
            self.reply(request_id, {"ok": False, "message": str(e)})

    @staticmethod
    def ChooseContourNearClick(contours, x_pos: int, y_pos: int):
        contours_containing_click: list[tuple[float, Any]] = []
        closest_contour: Optional[tuple[float, Any]] = None

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

    def OnRequest(self, message: String):
        request_id = ""

        try:
            request = json.loads(message.data)
            request_id = str(request.get("id", ""))

            # operation = request.get("op", "")
            # if operation not in ("extract_feature", "extract_border", "extract_waypoint"):
            #     self.reply(request_id, {"ok": False, "message": f"Unsupported op: {operation}"})
            #     return

            # Normalize to one handler with a mode
            mode = request["mode"]
            # if operation == "extract_border":
            #     mode = "border"
            # elif operation == "extract_waypoint":
            #     mode = "waypoint"
            # if mode not in ("border", "waypoint"):
            #     mode = "border"

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

            x_direction = request["x_direction"]
            y_direction = request["y_direction"]

            x_direction = request["x_direction"]
            y_direction = request["y_direction"]

            image_bgr, image_hue_saturation_value, image_height, image_width = (
                self.load_image(image_id)
            )

            if not (0 <= x_pos < image_width and 0 <= y_pos < image_height):
                self.reply(
                    request_id,
                    {"ok": False, "message": f"Out of bounds ({x_pos},{y_pos}) for {image_width}x{image_height}",
                     "sample_x": x_pos, "sample_y": y_pos},
                )
                return

            clicked_hue, clicked_saturation, clicked_value = [
                int(channel) for channel in image_hue_saturation_value[y_pos, x_pos]
            ]
            blue, green, red = [int(channel) for channel in image_bgr[y_pos, x_pos]]
            sample_hexadecimal_color = f"#{red:02X}{green:02X}{blue:02X}"

            mask = None
            for lower_bound, upper_bound in HueSaturationValueRanges(
                clicked_hue, clicked_saturation, clicked_value,
                hue_tolerance, saturation_tolerance, value_tolerance,
            ):
                range_mask = cv2.inRange(
                    image_hue_saturation_value, lower_bound, upper_bound
                )
                mask = range_mask if mask is None else cv2.bitwise_or(mask, range_mask)

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
                    {"ok": False, "message": "No contour found (tolerance too tight?)",
                     "sample_x": x_pos, "sample_y": y_pos,
                     "sample_hex": sample_hexadecimal_color, "sample_image_hex": sample_hexadecimal_color},
                )
                return

            # Filter small blobs (noise)
            contours = [
                contour for contour in contours if cv2.contourArea(contour) >= min_area
            ]
            if not contours:
                self.reply(
                    request_id,
                    {"ok": False, "message": "Contours found but all filtered by min_area",
                     "sample_x": x_pos, "sample_y": y_pos,
                     "sample_hex": sample_hexadecimal_color, "sample_image_hex": sample_hexadecimal_color},
                )
                return

            chosen_contour = self.ChooseContourNearClick(contours, x_pos, y_pos)
            if chosen_contour is None:
                self.reply(
                    request_id,
                    {"ok": False, "message": "No suitable contour near click",
                     "sample_x": x_pos, "sample_y": y_pos,
                     "sample_hex": sample_hexadecimal_color, "sample_image_hex": sample_hexadecimal_color},
                )
                return

            approximated_contour = cv2.approxPolyDP(chosen_contour, approximation_epsilon, True)


            # Always compute centroid (useful for both modes)
            bounding_x_pos, bounding_y_pos, bounding_width, bounding_height = (
                cv2.boundingRect(chosen_contour)
            )
            if mode == "waypoint" or mode == "border":
                centroid = contour_centroid_pixel_position(chosen_contour)
                if centroid is None:
                    centroid = (
                        int(bounding_x_pos + bounding_width / 2),
                        int(bounding_y_pos + bounding_height / 2),
                    )
            elif mode == "origin":
                if x_direction == "right":
                    if y_direction == "down":
                        centroid = (
                            int(bounding_x_pos),
                            int(bounding_y_pos),
                        )
                    elif y_direction == "up":
                        centroid = (
                            int(bounding_x_pos),
                            int(bounding_y_pos + bounding_height),
                        )
                elif x_direction == "left":
                    if y_direction == "down":
                        centroid = (
                            int(bounding_x_pos + bounding_width),
                            int(bounding_y_pos),
                        )
                    elif y_direction == "up":
                        centroid = (
                            int(bounding_x_pos + bounding_width),
                            int(bounding_y_pos + bounding_height),
                        )
                elif x_direction == "down":
                    if y_direction == "right":
                        centroid = (
                            int(bounding_x_pos),
                            int(bounding_y_pos),
                        )
                    elif y_direction == "left":
                        centroid = (
                            int(bounding_x_pos + bounding_width),
                            int(bounding_y_pos),
                        )
                elif x_direction == "up":
                    if y_direction == "right":
                        centroid = (
                            int(bounding_x_pos),
                            int(bounding_y_pos + bounding_height),
                        )
                    elif y_direction == "left":
                        centroid = (
                            int(bounding_x_pos + bounding_width),
                            int(bounding_y_pos + bounding_height),
                        )
                    # if bounding_width > bounding_height:
                    #     centroid = (
                    #         int(bounding_x_pos + bounding_width),
                    #         int(bounding_y_pos + bounding_height / 2),
                    #     )
                    # elif bounding_height > bounding_width:
                    #       centroid = (
                    #         int(bounding_x_pos + bounding_width / 2),
                    #         int(bounding_y_pos + bounding_height),
                    #     )

            payload: dict[str, Any] = {
                "ok": True,
                "message": "OK",
                "mode": mode,
                "sample_x": x_pos,
                "sample_y": y_pos,
                "sample_hex": sample_hexadecimal_color,
                "sample_image_hex": sample_hexadecimal_color,
                "sample_rgb": [red, green, blue],
                "centroid": [centroid[0], centroid[1]],
            }

            payload["contour"] = ContourToPoints(approximated_contour)

            self.reply(request_id, payload)

        except Exception as error:
            self.reply(request_id, {"ok": False, "message": f"ERROR: {error}"})


def main(arguments=None):
    rclpy.init(args=arguments)
    node = ExtractFeatures()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
