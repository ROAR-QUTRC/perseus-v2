#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from perseus_interfaces.srv import TakeAs7343Reading

class MockSensor(Node):
    def __init__(self):
        super().__init__('mock_sensor')
        self.srv = self.create_service(
            TakeAs7343Reading,
            '/read_ilmenite',
            self.callback
        )

    def callback(self, request, response):
        def fill(data, base):
            data.f1_405nm = base + 1
            data.f2_425nm = base + 2
            data.fz_450nm = base + 3
            data.f3_475nm = base + 4
            data.f4_515nm = base + 5
            data.f5_550nm = base + 6
            data.fy_555nm = base + 7
            data.fxl_600nm = base + 8
            data.f6_640nm = base + 9
            data.f7_690nm = base + 10
            data.f8_745nm = base + 11
            data.nir_855nm = base + 12
            data.vis_clear = base + 13
            data.fd_flicker = 5
            data.redo_reading = False
            data.success = True

        fill(response.no_led_reading, 100)
        fill(response.white_led_reading, 200)
        fill(response.uv_led_reading, 300)

        response.success = True
        response.message = "Mock data OK"

        self.get_logger().info("Mock sensor called")
        return response


def main():
    rclpy.init()
    node = MockSensor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
