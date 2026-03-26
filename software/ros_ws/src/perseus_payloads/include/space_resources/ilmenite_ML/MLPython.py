#from the servie import something, import get_input_data

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('ilmenite_concentration_service')
        self.srv = self.create_service(get_input_data, 'get_input_data', self.get_input_data_callback)

    def get_input_data_callback(self, request, response):
        response.data = [request.a, request.b, request.c, request.d, request.e, request.f, request.g, request.h,
                         request.i, request.j, request.k, request.l, request.m, request.n, request.o, request.p,
                         request.q, request.r
                        ]
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()