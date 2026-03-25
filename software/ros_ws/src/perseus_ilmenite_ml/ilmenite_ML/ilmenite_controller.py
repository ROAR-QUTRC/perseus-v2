#!/usr/bin/env python3

from std_srvs import Empty
from perseus_interfaces import Concentration

import rclpy
from rclpy.node import Node        

class IlmeniteController(Node):
    def __init__(self):
        super().__init__('ilmenite_controller')

        self.cache = {}
        self.response_publisher = self.create_publisher(float, '/ilmenite_concentration/result', 10)

        self.srv = self.create_service(Empty, 'ilmenite/request', self.ilmenite_request_callback)

        self.cli = self.create_client(Concentration, 'ilmenite/reading')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Concentration service for ilmenite not available")
        self.req = Concentration.Request()

        self.get_logger().info("Ilmenite ML Ready")
    
    def reply(self, payload: float):
        self.response_publisher.publish(payload)

    def ilmenite_request_callback(self, request, response):
        #Sending 'call' to the ML, triggering the process that will send the concentration back to the controller
        ilmenite_reading_client_async = IlmeniteReadingClientAsync()
        future = ilmenite_reading_client_async.send_request()
        rclpy.spin_until_future_complete(ilmenite_reading_client_async, future)
        response = future.result()

        get_sensor_readings(response)

        #Extract concentration and forward to WebUI via topic
        concentration = response.concentration
        self.reply(concentration)
        self.get_logger().info("Ilmenite concentration=%.4f", concentration)        
        
        return
    
    def send_request(self):
        return self.cli.call_async()

def main(arguments=None):
    rclpy.init(args=arguments)
    node = IlmeniteController()
    rclpy.spin(node)
    node.destory_node()
    rclpy.shutdown()
