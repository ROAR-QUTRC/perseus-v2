#!/usr/bin/env python3

from std_srvs.srv import Empty
from std_msgs.msg import Float64
from perseus_interfaces.srv import TakeAs7343Reading

import time
import torch
import json

from perseus_ilmenite_ml.deploy import get_sensor_reading
from perseus_ilmenite_ml.deploy import load_model
from perseus_ilmenite_ml.deploy import load_scaler
from perseus_ilmenite_ml.deploy import predict_concentration
from perseus_ilmenite_ml.deploy import snap_to_nearest
from perseus_ilmenite_ml.deploy import log_prediction

import rclpy
from rclpy.node import Node

#configuration
WEIGHTS_PATH = "ilmenite_model.pth"
LOG_PATH = "predictions_log.csv"
POLL_INTERVAL = 1.0 
SCALER_PATH = "ilmenite_scaler.json"

class IlmeniteController(Node):
    def __init__(self):
        super().__init__("ilmenite_controller")

        self.cache = {}
        self.response_publisher = self.create_publisher(
            Float64, "/ilmenite_concentration/result", 10
        )

        self.srv = self.create_service(
            Empty, "ilmenite/request", self.ilmenite_request_callback
        )
 
        self.cli = self.create_client(TakeAs7343Reading, "/read_ilmenite")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Concentration service for ilmenite not available")
        self.req = TakeAs7343Reading.Request()

        self.get_logger().info("Ilmenite ML Ready")

    def send_request(self):
        return self.cli.call_async(self.req)
    
    def reply(self, payload: float):
        self.response_publisher.publish(payload)

    def ilmenite_request_callback(self, request, response):
        self.get_logger().info("Ilmenite Reading Requested")

        future = self.send_request()
        future.add_done_callback(self.handle_response)

        return response  # Return the response immediately

    def handle_response(self, future):
        try:
            response = future.result()
            self.get_logger().info("Ilmenite Reading Received")
            
            device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
            print(f"Using device: {device}")

            model = load_model(WEIGHTS_PATH, device)
            scaler = load_scaler(SCALER_PATH)

            print(f"Logging predictions to: {LOG_PATH}")

            while True:
                raw_sample = get_sensor_reading()

                if len(raw_sample) != 39:
                    print(f"Warning: expected 39 sensor values, got {len(raw_sample)} — skipping")
                    time.sleep(POLL_INTERVAL)
                    continue

                #run through model
                concentration = predict_concentration(model, raw_sample, scaler, device)
                rounded_concentration = snap_to_nearest(concentration)

                log_prediction(LOG_PATH, raw_sample, concentration)

                print(f"Concentration: {concentration:.4f} | Rounded Concentration: {rounded_concentration:.2%}")
                time.sleep(POLL_INTERVAL)
                
                # Extract concentration and forward to WebUI via topic
                self.reply(concentration)
                self.get_logger().info("Ilmenite concentration=%.4f", concentration)

        except Exception as e:
            self.get_logger().error(f"Failed to get ilmenite reading: {e}")



def main(arguments=None):
    rclpy.init(args=arguments)
    node = IlmeniteController()
    #executor = MultiThreadedExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
