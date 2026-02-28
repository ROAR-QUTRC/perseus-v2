import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

import serial
import struct


VESC_COMMAND_SET_DUTY = 5
DUTY_RAMP_RATE = 0.01

CONTROL_LOOP_PERIOD = 0.05


# see for info on VESC UART protocol and packet structure
# http://vedder.se/2015/10/communicating-with-the-vesc-using-UART/
def crc16(data: bytes):
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


def send_vesc_packet(serial_port, payload: bytes):
    # build and send VESC serial packet

    packet = bytearray()
    packet.append(0x02)
    packet.append(len(payload))
    packet.extend(payload)

    # CRC checksum on the payload
    crc = crc16(payload)
    packet.append((crc >> 8) & 0xFF)
    packet.append(crc & 0xFF)

    packet.append(0x03)

    serial_port.write(packet)


def set_duty_cycle(serial_port, duty):
    if serial_port is None:
        # simulation mode, prob remove later
        return
    duty_int = int(duty * 100000)
    payload = bytearray([VESC_COMMAND_SET_DUTY])
    payload.extend(struct.pack(">i", duty_int))
    send_vesc_packet(serial_port, payload)


class CentrifugeDriver(Node):
    def __init__(self):
        super().__init__("centrifuge_driver")

        # adjust max duty if needed, but 20% fine rn. Also don't overdo ramp speed or it could damage motor
        self.declare_parameter("serial_port", "/dev/serial0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("max_duty", 0.2)

        port = self.get_parameter("serial_port").value
        baud = self.get_parameter("baud_rate").value
        self.max_duty = self.get_parameter("max_duty").value

        # self.serial_port = serial.Serial(port, baud, timeout=0.1)

        # start handling
        try:
            self.serial_port = serial.Serial(port, baud, timeout=0.1)
        except serial.SerialException:
            self.serial_port = None
            self.get_logger().warn(
                f"Couldn't find serial port {port}; running simulation instead"
            )
        # end

        self.current_duty = 0.0
        self.target_duty = 0.0

        self.subscription = self.create_subscription(
            Float32, "/centrifuge/duty_cmd", self.command_callback, 10
        )

        self.timer = self.create_timer(CONTROL_LOOP_PERIOD, self.control_loop)

        self.get_logger().info("Centrifuge driver started.")

    def command_callback(self, msg: Float32):
        requested = max(min(msg.data, self.max_duty), -self.max_duty)
        self.target_duty = requested
        self.get_logger().info(f"Target duty set to {self.target_duty:.3f}")

    def control_loop(self):
        if self.current_duty < self.target_duty:
            self.current_duty = min(
                self.current_duty + DUTY_RAMP_RATE, self.target_duty
            )
        elif self.current_duty > self.target_duty:
            self.current_duty = max(
                self.current_duty - DUTY_RAMP_RATE, self.target_duty
            )

        set_duty_cycle(self.serial_port, self.current_duty)

    def destroy_node(self):
        # stops motor on shutdown
        set_duty_cycle(self.serial_port, 0.0)
        if self.serial_port:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CentrifugeDriver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
