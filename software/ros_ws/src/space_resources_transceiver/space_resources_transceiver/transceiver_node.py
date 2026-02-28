"""ROS2 node bridging end effector commands over 915MHz transceiver."""

import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

from perseus_interfaces.msg import EndEffectorCommand, EndEffectorTelemetry

from .piicodev_transceiver import PiicoDevTransceiver
from .protocol import (
    decode_telemetry,
    encode_heartbeat,
    encode_set_all,
    encode_stop_all,
)


class TransceiverNode(Node):
    def __init__(self):
        super().__init__("transceiver_node")

        # Declare parameters
        self.declare_parameter("i2c_bus", 7)
        self.declare_parameter("i2c_address", 0x1A)
        self.declare_parameter("node_id", 1)
        self.declare_parameter("remote_node_id", 2)
        self.declare_parameter("network_id", 100)
        self.declare_parameter("tx_power_dbm", 13)
        self.declare_parameter("command_rate_hz", 10.0)
        self.declare_parameter("telemetry_poll_rate_hz", 20.0)

        # Get parameters
        i2c_bus = self.get_parameter("i2c_bus").value
        i2c_address = self.get_parameter("i2c_address").value
        node_id = self.get_parameter("node_id").value
        remote_node_id = self.get_parameter("remote_node_id").value
        network_id = self.get_parameter("network_id").value
        tx_power_dbm = self.get_parameter("tx_power_dbm").value
        command_rate_hz = self.get_parameter("command_rate_hz").value
        telemetry_poll_rate_hz = self.get_parameter("telemetry_poll_rate_hz").value

        # State
        self._seq = 0
        self._last_cmd_time = 0.0
        self._last_servo_speed = 0.0
        self._last_heater_duty = 0.0
        self._emergency_stop = False
        self._connected = False
        self._last_rssi = 0

        # I2C transceiver driver
        self._transceiver = PiicoDevTransceiver(bus=i2c_bus, address=i2c_address)
        self._node_id = node_id
        self._remote_node_id = remote_node_id
        self._network_id = network_id
        self._tx_power_dbm = tx_power_dbm

        # Publishers
        self._telemetry_pub = self.create_publisher(
            EndEffectorTelemetry, "~/telemetry", 10
        )
        self._connected_pub = self.create_publisher(Bool, "~/connected", 10)

        # Subscriber
        self._command_sub = self.create_subscription(
            EndEffectorCommand, "~/command", self._on_command, 10
        )

        # Timers
        self._cmd_timer = self.create_timer(
            1.0 / command_rate_hz, self._send_command_tick
        )
        self._poll_timer = self.create_timer(
            1.0 / telemetry_poll_rate_hz, self._poll_telemetry_tick
        )
        self._connect_timer = self.create_timer(2.0, self._try_connect)

        self.get_logger().info(
            f"Transceiver node starting (bus={i2c_bus} addr=0x{i2c_address:02X})"
        )
        # Attempt initial connection
        self._try_connect()

    def _try_connect(self):
        """Attempt to connect/reconnect to the transceiver."""
        if self._transceiver.is_connected:
            return

        self.get_logger().info("Attempting transceiver connection...")
        if self._transceiver.connect():
            self._transceiver.configure(
                node_id=self._node_id,
                network_id=self._network_id,
                dest_node_id=self._remote_node_id,
                tx_power_dbm=self._tx_power_dbm,
            )
            self._connected = True
            self.get_logger().info("Transceiver connected and configured")
        else:
            self._connected = False
            self.get_logger().warn("Transceiver connection failed, will retry...")

        # Publish connection status
        msg = Bool()
        msg.data = self._connected
        self._connected_pub.publish(msg)

    def _next_seq(self) -> int:
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFF
        return seq

    def _on_command(self, msg: EndEffectorCommand):
        """Handle incoming end effector command."""
        self._last_cmd_time = time.monotonic()

        if msg.emergency_stop:
            self._emergency_stop = True
            self._last_servo_speed = 0.0
            self._last_heater_duty = 0.0
        else:
            self._emergency_stop = False
            self._last_servo_speed = max(-1.0, min(1.0, msg.servo_speed))
            self._last_heater_duty = max(0.0, min(1.0, msg.heater_duty))

    def _send_command_tick(self):
        """Send command to Pico at 10 Hz."""
        if not self._transceiver.is_connected:
            return

        now = time.monotonic()
        seq = self._next_seq()

        # Watchdog: if no command in 500ms, send STOP_ALL
        if self._emergency_stop or (
            self._last_cmd_time > 0 and (now - self._last_cmd_time) > 0.5
        ):
            frame = encode_stop_all(seq)
        elif self._last_cmd_time == 0.0:
            # No command received yet, send heartbeat
            frame = encode_heartbeat(seq)
        else:
            frame = encode_set_all(seq, self._last_servo_speed, self._last_heater_duty)

        if not self._transceiver.send(frame):
            self._connected = False
            msg = Bool()
            msg.data = False
            self._connected_pub.publish(msg)

    def _poll_telemetry_tick(self):
        """Poll for telemetry from Pico at 20 Hz."""
        if not self._transceiver.is_connected:
            return

        data = self._transceiver.receive()
        if data is None:
            return

        telemetry = decode_telemetry(data)
        if telemetry is None:
            self.get_logger().debug("Invalid telemetry frame received")
            return

        self._last_rssi = self._transceiver.read_rssi()

        msg = EndEffectorTelemetry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.current_amps = telemetry.current_amps
        msg.servo_speed = telemetry.servo_speed
        msg.heater_duty = telemetry.heater_duty
        msg.overcurrent = telemetry.overcurrent
        msg.comm_timeout = telemetry.comm_timeout
        msg.servo_fault = telemetry.servo_fault
        msg.heater_fault = telemetry.heater_fault
        msg.transceiver_fault = telemetry.transceiver_fault
        msg.rssi_dbm = self._last_rssi

        self._telemetry_pub.publish(msg)

    def destroy_node(self):
        self._transceiver.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TransceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
