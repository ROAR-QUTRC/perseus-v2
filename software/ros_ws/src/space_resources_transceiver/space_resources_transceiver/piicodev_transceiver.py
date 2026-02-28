"""PiicoDev 915MHz Transceiver (CE08862) I2C driver.

RFM69HCW-based FSK radio with ATtiny1616 I2C bridge.
Thread-safe smbus2 driver following existing PiicoDev patterns.
"""

import logging
import struct
import threading
import time

try:
    from smbus2 import SMBus
except ImportError:
    SMBus = None  # Allow importing for tests without hardware

logger = logging.getLogger(__name__)

# Register addresses (read / write)
_REG_WHOAMI = 0x01
_REG_FIRM_MAJ = 0x02
_REG_FIRM_MIN = 0x03
_REG_STATUS = 0x04
_REG_NODE_ID_R = 0x15
_REG_NODE_ID_W = 0x95
_REG_NETWORK_ID_R = 0x16
_REG_NETWORK_ID_W = 0x96
_REG_DEST_NODE_R = 0x17
_REG_DEST_NODE_W = 0x97
_REG_TX_POWER_R = 0x18
_REG_TX_POWER_W = 0x98
_REG_PAYLOAD_LEN_R = 0x21
_REG_PAYLOAD_LEN_W = 0xA1
_REG_PAYLOAD_R = 0x22
_REG_PAYLOAD_W = 0xA2
_REG_PAYLOAD_NEW = 0x23
_REG_PAYLOAD_GO = 0xA4
_REG_TRANSCEIVER_READY = 0x25
_REG_RSSI = 0x24

_EXPECTED_WHOAMI = 495

# SMBus max data bytes per transaction
_I2C_CHUNK_SIZE = 31


class PiicoDevTransceiver:
    """I2C driver for PiicoDev 915MHz Transceiver."""

    def __init__(self, bus: int = 1, address: int = 0x1A):
        self._address = address
        self._bus_num = bus
        self._bus = None
        self._lock = threading.Lock()
        self._connected = False

    def connect(self) -> bool:
        """Open I2C bus and verify WHOAMI. Returns True on success."""
        if SMBus is None:
            logger.error("smbus2 not installed")
            return False
        try:
            with self._lock:
                self._bus = SMBus(self._bus_num)
                whoami = self._read_u16(_REG_WHOAMI)
                if whoami != _EXPECTED_WHOAMI:
                    logger.error(
                        "WHOAMI mismatch: expected %d, got %d",
                        _EXPECTED_WHOAMI,
                        whoami,
                    )
                    self._bus.close()
                    self._bus = None
                    return False
                self._connected = True
                logger.info(
                    "Transceiver connected on bus %d addr 0x%02X",
                    self._bus_num,
                    self._address,
                )
                return True
        except OSError as e:
            logger.error("I2C connection failed: %s", e)
            self._connected = False
            return False

    def close(self):
        """Close the I2C bus."""
        with self._lock:
            if self._bus is not None:
                self._bus.close()
                self._bus = None
            self._connected = False

    @property
    def is_connected(self) -> bool:
        return self._connected

    def configure(
        self,
        node_id: int = 1,
        network_id: int = 100,
        dest_node_id: int = 2,
        tx_power_dbm: int = 13,
    ):
        """Configure transceiver radio parameters."""
        with self._lock:
            self._write_u8(_REG_NODE_ID_W, node_id)
            time.sleep(0.005)
            self._write_u8(_REG_NETWORK_ID_W, network_id)
            time.sleep(0.005)
            self._write_u8(_REG_DEST_NODE_W, dest_node_id)
            time.sleep(0.005)
            self._write_u8(_REG_TX_POWER_W, tx_power_dbm)
            time.sleep(0.005)
            logger.info(
                "Configured: node=%d network=%d dest=%d power=%ddBm",
                node_id,
                network_id,
                dest_node_id,
                tx_power_dbm,
            )

    def send(self, data: bytes) -> bool:
        """Send a payload via radio. Returns True on success."""
        if len(data) > 61:
            logger.error("Payload too large: %d bytes (max 61)", len(data))
            return False

        with self._lock:
            try:
                # Wait for transceiver to be ready
                if not self._wait_ready(timeout_ms=100):
                    logger.warning("Transceiver not ready for send")
                    return False

                # Write payload length
                self._write_u8(_REG_PAYLOAD_LEN_W, len(data))

                # Write payload in chunks (SMBus limit: 31 data bytes)
                for offset in range(0, len(data), _I2C_CHUNK_SIZE):
                    chunk = data[offset : offset + _I2C_CHUNK_SIZE]
                    self._bus.write_i2c_block_data(
                        self._address, _REG_PAYLOAD_W, list(chunk)
                    )

                # Trigger send
                self._write_u8(_REG_PAYLOAD_GO, 1)
                return True

            except OSError as e:
                logger.error("Send failed: %s", e)
                self._connected = False
                return False

    def receive(self) -> bytes | None:
        """Check for and receive a payload. Returns None if no new data."""
        with self._lock:
            try:
                # Check if new payload available
                new_flag = self._read_u8(_REG_PAYLOAD_NEW)
                if not new_flag:
                    return None

                # Read payload length
                length = self._read_u8(_REG_PAYLOAD_LEN_R)
                if length == 0 or length > 61:
                    return None

                # Read payload in chunks
                payload = []
                remaining = length
                while remaining > 0:
                    chunk_size = min(remaining, _I2C_CHUNK_SIZE)
                    chunk = self._bus.read_i2c_block_data(
                        self._address, _REG_PAYLOAD_R, chunk_size
                    )
                    payload.extend(chunk)
                    remaining -= chunk_size

                return bytes(payload[:length])

            except OSError as e:
                logger.error("Receive failed: %s", e)
                self._connected = False
                return None

    def read_rssi(self) -> int:
        """Read last received signal strength in dBm."""
        with self._lock:
            try:
                raw = self._read_u8(_REG_RSSI)
                # RSSI is returned as unsigned, convert to signed dBm
                return raw if raw < 128 else raw - 256
            except OSError:
                return 0

    # --- Low-level I2C helpers (caller must hold self._lock) ---

    def _read_u8(self, reg: int) -> int:
        return self._bus.read_byte_data(self._address, reg)

    def _read_u16(self, reg: int) -> int:
        data = self._bus.read_i2c_block_data(self._address, reg, 2)
        return struct.unpack(">H", bytes(data))[0]

    def _write_u8(self, reg: int, value: int):
        self._bus.write_byte_data(self._address, reg, value & 0xFF)

    def _wait_ready(self, timeout_ms: int = 100) -> bool:
        deadline = time.monotonic() + timeout_ms / 1000.0
        while time.monotonic() < deadline:
            if self._read_u8(_REG_TRANSCEIVER_READY):
                return True
            time.sleep(0.002)
        return False
