#!/usr/bin/env python3
import struct
import time

import rclpy
from rclpy.node import Node
from control_tower_ros2.msg import DiffWheelCommands

import serial


# clamp the value between -32768 and 32767
def clamp_i16(x: int):
    return max(-32768, min(32767, x))


def crc8_atm(data: bytes) -> int:
    """CRC-8/ATM (poly 0x07, init 0x00)."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


class TwistToUSB(Node):
    # Feedback packet mirrors the forward command packet exactly:
    # SOF(2) + seq + flags + FLvel(2) + FRvel(2) + CRC(1) = 9 bytes.
    FB_PACKET_LEN = 9
    FB_PAYLOAD_FMT = "<BBhh"  # seq, flags, FLvel, FRvel  (bytes 2..7)

    FLAG_FL_STALE = 0x01
    FLAG_FR_STALE = 0x02

    def __init__(self):
        super().__init__("twist_to_usb")

        # parameters
        self.declare_parameter("port", "/dev/igvc_tx_pico")
        self.declare_parameter("baud", 921600)
        self.declare_parameter("topic", "/wheel_commands")
        self.declare_parameter("feedback_topic", "/wheel_feedback")
        self.declare_parameter("send_rate_hz", 50.0)
        self.declare_parameter("timeout_s", 0.2)

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        topic = self.get_parameter("topic").get_parameter_value().string_value
        fb_topic = self.get_parameter("feedback_topic").get_parameter_value().string_value
        send_rate_hz = float(self.get_parameter("send_rate_hz").value)
        self.timeout_s = float(self.get_parameter("timeout_s").value)

        # member variables
        self.seq = 0
        self.lastCommandTime = 0.0
        self.leftWheelVel = 0.0
        self.rightWheelVel = 0.0

        # rolling buffer for the inbound feedback byte stream
        self._rx_buf = bytearray()

        # open serial (timeout=0 -> non-blocking reads)
        self.serial = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0,
            write_timeout=0.05,
        )

        self.get_logger().info(f"Opened Serial {port} @ {baud} baud")

        # forward path: command subscriber + TX timer
        self.sub = self.create_subscription(
            DiffWheelCommands, topic, self.on_wheel_commands, 10
        )
        period = 1.0 / max(1.0, send_rate_hz)
        self.timer = self.create_timer(period, self.send_packet)

        # return path: feedback publisher (reuses DiffWheelCommands) + RX poll
        self.fb_pub = self.create_publisher(DiffWheelCommands, fb_topic, 10)
        self.rx_timer = self.create_timer(0.005, self.read_feedback)  # 200 Hz poll

    # ---------------- forward path ----------------
    def on_wheel_commands(self, msg: DiffWheelCommands):
        self.leftWheelVel = float(msg.v_left)
        self.rightWheelVel = float(msg.v_right)
        self.lastCommandTime = time.monotonic()

    def build_packet(self, leftWheelVel: float, rightWheelVel: float, flags: int = 0):
        # rounding before int() so e.g. 1.2*1000 -> 1200, not 1199
        leftWheelVelI16 = clamp_i16(int(round(leftWheelVel * 1000.0)))   # m/s -> mm/s
        rightWheelVelI16 = clamp_i16(int(round(rightWheelVel * 1000.0)))  # m/s -> mm/s

        sof = b"\xAA\x55"
        seq = self.seq & 0xFF
        flags = flags & 0xFF

        payload = struct.pack("<BBhh", seq, flags, leftWheelVelI16, rightWheelVelI16)
        crc = crc8_atm(payload)
        return sof + payload + struct.pack("<B", crc)

    def send_packet(self):
        now = time.monotonic()
        if (now - self.lastCommandTime) > self.timeout_s:
            leftWheelVel = 0.0
            rightWheelVel = 0.0
            flags = 0x01
        else:
            leftWheelVel = self.leftWheelVel
            rightWheelVel = self.rightWheelVel
            flags = 0x00

        pkt = self.build_packet(leftWheelVel, rightWheelVel, flags)
        try:
            self.serial.write(pkt)
        except serial.SerialTimeoutException:
            self.get_logger().warn("Serial write timeout", throttle_duration_sec=10.0)
        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}", throttle_duration_sec=10.0)

        self.seq = (self.seq + 1) & 0xFF

    # ---------------- return path ----------------
    def read_feedback(self):
        """Drain serial RX, frame-sync on 0xAA 0x55, verify CRC, publish."""
        try:
            chunk = self.serial.read(256)
        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}", throttle_duration_sec=10.0)
            return

        if chunk:
            self._rx_buf.extend(chunk)

        while True:
            sof = self._rx_buf.find(b"\xAA\x55")
            if sof < 0:
                if len(self._rx_buf) > 1:
                    del self._rx_buf[:-1]  # keep trailing byte (maybe split SOF)
                return

            if sof > 0:
                del self._rx_buf[:sof]

            if len(self._rx_buf) < self.FB_PACKET_LEN:
                return

            pkt = bytes(self._rx_buf[: self.FB_PACKET_LEN])
            payload = pkt[2:8]   # bytes 2..7
            rx_crc = pkt[8]
            if crc8_atm(payload) != rx_crc:
                del self._rx_buf[:2]  # drop SOF, resync
                continue

            del self._rx_buf[: self.FB_PACKET_LEN]
            self._publish_feedback(payload)

    def _publish_feedback(self, payload: bytes):
        seq, flags, fl_vel, fr_vel = struct.unpack(self.FB_PAYLOAD_FMT, payload)

        msg = DiffWheelCommands()
        msg.v_left = fl_vel / 1000.0    # mm/s -> m/s
        msg.v_right = fr_vel / 1000.0   # mm/s -> m/s
        self.fb_pub.publish(msg)

        if flags & (self.FLAG_FL_STALE | self.FLAG_FR_STALE):
            self.get_logger().warn(
                f"Stale wheel feedback (flags=0x{flags:02X})",
                throttle_duration_sec=2.0,
            )

    def destroy_node(self):
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
        finally:
            super().destroy_node()


def main():
    rclpy.init()
    node = TwistToUSB()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
