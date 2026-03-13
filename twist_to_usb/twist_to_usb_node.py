#!/usr/bin/env python3
import struct
import time

import rclpy
from rclpy.node import Node
from control_tower_ros2.msg import WheelCommands

import serial

# clamp the value between -32768 and 32767
def clamp_i16(x:int):
    return max(-32768, min(32767,x))

def crc8_atm(data: bytes, poly: int = 0x07, init: int = 0x00) -> int:
    """
    CRC-8/ATM (poly 0x07), init 0x00, no xorout, no reflection.
    Good simple CRC for short packets.
    """
    crc = init
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ poly) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc

class TwistToUSB(Node):
    def __init__(self):
        super().__init__("twist_to_usb")
    
        # parameters list: port, baud, input topic, send_rate_hz, timeout_s
        self.declare_parameter("port","/dev/igvc_tx_pico")
        self.declare_parameter("baud", 921600)
        self.declare_parameter("topic", "/wheel_commands")
        self.declare_parameter("send_rate_hz", 50.0)
        self.declare_parameter("timeout_s", 0.2) ### adjust for hz rate
                
        # getting values
        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        topic = self.get_parameter("topic").get_parameter_value().string_value
        send_rate_hz = float(self.get_parameter("send_rate_hz").value)
        self.timeout_s = float(self.get_parameter("timeout_s").value)
                
        # member varibles
        self.seq = 0
        self.last_command_time = 0.0
        
        self.last_left_speed = 0.0
        self.last_right_speed = 0.0
        self.last_left_steer = 0.0
        self.last_right_steer = 0.0
        
        # open serial
        self.serial = serial.Serial(
            port=port,
            baudrate=baud,
            timeout=0,
            write_timeout=0.05
        )
        
        # debugging 
        self.get_logger().info(f"Opened Serial {port} @ {baud} baud")
        
        # Creating the input sub of cammnd val
        self.sub = self.create_subscription(WheelCommands,topic, self.on_wheel_commands, 10)
        period = 1.0 /max(1.0, send_rate_hz)
        self.timer = self.create_timer(period, self.send_packet)
        
    def on_wheel_commands(self, msg: WheelCommands):
        #Left/Right speed
        self.last_left_speed = float(msg.left_speed)
        self.last_right_speed = float(msg.right_speed)
        # Left/Right steer
        self.last_left_steer = float(msg.left_steer)
        self.last_right_steer = float(msg.right_steer)
        # timing
        self.last_command_time = time.monotonic()
        
    def build_packet(self, left_speed: float,
                           right_speed: float, 
                           left_steer: float, 
                           right_steer: float, 
                           flags: int = 0):
        # scaling
        # the rounding is required becuase 1.2 * 1000.0 can produce 1199.9998 then int truncates the .XX
        
        left_speed_i16 = clamp_i16(int(round(left_speed * 1000.0)))    # m/s -> mm/s
        right_speed_i16 = clamp_i16(int(round(right_speed * 1000.0)))  # m/s -> mm/s
        left_steer_i16 = clamp_i16(int(round(left_steer * 1000.0)))    # rad -> mrad
        right_steer_i16 = clamp_i16(int(round(right_steer * 1000.0)))  # rad -> mrad
        
        sof = b"\xAA\x55" # Start-of-frame - if you see AA 55 then you know a package is coming
        
        seq = self.seq & 0xFF # Sequence number - & 0xFF keeps only the lowest 8 bits 0 -255 then wraps around
        flags = flags & 0xFF # this the bit field we can flag, 0 -255
        
        # < is little endian
        # B is unsinged int 8-bit
        # h is signed int 16-bit
        payload = struct.pack("<BBhhhh",seq, flags, left_speed_i16, right_speed_i16, left_steer_i16, right_steer_i16)
        # 10 bytes payload
        
        crc = crc8_atm(payload) # checksum if fail discard
        pkg = sof + payload + struct.pack("<B", crc) # completing the full package
        #13 bytes total package
        return pkg 
    
    def send_packet(self):
        now = time.monotonic()
        
        
        if(now - self.last_command_time) > self.timeout_s:
            left_speed = 0.0
            right_speed = 0.0
            left_steer = 0.0
            right_steer = 0.0
            flags = 0x01 
        
        else:
            left_speed = self.last_left_speed
            right_speed = self.last_right_speed
            left_steer = self.last_left_steer
            right_steer = self.last_right_steer
            flags = 0x00
        
        pkt = self.build_packet(left_speed, right_speed, left_steer, right_steer, flags)
        
        try:
            self.serial.write(pkt)
        except serial.SerialTimeoutException:
                self.get_logger().warn("Serial write timeout", throttle_duration_sec=10.0)
        except Exception as e:
                self.get_logger().error(f"Serial write error: {e}", throttle_duration_sec=10.0)
                        
        self.seq = (self.seq + 1) & 0xFF
    
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
            
