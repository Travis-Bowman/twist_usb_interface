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
        self.lastCommandTime = 0.0
        
        # Front motors
        self.lastFrontLeftSpeed = 0.0
        self.lastFrontLeftSteer = 0.0
        self.lastFrontRightSpeed = 0.0
        self.lastFrontRightSteer = 0.0
        # Rear motors
        self.lastRearLeftSpeed = 0.0
        self.lastRearLeftSteer = 0.0
        self.lastRearRightSpeed = 0.0
        self.lastRearRightSteer = 0.0
        
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
        
        # Front motors 
        self.lastFrontLeftSpeed = float(msg.front_left_speed)
        self.lastFrontLeftSteer = float(msg.front_left_steer)
        self.lastFrontRightSpeed = float(msg.front_right_speed)
        self.lastFrontRightSteer = float(msg.front_right_steer)
        # Rear motors
        self.lastRearLeftSpeed = float(msg.rear_left_speed)
        self.lastRearLeftSteer = float(msg.rear_left_steer)
        self.lastRearRightSpeed = float(msg.rear_right_speed)
        self.lastRearRightSteer = float(msg.rear_right_steer)
        # Timestamp
        self.lastCommandTime = time.monotonic()
        
    def build_packet(self, frontLeftSpeed: float,
                           frontLeftSteer: float, 
                           frontRightSpeed: float,
                           frontRightsteer: float,
                           rearLeftSpeed: float,
                           rearLeftSteer: float, 
                           rearRightSpeed: float,
                           rearRightsteer: float, 
                           flags: int = 0):
        # scaling
        # the rounding is required becuase 1.2 * 1000.0 can produce 1199.9998 then int truncates the .XX
        # Front motors
        frontLeftSpeedI16 = clamp_i16(int(round(frontLeftSpeed * 1000.0)))    # m/s -> mm/s
        frontLeftSteerI16 = clamp_i16(int(round(frontLeftSteer * 1000.0)))  # m/s -> mm/s
        frontRightSpeedI16 = clamp_i16(int(round(frontRightSpeed * 1000.0)))    # rad -> mrad
        frontRightsteerI16 = clamp_i16(int(round(frontRightsteer * 1000.0)))  # rad -> mrad
        # Rear motors
        rearLeftSpeedI16 = clamp_i16(int(round(rearLeftSpeed * 1000.0)))    # m/s -> mm/s
        rearLeftSteerI16 = clamp_i16(int(round(rearLeftSteer * 1000.0)))  # m/s -> mm/s
        rearRightSpeedI16 = clamp_i16(int(round(rearRightSpeed * 1000.0)))    # rad -> mrad
        rearRightsteerI16 = clamp_i16(int(round(rearRightsteer * 1000.0)))  # rad -> mrad
        
        # Start-of-frame
        sof = b"\xAA\x55"
        # Sequence number: 0-255
        seq = self.seq & 0xFF 
        flags = flags & 0xFF
        
        # < is little endian
        # B is unsinged int 8-bit
        # h is signed int 16-bit
        payload = struct.pack("<BBhhhhhhhh",seq, flags, frontLeftSpeedI16, 
                                                        frontLeftSteerI16, 
                                                        frontRightSpeedI16, 
                                                        frontRightsteerI16,
                                                        rearLeftSpeedI16,
                                                        rearLeftSteerI16,
                                                        rearRightSpeedI16,
                                                        rearRightsteerI16)
        # checksum if fail discard
        crc = crc8_atm(payload) 
        # completing the full package
        pkg = sof + payload + struct.pack("<B", crc) 
        #18 bytes total package
        return pkg 
    
    def send_packet(self):
        now = time.monotonic()
        
        if(now - self.lastCommandTime) > self.timeout_s:
            # Front motors
            frontLeftSpeed = 0.0
            frontLeftSteer = 0.0
            frontRightSpeed = 0.0
            frontRightsteer = 0.0
            # Rear motors
            rearLeftSpeed = 0.0
            rearLeftSteer = 0.0
            rearRightSpeed = 0.0
            rearRightsteer = 0.0
            # flag
            flags = 0x01 
        
        else:
            # Front motors
            frontLeftSpeed = self.lastFrontLeftSpeed
            frontLeftSteer = self.lastFrontLeftSteer
            frontRightSpeed = self.lastFrontRightSpeed
            frontRightsteer = self.lastFrontRightSteer
            # Rear motors
            rearLeftSpeed = self.lastRearLeftSpeed
            rearLeftSteer = self.lastRearLeftSteer
            rearRightSpeed = self.lastRearRightSpeed
            rearRightsteer = self.lastRearRightSteer
            flags = 0x00
        
        pkt = self.build_packet(frontLeftSpeed,
                                frontLeftSteer,
                                frontRightSpeed,
                                frontRightsteer,
                                rearLeftSpeed,
                                rearLeftSteer,
                                rearRightSpeed,
                                rearRightsteer,
                                flags)
        
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
            
