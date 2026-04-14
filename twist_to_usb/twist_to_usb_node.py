#!/usr/bin/env python3
import struct
import time

import rclpy
from rclpy.node import Node
from control_tower_ros2.msg import DiffWheelCommands

import serial

# clamp the value between -32768 and 32767
def clamp_i16(x:int):
    return max(-32768, min(32767,x))

def crc8_atm(data: bytes) -> int:
    """CRC-8/ATM (poly 0x07, init 0x00). Covers bytes 2-19 of the packet (full payload)."""
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
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
        self.leftWheelVel = 0.0
        self.rightWheelVel = 0.0
        
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
        self.sub = self.create_subscription(DiffWheelCommands,topic, self.on_wheel_commands, 10)
        period = 1.0 /max(1.0, send_rate_hz)
        self.timer = self.create_timer(period, self.send_packet)
        
    def on_wheel_commands(self, msg: DiffWheelCommands):
        
        # Front motors 
        self.l_wheel_vel = float(msg.v_left)
        self.r_wheel_vel = float(msg.v_right)

        self.lastCommandTime = time.monotonic()
        
    def build_packet(self, leftWheelVel: float,
                           rightWheelVel: float, 
                           flags: int = 0):
        # scaling
        # the rounding is required becuase 1.2 * 1000.0 can produce 1199.9998 then int truncates the .XX
        # Front motors
        leftWheelVelI16 = clamp_i16(int(round(leftWheelVel * 1000.0)))    # m/s -> mm/s
        rightWheelVelI16 = clamp_i16(int(round(rightWheelVel * 1000.0)))  # m/s -> mm/s

        
        # Start-of-frame
        sof = b"\xAA\x55"
        # Sequence number: 0-255
        seq = self.seq & 0xFF 
        flags = flags & 0xFF
        
        # < is little endian
        # B is unsinged int 8-bit
        # h is signed int 16-bit
        payload = struct.pack("<BBhh",seq, flags, leftWheelVelI16, 
                                                    rightWheelVelI16, 
)
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
            leftWheelVel = 0.0
            rightWheelVel = 0.0
            # flag
            flags = 0x01 
        
        else:
            # Front motors
            leftWheelVel = self.leftWheelVel
            rightWheelVel = self.rightWheelVel

            flags = 0x00
        
        pkt = self.build_packet(leftWheelVel,
                                rightWheelVel,
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
            
