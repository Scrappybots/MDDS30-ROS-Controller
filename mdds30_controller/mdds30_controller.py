#!/usr/bin/env python3
# Written By: Bryan Ribas
# CC BY-SA 4.0

import rclpy
from rclpy.node import Node
import serial.tools.list_ports
from serial import Serial
from std_msgs.msg import Int16


class MDDS30Controller(Node):
    def __init__(self):
        super().__init__('mdds30_controller')
        
        # Declare parameters - Default port for Jetson Orin Nano
        self.declare_parameter('port', '/dev/ttyTHS0')
        portname = self.get_parameter('port').get_parameter_value().string_value
        
        self.serialConnection = None
        
        # Connect to the serial port
        try:
            self.serialConnection = Serial(
                port=portname,
                baudrate=9600,
                timeout=0.5
            )
            self.get_logger().info(f"[MDDS30_controller]: Connected to port {portname}")
        # Print error if connection fails but don't shutdown the node here
        except Exception as e:
            self.get_logger().error(f"[MDDS30_controller]: Error connecting to MDDS30: {str(e)}")
            self.serialConnection = None
            return

        # Setup the subscribers for each motor's speeds
        self.left_motor_sub = self.create_subscription(
            Int16,
            'MDDS30_controller/motor_left',
            lambda msg: self.motor_change(msg, "left"),
            10
        )
        
        self.right_motor_sub = self.create_subscription(
            Int16,
            'MDDS30_controller/motor_right',
            lambda msg: self.motor_change(msg, "right"),
            10
        )
        
        self.get_logger().info("[MDDS30_controller]: Node initialized successfully")

    def motor_change(self, speed, motor):
        packet = bytearray()
        
        # We need to convert the -+100 to the byte expected
        # by the MDDS300. See the manual for more info:
        # http://www.tinyurl.com/3ddaevwa
        
        # First we check if the direction is CCW (positive) or CW (negative)
        if speed.data >= 0:  # CCW motion
            if motor == "left":
                byteToSend = int((speed.data * 63) / 100) + 64
            else:
                byteToSend = int((speed.data * 63) / 100) + 192
        else:  # CW motion
            if motor == "left":
                byteToSend = int(((speed.data * -1) * 63) / 100)
            else:
                byteToSend = int(((speed.data * -1) * 63) / 100) + 128
        
        # Send the packet
        if self.serialConnection is not None:
            packet.append(byteToSend)
            self.serialConnection.write(packet)
            packet.clear()
            # Let the log know what changed
            self.get_logger().info(f"[MDDS30_controller]: Motor {motor} Speed Changed To: {speed.data}")

    def shutdown(self):
        self.get_logger().info("[MDDS30_controller]: shutting down!")
        # If the serial connection was open attempt to close it.
        if self.serialConnection is not None:
            try:
                self.serialConnection.close()
                self.get_logger().info("[MDDS30_controller]: Closed serial connection")
            except Exception as e:
                self.get_logger().error(f"[MDDS30_controller]: Error disconnecting to MDDS30: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    
    controller = None
    try:
        controller = MDDS30Controller()
        if controller.serialConnection is not None:  # Only spin if connection was successful
            rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if controller is not None:
            controller.shutdown()
        if rclpy.ok():  # Only shutdown if rclpy was successfully initialized
            rclpy.shutdown()


if __name__ == '__main__':
    main()
