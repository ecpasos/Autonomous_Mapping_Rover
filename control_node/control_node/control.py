import rclpy
from rclpy.node import Node
import serial
import threading

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        self.get_logger().info("ControlNode has started.")
        self.selected_port = "/dev/cu.usbserial-0001"

        try:
            self.serialInst = serial.Serial(port=self.selected_port, baudrate=115200, timeout=0.01)
            self.get_logger().info(f"Connected to {self.selected_port}")
            self.running = True
            self.read_thread = threading.Thread(target=self.read_serial_loop)
            self.read_thread.start()
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            self.serialInst = None

    def read_serial_loop(self):
        while self.running:
            try:
                if self.serialInst.in_waiting:
                    packet = self.serialInst.readline().decode('utf-8', errors='replace').strip()
                    self.get_logger().info(f"Received: {packet}")
            except Exception as e:
                self.get_logger().warn(f"Serial read failed: {e}")

    def destroy_node(self):
        self.running = False
        if self.read_thread.is_alive():
            self.read_thread.join()
        if self.serialInst:
            self.serialInst.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
