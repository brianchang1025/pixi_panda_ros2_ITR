import rclpy
from rclpy.node import Node
# Updated from RobotState to FrankaState
from franka_msgs.msg import FrankaState 

class ButtonDetector(Node):
    def __init__(self):
        super().__init__('button_detector_node')
        
        # Subscribe using the correct Message Type
        self.subscription = self.create_subscription(
            FrankaState, 
            '/franka_robot_state_broadcaster/robot_state',
            self.robot_state_callback,
            10)
            
        self.get_logger().info("Listening for FrankaState... Go press the Pilot buttons!")

    def robot_state_callback(self, msg):
    # This will print ONLY when the mode changes
        if not hasattr(self, 'last_mode'):
            self.last_mode = msg.robot_mode
            print(f"Initial Robot Mode: {msg.robot_mode}")

        if msg.robot_mode != self.last_mode:
            print(f"MODE CHANGED: From {self.last_mode} to {msg.robot_mode}")
            self.last_mode = msg.robot_mode

def main(args=None):
    rclpy.init(args=args)
    node = ButtonDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()