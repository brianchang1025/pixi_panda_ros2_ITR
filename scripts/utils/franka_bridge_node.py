import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import hashlib, base64, json, ssl, threading, requests, urllib3
# Note: Ensure your utils folder is in the python path or remove the COLORS import for standalone testing
try:
    from utils.utils import COLORS
except ImportError:
    COLORS = {'G': '\033[92m', 'RE': '\033[0m'}

from websockets.sync.client import connect
from utils.franka_lock_unlock import FrankaLockUnlock
from http import HTTPStatus

urllib3.disable_warnings(urllib3.exceptions.InsecureRequestWarning)

class FrankaWebBridge(Node):
    def __init__(self, robot_ip: str, namespace: str = ""):
        super().__init__('franka_web_bridge', namespace=namespace)
        
        # --- CONFIGURATION ---
        self.declare_parameter('robot_ip', robot_ip)
        self.declare_parameter('username', 'admin')
        self.declare_parameter('password', 'pandapanda')

        self.ip = self.get_parameter('robot_ip').value
        self.user = self.get_parameter('username').value
        self.pw = self.get_parameter('password').value

        # ROS Publishers
        self.cross_pub = self.create_publisher(Bool, 'franka_buttons/cross', 10)
        self.check_pub = self.create_publisher(Bool, 'franka_buttons/check', 10)
        
        # Internal State
        self.session = requests.Session()
        self.session.verify = False 
        self.base_url = f"https://{self.ip}"
        self.token = None      # Auth token for WS
        self.token_id = None   # Control token ID for Hardware

        # Start the sequence
        self.initialize_robot()

    def encode_password(self, user, pw):
        """The specific hash Franka Desk expects."""
        digest = hashlib.sha256(f'{pw}#{user}@franka'.encode('utf-8')).digest()
        bytes_str = ','.join([str(b) for b in digest])
        return base64.b64encode(bytes_str.encode('utf-8')).decode('utf-8')

    def initialize_robot(self):
        """Sequence: Login -> Get Control -> Unlock -> FCI -> Start Thread"""
        try:
            # 1. Login
            self.franka_lock_unlock = FrankaLockUnlock(self.ip, self.user, self.pw, relock=True)
            self.franka_lock_unlock.run(unlock=True, wait=True, request=True, fci=True, persistent=True, home=True)
            
            # login_url = f'https://{self.ip}/admin/api/login'
            # payload = {'login': self.user, 'password': self.encode_password(self.user, self.pw)}
            # r = self.session.post(login_url, json=payload, timeout=5)
            
            # 2. Get Token (the response text is the token)
            self.token = self.franka_lock_unlock.get_logged_in_token()
            print(f"🌐 {COLORS['G']}WebBridge Online{COLORS['RE']} [{self.ip}]")

            # 5. Start WebSocket Listener for Buttons in a background thread
            threading.Thread(target=self.listen_loop, daemon=True).start()

        except Exception as e:
            self.get_logger().error(f"Initialization Failed: {e}")

    def listen_loop(self):
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE
        
        # Using events endpoint for button presses
        uri = f'wss://{self.ip}/desk/api/navigation/events'
        
        try:
            with connect(uri, ssl_context=ctx, additional_headers={'authorization': self.token}) as ws:
                while rclpy.ok():
                    msg = ws.recv()
                    data = json.loads(msg)
                    
                    # Logic for publishing buttons
                    if 'cross' in data:
                        is_pressed = bool(data['cross'])
                        #print(f"[{self.ip}] ❌ Cross Button: {is_pressed}")
                        self.cross_pub.publish(Bool(data=is_pressed))
                    if 'check' in data:
                        is_pressed = bool(data['check'])
                        #print(f"[{self.ip}] ✅ Check Button: {is_pressed}")
                        self.check_pub.publish(Bool(data=is_pressed))
                        
        except Exception as e:
            if rclpy.ok():
                self.get_logger().warn(f"WebSocket Loop closed: {e}")
    
    def cleanup(self):
        """Releases the control token when the node stops."""
        if self.token_id:
            #print(f"\n[{self.ip}] 🔒 Releasing control token...")
            self.franka_lock_unlock._cleanup() # Force lock to ensure it's released
            self.session.delete(f"{self.base_url}/admin/api/control-token", json={'token': self.token})

# def main(args=None):
#     rclpy.init(args=args)
    
#     # --- TEST CONFIG ---
#     # Put your robot IP here for the test
#     test_ip = "192.168.31.10" 
    
#     node = FrankaWebBridge(robot_ip=test_ip)
   

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.cleanup()
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()