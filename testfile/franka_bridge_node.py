import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import hashlib, base64, json, ssl, threading, requests
from websockets.sync.client import connect

class FrankaWebBridge(Node):
    def __init__(self, robot_ip: str, namespace: str = ""):
        super().__init__('franka_web_bridge', namespace=namespace)
        
        # --- CONFIGURATION ---
        self.declare_parameter('robot_ip', robot_ip) # Change to your IP
        self.declare_parameter('username', 'admin')
        self.declare_parameter('password', 'pandapanda')

        self.ip = self.get_parameter('robot_ip').value
        self.user = self.get_parameter('username').value
        self.pw = self.get_parameter('password').value

        
        # Publisher for the Cross button
        self.cross_pub = self.create_publisher(Bool, 'franka_buttons/cross', 10)
        # Publisher for the Checkmark button
        self.check_pub = self.create_publisher(Bool, 'franka_buttons/check', 10)
        
        # Setup Session
        self.session = requests.Session()
        self.session.verify = False # Ignore SSL certs from robot
        
        self.start_bridge()

    def encode_password(self, user, pw):
        """The specific hash Franka Desk expects."""
        digest = hashlib.sha256(f'{pw}#{user}@franka'.encode('utf-8')).digest()
        bytes_str = ','.join([str(b) for b in digest])
        return base64.b64encode(bytes_str.encode('utf-8')).decode('utf-8')

    def start_bridge(self):
        try:
            # 1. Login
            login_url = f'https://{self.ip}/admin/api/login'
            payload = {'login': self.user, 'password': self.encode_password(self.user, self.pw)}
            r = self.session.post(login_url, json=payload, timeout=5)
            
            # 2. Get Token (the response text is the token)
            self.token = r.text
            self.get_logger().info(f"Connected to {self.ip}. Listening for buttons...")
            
            # 3. Start Listener Thread
            threading.Thread(target=self.listen_loop, daemon=True).start()
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")

    def listen_loop(self):
        ctx = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
        ctx.check_hostname = False
        ctx.verify_mode = ssl.CERT_NONE
        
        uri = f'wss://{self.ip}/desk/api/navigation/events'
        
        with connect(uri, ssl_context=ctx, additional_headers={'authorization': self.token}) as ws:
            while rclpy.ok():
                msg = ws.recv()
                data = json.loads(msg)
            
                # Check for the 'cross' button in the JSON data
                if 'cross' in data:
                    msg_cross = Bool()
                    msg_cross.data = bool(data['cross'])
                    self.cross_pub.publish(msg_cross)
                    #self.get_logger().info(f"Cross (Left Side) is: {msg_cross.data}")
                # Handle Check Button (Top)
                elif 'check' in data:
                    msg_check = Bool()
                    msg_check.data = bool(data['check'])
                    self.check_pub.publish(msg_check)
                    #self.get_logger().info(f"Check (Right Side) is: {msg_check.data}")
        
        

