# panda_arm.py
import os
import shlex
import subprocess
import time
import threading
import rclpy


COLORS = {"R": "\033[91m", "G": "\033[92m", "B": "\033[94m", "RE": "\033[0m"}
TMPDIR = os.environ.get("TMPDIR", "/tmp")

class PandaArm:
    def __init__(self, name: str, ip: str, namespace: str, button_ctr:bool=False):
        self.name = name
        self.ip = ip
        self.ns = namespace
        self.log_path = os.path.join(TMPDIR, f"{namespace}_panda.log")
        self.is_connected = False
        self.button_ctr = button_ctr

    def launch(self, terminal_logic):
        self.cleanup()

        print(f"\n{COLORS['B']}Launching {self.name} Panda -> {self.ip}{COLORS['RE']}")
        robot_cmd = f"pixi run -e jazzy franka robot_ip:={shlex.quote(self.ip)} namespace:={self.ns}"
        self.process = terminal_logic(f"{self.ns}_panda", robot_cmd, self.log_path)
        
        #self.launch_camera()
        self.is_connected = self._wait_for_network()
        
        # if self.button_ctr:
        #     self._start_bridge_thread()
        return self.is_connected

    def launch_camera(self):
        print(f"{COLORS['B']}Launching Camera for {self.ns}...{COLORS['RE']}")
        camera_cmd = (
            f"cd ../pixi_realsense && pixi run -e jazzy ros2 launch realsense2_camera rs_launch.py "
            f"camera_name:={self.ns}_camera color_qos:=SENSOR_DATA color_info_qos:=SENSOR_DATA"
        )
        subprocess.Popen(camera_cmd, shell=True, preexec_fn=os.setpgrp)

    def run_health_dashboard(self, topics_sub, topics_pub):
        """Verifies ROS 2 topics are actually alive for this specific arm."""
        print(f"\n--- {COLORS['B']}{self.name} Panda Topic Subscription Test{COLORS['RE']} ---")
        all_good = True
        
        # Check Robot Subscriptions (Echo)
        for t_name in topics_sub:
            if not self._verify_topic(f"{self.ns}/{t_name}", mode="echo"):
                all_good = False
        print(f"--- {COLORS['B']}{self.name} Panda Topic Subscription Test{COLORS['RE']} ---")
        # Check Robot Publishers (Pub)
        for t_name, m_type in topics_pub.items():
            if not self._verify_topic(f"{self.ns}/{t_name}", mode="pub", msg_type=m_type):
                all_good = False

        return all_good

    def _verify_topic(self, full_topic, mode="echo", msg_type=""):
        if mode == "echo":
            cmd = f"timeout 3s pixi run -e jazzy ros2 topic echo /{full_topic} --once --no-arr"
        else:
            cmd = f"timeout 3s pixi run -e jazzy ros2 topic pub -1 /{full_topic} {msg_type} '{{}}'"
        
        res = subprocess.run(cmd, shell=True, capture_output=True)
        success = (res.returncode == 0)
        status = f"{COLORS['G']}✅ Success{COLORS['RE']}" if success else f"{COLORS['R']}❌ Fail{COLORS['RE']}"
        print(f"{status}: /{full_topic}")
        return success

    def _wait_for_network(self, timeout=12):
        start = time.time()
        while (time.time() - start) < timeout:
            try:
                output = subprocess.check_output("pixi run -e jazzy ros2 topic list", shell=True).decode()
                if f"/{self.ns}/franka_robot_state" in output:
                    return True
            except: pass
            time.sleep(1)
        return False

    # def _start_bridge_thread(self):
    #     def run():
    #         if not rclpy.ok(): rclpy.init()
    #         node = FrankaWebBridge(robot_ip=self.ip, namespace=self.ns)
    #         rclpy.spin(node)
    #     threading.Thread(target=run, daemon=True).start()

    def cleanup(self):
        """Kills existing processes for this specific arm namespace."""
        print(f"\nCleaning up existing processes for {self.ns}...")
        # Kill any ROS nodes or pixi runs specifically containing this namespace
        subprocess.run(
            ["pkill", "-9", "-f", f"namespace:={self.ns}"], 
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        # Also target the camera launch for this namespace
        subprocess.run(
            ["pkill", "-9", "-f", f"camera_name:={self.ns}"], 
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL
        )
        self.is_connected = False