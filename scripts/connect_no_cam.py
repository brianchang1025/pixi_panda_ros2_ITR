import os
import sys
import tempfile
import time
import rclpy
from threading import Thread
from rclpy.executors import MultiThreadedExecutor
from utils.panda_arm import PandaArm
from utils.utils import COLORS, launch_in_terminal, kill_all, read_single_key, prompt_default
from utils.franka_bridge_node import FrankaWebBridge

# Configuration
TMPDIR = os.environ.get("TMPDIR") or tempfile.mkdtemp(prefix="panda_connect.")
TOPICS_TO_SUB = {
    "current_pose": "geometry_msgs/msg/PoseStamped",
    "joint_states": "sensor_msgs/msg/JointState",
    "current_twist": "geometry_msgs/msg/TwistStamped",
    #"franka_buttons/check": "std_msgs/msg/Bool",
    #"franka_buttons/cross": "std_msgs/msg/Bool",
    "gripper/joint_states": "sensor_msgs/msg/JointState",
    #"gripper/gripper_closing_controller/commands": "std_msgs/msg/Bool",

}
TOPICS_TO_PUB = {
    "target_pose": "geometry_msgs/msg/PoseStamped",
    "target_joint": "sensor_msgs/msg/JointState",
    "franka_buttons/check": "std_msgs/msg/Bool",
    "franka_buttons/cross": "std_msgs/msg/Bool",
    #"gripper/closing_state": "std_msgs/msg/Bool",
    "gripper/joint_states": "sensor_msgs/msg/JointState",
}

def main():
    rclpy.init()

    ip1 = prompt_default("IP Left Panda", "192.168.31.10")
    ip1_ns = prompt_default("Namespace Left Panda", "left")
    ip2 = prompt_default("IP Right Panda", "192.168.32.10")
    ip2_ns = prompt_default("Namespace Right Panda", "right")

    node_l = FrankaWebBridge(robot_ip=ip1, namespace=ip1_ns)
    node_r = FrankaWebBridge(robot_ip=ip2, namespace=ip2_ns)
    executor = MultiThreadedExecutor()
    executor.add_node(node_l)
    executor.add_node(node_r)

    # Start spinning in a background thread so the rest of the script can run
    spin_thread = Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print(f"\nif both Panda's status lights are {COLORS['B']}blue{COLORS['RE']}, press enter to start connecting ...")
    print(f"Otherwise please {COLORS['B']}reopen the USER STOP{COLORS['RE']} on both Pandas before continuing.")
    
    while True:
            
        k = read_single_key() # Note: In raw mode, 'Enter' is often '\r' (carriage return)
        if k == '\r' or k == '\n' or k.lower() == 'enter':
                break
        elif k.lower() == 'q':
            node_l.cleanup()
            node_r.cleanup()
            rclpy.shutdown()
            time.sleep(1)    # Small delay to allow any pending logs to print
            kill_all()
            return
        else:
            print(f"Press {COLORS['B']}Enter{COLORS['RE']} to continue (or {COLORS['R']}q{COLORS['RE']} to quit)...")    
    
    left_arm = PandaArm("Left", ip1, ip1_ns)
    right_arm = PandaArm("Right", ip2, ip2_ns)
    
    # Parallel Launch
    left_arm.launch(launch_in_terminal)
    time.sleep(2)  # Stagger launches slightly for cleaner output
    right_arm.launch(launch_in_terminal)
    time.sleep(1.5)

    while True:
        status_l = f"{COLORS['G']}Online{COLORS['RE']}" if left_arm.is_connected else f"{COLORS['R']}OFFLINE{COLORS['RE']}"
        status_r = f"{COLORS['G']}Online{COLORS['RE']}" if right_arm.is_connected else f"{COLORS['R']}OFFLINE{COLORS['RE']}"
        
        # Clear line and print current status
        sys.stdout.write(f"\n\r[STATUS] Left: {status_l} | Right: {status_r} | Commands: [l] retry Left, [r] retry Right, [q] quit  ")
        sys.stdout.flush()

        # Check if both are ready to move to dashboard
        if left_arm.is_connected and right_arm.is_connected:
            print(f"\n\n{COLORS['G']}✅ All systems operational!{COLORS['RE']}")
            break

        # Non-blocking-ish input check
        k = read_single_key().lower()
        if k == 'l':
            print(f"\n{COLORS['B']}Relaunching Left Panda (cleaning old processes)...{COLORS['RE']}")
            left_arm.launch(launch_in_terminal)
        elif k == 'r':
            print(f"\n{COLORS['B']}Relaunching Right Panda (cleaning old processes)...{COLORS['RE']}")
            right_arm.launch(launch_in_terminal)
        elif k == 'q':
            kill_all()
            return

    # 4. Health Dashboard (Now inside the class!)
    left_arm.run_health_dashboard(TOPICS_TO_SUB, TOPICS_TO_PUB)
    right_arm.run_health_dashboard(TOPICS_TO_SUB, TOPICS_TO_PUB)

    # 5. Persistent Control
    print(f"\n🚀 {COLORS['G']}Lab Environment Ready.{COLORS['RE']}")
    print(f"Press {COLORS['B']}'q'{COLORS['RE']} to terminate all processes and exit.")
    
    while True:
        if read_single_key().lower() == 'q':
            node_l.cleanup()
            node_r.cleanup()
            rclpy.shutdown()
            time.sleep(1)  # Give some time for cleanup logs to print
            kill_all()
            break
    
    # 2. Shutdown RO
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        kill_all()