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
    
    
    # persistent control loop: user can press
    #   'q' - quit everything
    #   'l' - reboot left arm
    #   'r' - reboot right arm
    print(f"\nPersistent control active: press 'q' to quit, 'l' to reboot left, 'r' to reboot right.")
    while True:
        k = read_single_key().lower()
        if k == 'q':
            node_l.cleanup()
            node_r.cleanup()
            rclpy.shutdown()
            time.sleep(1)  # Give some time for cleanup logs to print
            kill_all()
            break
        elif k == 'l':
            # reboot left arm: restart web bridge then relaunch arm
            print(f"\n{COLORS['B']}Rebooting Left Panda web bridge...{COLORS['RE']}")
            node_l.cleanup()
            node_l = FrankaWebBridge(robot_ip=ip1, namespace=ip1_ns)
            executor.add_node(node_l)
            time.sleep(0.5)  # give bridge a moment to reinitialize

            # wait for user to signal readiness before starting the arm process
            print(f"Press {COLORS['B']}Enter{COLORS['RE']} when ready to restart the Left Panda")
            while True:
                r = read_single_key()
                if r == '\r' or r == '\n' or r.lower() == 'enter':
                    break
            print(f"{COLORS['B']}Relaunching Left Panda process...{COLORS['RE']}")
            left_arm.launch(launch_in_terminal)
            time.sleep(1)  # let launch settle
        elif k == 'r':
            # reboot right arm: restart web bridge then relaunch arm
            print(f"\n{COLORS['B']}Rebooting Right Panda web bridge...{COLORS['RE']}")
            node_r.cleanup()
            node_r = FrankaWebBridge(robot_ip=ip2, namespace=ip2_ns)
            executor.add_node(node_r)
            time.sleep(0.5)

            # wait for user confirmation before restart
            print(f"Press {COLORS['B']}Enter{COLORS['RE']} when ready to restart the Right Panda")
            while True:
                r = read_single_key()
                if r == '\r' or r == '\n' or r.lower() == 'enter':
                    break
            # relaunch right arm; status checks in the outer loop cover safety
            print(f"{COLORS['B']}Relaunching Right Panda process...{COLORS['RE']}")
            right_arm.launch(launch_in_terminal)
            time.sleep(1)
        # additional keys could be handled here
    
    # 2. Shutdown RO
if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        kill_all()