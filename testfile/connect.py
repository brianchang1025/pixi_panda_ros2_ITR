# ...existing code...
#!/usr/bin/env python3
import threading
import rclpy
import os
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import time
from typing import Optional
from franka_bridge_node import FrankaWebBridge  # Ensure this is in the same directory or properly installed

TMPDIR = os.environ.get("TMPDIR") or tempfile.mkdtemp(prefix="panda_connect.")
LOG1 = os.path.join(TMPDIR, "left_panda.log")
LOG2 = os.path.join(TMPDIR, "right_panda.log")
TMUX_SESSION = "panda_connect"
R = "\033[91m"
G = "\033[92m"  # Green
B = "\033[94m"  # Blue
RE = "\033[0m"
topics_to_sub = {
    "current_pose": "geometry_msgs/msg/PoseStamped",
    "joint_states": "sensor_msgs/msg/JointState",
    "current_twist": "geometry_msgs/msg/TwistStamped",
}
topics_to_pub = {
    "target_pose": "geometry_msgs/msg/PoseStamped",
    "target_joint": "sensor_msgs/msg/JointState",
}

def which(name: str) -> Optional[str]:
    return shutil.which(name)

def make_payload(cmd: str, log: str) -> str:
    return (
        f"{cmd} 2>&1 | tee -a '{log}'; echo; echo '[process exited]'; "
        "echo 'Press q to close this window...'; while true; do read -n1 -s key; [[ \"$key\" == q ]] && break; done"
    )

def launch_in_terminal(title: str, cmd: str, log: str) -> Optional[subprocess.Popen]:
    payload = make_payload(cmd, log)
    if which("gnome-terminal"):
        args = ["gnome-terminal", "--title", title, "--", "bash", "-lc", payload]
        return subprocess.Popen(args)
    if which("konsole"):
        args = ["konsole", "--new-tab", "-e", "bash", "-lc", payload]
        return subprocess.Popen(args)
    if which("xterm"):
        args = ["xterm", "-T", title, "-e", "bash", "-lc", payload]
        return subprocess.Popen(args)
    if which("tilix"):
        args = ["tilix", "-a", "session-add-right", "-x", "bash", "-lc", payload]
        return subprocess.Popen(args)
    if which("tmux"):
        if subprocess.run(["tmux", "has-session", "-t", TMUX_SESSION],
                          stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL).returncode != 0:
            subprocess.run(["tmux", "new-session", "-d", "-s", TMUX_SESSION, "-n", title, "bash", "-lc", payload])
        else:
            subprocess.run(["tmux", "new-window", "-t", TMUX_SESSION, "-n", title, "bash", "-lc", payload])
        print(f"Opened in tmux session '{TMUX_SESSION}'. Attach: tmux attach -t {TMUX_SESSION}")
        return None
    # fallback: background in current shell
    p = subprocess.Popen(["bash", "-lc", payload])
    return p

def kill_pixi_processes(ns: str = ""):
    if ns == "":
        subprocess.run(["pkill", "-9", "-f", "pixi run -e jazzy franka"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    else:
        subprocess.run(["pkill", "-9", "-f", f"namespace:={ns}"], 
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if which("tmux"):
        subprocess.run(["tmux", "kill-session", "-t", TMUX_SESSION], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

def prompt_default(prompt_text: str, default: str) -> str:
    try:
        val = input(f"{prompt_text} default [{default}]: ").strip()
    except EOFError:
        val = ""
    return val if val else default

def read_single_key() -> str:
    import termios, tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def is_robot_on_network(namespace: str, timeout: int = 15):
    """Checks if the robot is actually broadcasting data on ROS 2."""
    print(f"Waiting for {namespace} Panda to appear on ROS 2 network...")
    start_time = time.time()
    
    while (time.time() - start_time) < timeout:
        # Check the topic list
        cmd = "pixi run -e jazzy ros2 topic list"
        try:
            output = subprocess.check_output(cmd, shell=True).decode()
            if f"/{namespace}/franka_robot_state" in output:
                print(f"✅{G}{namespace} Panda detected on network!{RE}")
                return True
        except:
            pass
        time.sleep(1)
        
    return False

def run_bridge_thread(ip, ns):
    """Starts a ROS 2 node for the bridge in a dedicated thread."""
    # We must initialize rclpy for each thread or once globally
    if not rclpy.ok():
        rclpy.init()
    
    node = FrankaWebBridge(robot_ip=ip, namespace=ns)
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Bridge Error for {ns}: {e}")
    finally:
        node.destroy_node()

def launch_left_Panda(title: str, ip_left: str, log: str) -> Optional[subprocess.Popen]:
    
    print(f"\n{B}Launching Left Panda -> {ip_left} (ns=left){RE}")
    cmd = f"pixi run -e jazzy franka robot_ip:={shlex.quote(ip_left)} namespace:=left"
    term = launch_in_terminal(title, cmd, log)
    time.sleep(0.5)
    print("left Logs:", log)
    print("checking left terminal for connection...")
    connect = is_robot_on_network("left", timeout=10)
    if connect:
        print(f"{G}✅Connection with left arm successful{RE}")
        bridge_thread = threading.Thread(target=run_bridge_thread, args=(ip_left, "left"), daemon=True)
        bridge_thread.start()
        return True
    else:
        print(f"❌{R}Connection fail, check the FCI and safe lock for left Panda{RE}")
        print(f"❌{R}Terminating left Panda pixi processes...{RE}")
        kill_pixi_processes("left")
        return False

def launch_right_Panda(title: str, ip_right: str, log: str) -> Optional[subprocess.Popen]:  
    print(f"\n{B}Launching Right Panda -> {ip_right} (ns=right){RE}")
    cmd = f"pixi run -e jazzy franka robot_ip:={shlex.quote(ip_right)} namespace:=right"
    term = launch_in_terminal(title, cmd, log)
    time.sleep(0.5)
    print("right Logs:", log)
    print("checking right terminal for connection...")
    connect = is_robot_on_network("right", timeout=10)
    if connect:
        print(f"{G}✅Connection with right arm successful{RE}")
        return True
    else:
        print(f"❌{R}Connection fail, check the FCI and safe lock for right Panda{RE}")
        print(f"❌{R}Terminating right Panda pixi processes...{RE}")
        kill_pixi_processes("right")
        return False

def can_subscribe(namespace, topic):
    full_topic = f"/{namespace}/{topic}"
    # 'timeout 2s' kills it if no message arrives. 
    # '--once' makes it exit immediately after 1 message.
    cmd = f"timeout 2s pixi run -e jazzy ros2 topic echo {full_topic} --once --no-arr"
    
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    
    if result.returncode == 0:
        print(f"✅ {G}Subscribe Success:{RE} Received data from {full_topic}")
        return True
    else:
        print(f"❌ {R}Subscribe Fail:{RE} No data on {full_topic}")
        return False
    
def can_publish(namespace, topic, msg_type):
    full_topic = f"/{namespace}/{topic}"
    # We publish exactly once (-1) and wait for discovery
    # Using an empty/default message '{}' just to check the pipe
    cmd = f"timeout 3s pixi run -e jazzy ros2 topic pub -1 {full_topic} {msg_type} '{{}}'"
    
    result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
    
    # In ROS 2, if 'pub' finds a subscriber and sends successfully, it exits with 0
    if result.returncode == 0:
        print(f"✅ {G}Publish Success:{RE} Topic {full_topic} is reachable")
        return True
    else:
        print(f"❌ {R}Publish Fail:{RE} Could not reach {full_topic}")
        return False
    
def main():
    # defaults
    def_ip1 = os.environ.get("ROBOT1_IP", "192.168.31.10")
    ns1 = os.environ.get("ROBOT1_NS", "left")
    def_ip2 = os.environ.get("ROBOT2_IP", "192.168.32.10")
    ns2 = os.environ.get("ROBOT2_NS", "right")

    # interactive prompt with defaults; press Enter to accept default
    print("Enter IPs. Press Enter to accept default in brackets.")
    ip1 = prompt_default("IP_left", def_ip1)
    ip2 = prompt_default("IP_right", def_ip2)
    print("Temp dir for logs:", TMPDIR)
    

    global p1, p2
    p1 = launch_left_Panda(f"{ns1} Panda", ip1, LOG1)
    time.sleep(2)
    p2 = launch_right_Panda(f"{ns2} Panda", ip2, LOG2)
    

    while True:
        # both running -> done
        if p1 and p2:
            print(f"\n{G}✅Connected both Pandas.{RE}")
            break

        # left down, right up
        if not p1 and p2:
            print(f"\n❌{R}Connection with left Panda failed{RE};✅{G}Connection with right Panda successful.{RE}")
            print(f"Press {B}'y'{RE} to reconnect the left Panda, or {B}'q'{RE} to quit the whole program.")
            k = read_single_key()
            if k.lower() == "y":
                print(f"\n{B}Reconnecting left Panda...{RE}")
                p1 = launch_left_Panda(f"{ns1} Panda", ip1, LOG1)
                time.sleep(0.5)
                continue
            if k.lower() == "q":
                print(f"\n{R}Quitting program and terminating pixi processes...{RE}")
                kill_pixi_processes()
                return
            print(f"\nPress {B}'y'{RE} to reconnect left Panda, or {B}'q'{RE} to quit.")
            time.sleep(0.2)
            continue

        # right down, left up
        if p1 and not p2:
            print(f"\n❌{R}Connection with right Panda failed{RE};✅{G}Connection with left Panda successful.{RE}")
            print(f"Press {B}'y'{RE} to reconnect the right Panda, or {B}'q'{RE} to quit the whole program.")
            k = read_single_key()
            if k.lower() == "y":
                print(f"\n{B}Reconnecting right Panda...{RE}")
                p2 = launch_right_Panda(f"{ns2} Panda", ip2, LOG2)
                time.sleep(0.5)
                continue
            if k.lower() == "q":
                print(f"\n{R}Quitting program and terminating pixi processes...{RE}")
                kill_pixi_processes()
                return
            print(f"\nPress {B}'y'{RE} to reconnect right Panda, or {B}'q'{RE} to quit.")
            time.sleep(0.2)
            continue

        # both down
        if not p1 and not p2:
            print(f"\n❌{R}Connections to both Panda failed.{RE}")
            print(f"Press {B}'y'{RE} to reconnect both Pandas, or {B}'q'{RE} to quit the whole program.")
            k = read_single_key()
            if k.lower() == "y":
                print(f"\n{B}Reconnecting left Panda...{RE}")
                p1 = launch_left_Panda(f"{ns1} Panda", ip1, LOG1)
                time.sleep(0.5)
                print(f"\n{B}Reconnecting right Panda...{RE}")
                p2 = launch_right_Panda(f"{ns2} Panda", ip2, LOG2)
                time.sleep(0.5)
                continue
            if k.lower() == "q":
                print(f"\n{R}Quitting program and terminating pixi processes...{RE}")
                kill_pixi_processes()
                return
            print(f"\nPress {B}'y'{RE} to reconnect both Pandas, or {B}'q'{RE} to quit.")
            time.sleep(0.2)
            continue
    # note: user requested an initial 5s wait before checking; we already waited 5s before launching second terminal,
    # so use initial_delay=0 here. If explicit 5s desired, set initial_delay=5.0 above.
    
    print(f"\n--- {B}Left Panda Health Dashboard{RE} ---")
    left_all_good = True
    for t_name, msg_type in topics_to_sub.items():
        if not can_subscribe("left", t_name) :
            left_all_good = False
    for t_name, msg_type in topics_to_pub.items():
        if not can_publish("left", t_name, msg_type):
            left_all_good = False

    if left_all_good:
        print(f"\n✅ {G}Left Panda. Ready to control.{RE}")
    else:
        print(f"\n❌ {R}Left Panda link incomplete. Check FCI connection.{RE}")

    print(f"\n--- {B}Right Panda Health Dashboard{RE} ---")
    right_all_good = True
    for t_name, msg_type in topics_to_sub.items():
        if not can_subscribe("right", t_name) :
            right_all_good = False
    for t_name, msg_type in topics_to_pub.items():
        if not can_publish("right", t_name, msg_type):
            right_all_good = False

    if right_all_good:
        print(f"\n✅ {G}Right Panda. Ready to control.{RE}")
    else:
        print(f"\n❌ {R}Right Panda link incomplete. Check FCI connection.{RE}")

    print("\nControls: press 'q' (single key) here to terminate all pixi sessions and exit.")
    while True:
        k = read_single_key()
        if k == "q":
            print("\nTerminating pixi processes...")
            kill_pixi_processes()
            break

    for p in (p1, p2):
        if isinstance(p, subprocess.Popen):
            try:
                p.terminate()
                time.sleep(0.2)
                p.kill()
            except Exception:
                pass

    print("Done. Logs kept in", TMPDIR)

if __name__ == "__main__":
    main()
# ...existing code...