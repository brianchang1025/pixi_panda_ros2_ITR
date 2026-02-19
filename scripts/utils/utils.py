# utils.py
import os
import shutil
import subprocess
import sys
from typing import Optional

# Configuration Constants
TMUX_SESSION = "panda_connect"
COLORS = {
    "R": "\033[91m",
    "G": "\033[92m", 
    "B": "\033[94m", 
    "RE": "\033[0m"
}

def which(name: str) -> Optional[str]:
    """Check if a command is available on the system."""
    return shutil.which(name)

def make_payload(cmd: str, log: str) -> str:
    """Creates a shell script payload that keeps the terminal open on exit."""
    return (
        f"{cmd} 2>&1 | tee -a '{log}'; echo; echo '[process exited]';"
        "echo 'Press q to close this window...'; "
        "while true; do read -n1 -s key; [[ \"$key\" == q ]] && break; done"
    )

def launch_in_terminal(title: str, cmd: str, log: str) -> Optional[subprocess.Popen]:
    """Spawns the given command in a new GUI terminal or tmux window."""
    payload = make_payload(cmd, log)
    
    if which("gnome-terminal"):
        args = ["gnome-terminal", "--title", title, "--", "bash", "-lc", payload]
        return subprocess.Popen(args)
    
    if which("tmux"):
        has_session = subprocess.run(["tmux", "has-session", "-t", TMUX_SESSION], 
                                     capture_output=True).returncode == 0
        if not has_session:
            subprocess.run(["tmux", "new-session", "-d", "-s", TMUX_SESSION, "-n", title, "bash", "-lc", payload])
        else:
            subprocess.run(["tmux", "new-window", "-t", TMUX_SESSION, "-n", title, "bash", "-lc", payload])
        return None

    return subprocess.Popen(["bash", "-lc", payload])

def kill_all():
    """Kills all Pixi, ROS 2, and UI sessions."""
    print(f"\n{COLORS['R']}Shutting down all system processes...{COLORS['RE']}")
    subprocess.run(["pkill", "-9", "-f", "pixi run -e jazzy"], stderr=subprocess.DEVNULL)
    if which("tmux"):
        subprocess.run(["tmux", "kill-session", "-t", TMUX_SESSION], stderr=subprocess.DEVNULL)

def read_single_key() -> str:
    """Reads one character from stdin without requiring Enter."""
    import termios, tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
    return ch

def prompt_default(prompt_text: str, default: str) -> str:
    """Input prompt that returns default if user presses Enter or just spaces."""
    try:
        # We add a hint (Enter to accept) so the user knows what to do
        hint = f"{COLORS['B']}(Press Enter to accept default){COLORS['RE']}"
        
        # Display: IP Left Panda [192.168.31.10] (Enter to accept): 
        user_input = input(f"{prompt_text} [{default}] {hint}: ").strip()
        
        # If user_input is empty, 'bool(user_input)' is False, so it returns 'default'
        return user_input if user_input else default
        
    except EOFError:
        return default