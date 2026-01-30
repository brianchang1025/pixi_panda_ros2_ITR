#!/usr/bin/env bash
set -euo pipefail

# usage: ./connect.sh [IP1] [NS1] [IP2] [NS2]
IP1="${1:-${ROBOT1_IP:-192.168.31.10}}"
NS1="${2:-${ROBOT1_NS:-left}}"
IP2="${3:-${ROBOT2_IP:-192.168.32.10}}"
NS2="${4:-${ROBOT2_NS:-right}}"

CMD1="pixi run -e jazzy franka robot_ip:=${IP1} namespace:=${NS1}"
CMD2="pixi run -e jazzy franka robot_ip:=${IP2} namespace:=${NS2}"

open_terminal() {
  local title="$1"
  local cmd="$2"

  if command -v gnome-terminal >/dev/null 2>&1; then
    # --title sets the window name, -- bash -lc runs the command
    gnome-terminal --title="${title}" -- bash -lc "${cmd}; echo; echo '[process exited]'; exec bash"
  elif command -v konsole >/dev/null 2>&1; then
    konsole --p "tabtitle=${title}" -e bash -c "${cmd}; echo; echo '[process exited]'; exec bash" &
  elif command -v xterm >/dev/null 2>&1; then
    xterm -T "${title}" -n "${title}" -e "bash -lc '${cmd}; echo; echo \"[process exited]\"; exec bash'" &
  elif command -v tmux >/dev/null 2>&1; then
    session="panda_connect"
    if ! tmux has-session -t "${session}" 2>/dev/null; then
      tmux new-session -d -s "${session}" -n "${title}" "${cmd}"
    else
      tmux new-window -t "${session}" -n "${title}" "${cmd}"
    fi
  else
    echo "No terminal emulator found. Running ${title} in background..."
    bash -c "${cmd}" &
  fi
}

echo "Launching Panda 1 ($NS1) at $IP1..."
open_terminal "${NS1}" "${CMD1}"

sleep 1.0 # Give the first one a second to initialize

echo "Launching Panda 2 ($NS2) at $IP2..."
open_terminal "${NS2}" "${CMD2}"

echo "Terminals opened with titles: $NS1 and $NS2"