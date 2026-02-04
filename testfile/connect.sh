#!/usr/bin/env bash
set -euo pipefail

# usage:
#   ./connect.sh [IP1] [NS1] [IP2] [NS2]
# env fallbacks: ROBOT1_IP/ROBOT1_NS, ROBOT2_IP/ROBOT2_NS
# Example:
#   ./connect.sh 192.168.31.10 left 192.168.31.11 right

IP1="${1:-${ROBOT1_IP:-192.168.31.10}}"
NS1="${2:-${ROBOT1_NS:-left}}"
IP2="${3:-${ROBOT2_IP:-192.168.32.10}}"
NS2="${4:-${ROBOT2_NS:-right}}"

CMD1="pixi run -e jazzy franka robot_ip:=${IP1} namespace:=${NS1}"
CMD2="pixi run -e jazzy franka robot_ip:=${IP2} namespace:=${NS2}"

# create temp dir for logs
TMPDIR="${TMPDIR:-$(mktemp -d /tmp/panda_connect.XXXXXX)}"
LOG1="${TMPDIR}/panda1.log"
LOG2="${TMPDIR}/panda2.log"

# spawn a terminal (tries gnome-terminal, xterm, konsole, tmux; else background)
# the spawned terminal will run the command, tee to a log, then wait for 'q' to close.
open_terminal() {
  local title="$1"; shift
  local cmd="$1"; shift
  local log="$1"; shift

  local payload="( ${cmd} ) 2>&1 | tee -a '${log}'; echo; echo '[process exited]'; echo 'Press q to close this window...'; while true; do read -n1 -s key; [[ \"\$key\" == q ]] && break; done"

  if command -v gnome-terminal >/dev/null 2>&1; then
    gnome-terminal --title="${title}" -- bash -lc "${payload}" &
    return 0
  fi

  if command -v xterm >/dev/null 2>&1; then
    xterm -T "${title}" -e "bash -lc \"${payload}\"" &
    return 0
  fi

  if command -v konsole >/dev/null 2>&1; then
    konsole --new-tab -e bash -lc "${payload}" &
    return 0
  fi

  if command -v tmux >/dev/null 2>&1; then
    local session="panda_connect"
    if ! tmux has-session -t "${session}" 2>/dev/null; then
      tmux new-session -d -s "${session}" -n "${title}" "bash -lc \"${payload}\""
    else
      tmux new-window -t "${session}" -n "${title}" "bash -lc \"${payload}\""
    fi
    echo "Opened in tmux session '${session}'. Attach with: tmux attach -t ${session}"
    return 0
  fi

  # fallback: run in background in current terminal
  bash -c "${payload}" &
  echo "No GUI terminal found; launched in background (PID=$!)."
  return 0
}

# Launch both robot sessions (no FCI checks)
echo "Launching Panda1 -> ${IP1} (ns=${NS1})"
open_terminal "Panda1-${NS1}" "${CMD1}" "${LOG1}"
sleep 0.5
echo "Launching Panda2 -> ${IP2} (ns=${NS2})"
open_terminal "Panda2-${NS2}" "${CMD2}" "${LOG2}"

echo
echo "Logs: ${LOG1}  ${LOG2}"
echo "Controls: press 'q' to terminate all and exit."

# monitor left log automatically: wait 5s, then poll last line until pattern appears or timeout.
monitor_left_log() {
  local logpath="$1"
  local pattern="waiting for service /left/controller_manager/list_controllers"
  # initial delay
  sleep 5
  local max_attempts=60   # total ~60s polling after initial delay
  local attempt=0
  while (( attempt < max_attempts )); do
    if [[ -f "$logpath" ]]; then
      # get last non-empty line
      local last_line
      last_line="$(tail -n 200 "$logpath" 2>/dev/null | awk 'NF{line=$0}END{print line}')"
      if [[ -n "$last_line" ]]; then
        if [[ "$last_line" == *"$pattern"* ]]; then
          echo "open the safe lock"
          return 0
        fi
      fi
    fi
    ((attempt++))
    sleep 1
  done
  return 1
}

# run monitor in background and capture PID so it can be killed on exit
echo "Waiting 5s before checking left terminal log..."
sleep 5
if monitor_left_log "${LOG1}"; then
  echo "open the safe lock"
else
  echo "pattern not detected in left log after polling period"
fi

# controller loop: 'q' -> kill pixi and exit. monitor runs automatically.
while true; do
  read -n1 -s -p "Press 'q' to terminate all robot sessions and exit: " k
  echo
  if [[ "$k" == "q" ]]; then
    echo "Terminating pixi processes..."
    pkill -f "pixi run -e jazzy franka" 2>/dev/null || true
    if command -v tmux >/dev/null 2>&1 && tmux has-session -t panda_connect 2>/dev/null; then
      tmux kill-session -t panda_connect 2>/dev/null || true
    fi
    break
  fi
done

# cleanup
if kill -0 "$MONITOR_PID" 2>/dev/null; then
  kill "$MONITOR_PID" 2>/dev/null || true
fi

echo "Done."