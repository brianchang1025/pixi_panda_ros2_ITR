#!/usr/bin/env python3
# Copyright jk-ethz
# Released under GNU AGPL-3.0
# Contact us for other licensing options.

# Developed and tested on system version
# 4.2.1

# Inspired by
# https://github.com/frankaemika/libfranka/issues/63
# https://github.com/ib101/DVK/blob/master/Code/DVK.py

from itertools import count
from time import sleep
from threading import Event
import atexit
import argparse
from urllib.parse import urljoin
from utils.franka_client import FrankaClient


class FrankaLockUnlock(FrankaClient):
    def __init__(self, hostname: str, username: str, password: str, protocol: str = 'https', relock: bool = False):
        super().__init__(hostname, username, password, protocol=protocol)
        self._relock = relock
        atexit.register(self._cleanup)

    def _cleanup(self):
        #print("Cleaning up...")
        if self._relock:
            self._lock_unlock(unlock=False)
        if self._token is not None or self._token_id is not None:
            self._release_token()
        if self._logged_in:
            self._logout()
        #print(f"[{self._hostname}]Successfully cleaned up.")

    def _activate_fci(self):
        print(f"[{self._hostname}] Activating FCI...")
        fci_request = self._session.post(urljoin(self._hostname, f'/admin/api/control-token/fci'), \
                                         json={'token': self._token})
        assert fci_request.status_code == 200, f"[{self._hostname}]Error activating FCI."
        print(f"[{self._hostname}] Successfully activated FCI.")

    def _home_gripper(self):
        print(f"[{self._hostname}] Homing the gripper...")
        action = self._session.post(urljoin(self._hostname, f'/desk/api/gripper/homing'), \
                                    headers={'X-Control-Token': self._token})
        assert action.status_code == 200, f"[{self._hostname}]Error homing gripper."
        print(f"[{self._hostname}] Successfully homed the gripper.")

    def _lock_unlock(self, unlock: bool, force: bool = False):
        print(f'[{self._hostname}] {"Unlocking" if unlock else "Locking"} the robot...')
        action = self._session.post(urljoin(self._hostname, f'/desk/api/robot/{"open" if unlock else "close"}-brakes'), \
                                    files={'force': force},
                                    headers={'X-Control-Token': self._token})
        assert action.status_code == 200, f"[{self._hostname}] Error requesting brake open/close action."
        print(f"[{self._hostname}] Successfully {'unlocked' if unlock else 'locked'} the robot.")

    def run(self, unlock: bool = False, force: bool = False, wait: bool = False, request: bool = False, persistent: bool = False, fci: bool = False, home: bool = False) -> None:
        assert not request or wait, f"[{self._hostname}] Requesting control without waiting for obtaining control is not supported."
        assert not fci or unlock, f"[{self._hostname}] Activating FCI without unlocking is not possible."
        assert not fci or persistent, f"[{self._hostname}] Activating FCI without persistence is not possible."
        assert not home or unlock, f"[{self._hostname}] Homing the gripper without unlocking is not possible."
        self._login()
        try:
            assert self._token is not None or self._get_active_token_id() is None or wait, f"[{self._hostname}] Error requesting control, the robot is currently in use."
            while True:
                self._request_token(physically=request)
                try:
                    # Consider the timeout of 20 s for requesting physical access to the robot
                    for _ in range(20) if request else count():
                        if (not wait and not request) or self._is_active_token():
                            print(f'[{self._hostname}] Successfully acquired control over the robot.')
                            self._lock_unlock(unlock=unlock)
                            if home:
                                self._home_gripper()
                            if fci:
                                self._activate_fci()
                            return
                        if request:
                            print(f'[{self._hostname}] Please press the button with the (blue) circle on the robot to confirm physical access.')
                        elif wait:
                            print(f'[{self._hostname}] Please confirm the request message in the web interface on the logged in user.')
                        sleep(1)
                    # In case physical access was not confirmed, try again
                    self._release_token()
                finally:
                    if not persistent:
                        self._release_token()
        finally:
            if not persistent:
                self._logout()
    
    



