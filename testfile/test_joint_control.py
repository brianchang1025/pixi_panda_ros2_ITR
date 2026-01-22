"""Example controlling the joints."""
import time
import os

from crisp_py.robot import make_robot

robot = make_robot("panda")
robot.wait_until_ready()

print(f"Left arm joint values: {robot.joint_values}")
robot.home(blocking=True)


robot.controller_switcher_client.switch_controller("joint_trajectory_controller")
print(f"Left arm joint values: {robot.joint_values}")
# %%
q = robot.joint_values
q[5] += 0.0
q[6] +=0.0
robot.set_target_joint(q)
print(f"Left arm joint values: {robot.joint_values}")
time.sleep(1.0)

robot.shutdown()