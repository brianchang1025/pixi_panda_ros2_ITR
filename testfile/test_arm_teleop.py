import time
from crisp_py.robot import make_robot

# %%

left_arm = make_robot("panda_left")
right_arm = make_robot("panda_right")
left_arm.wait_until_ready()
right_arm.wait_until_ready()

# %%
print("Going to home position...")
left_arm.home(blocking=True)
right_arm.home(blocking=True)

#left_arm.controller_switcher_client.deactivate_all_motion_controllers()
#right_arm.controller_switcher_client.deactivate_all_motion_controllers()

time.sleep(0.5)
print("Switching to teleop controllers...")
# %%
left_arm.controller_switcher_client.switch_controller("gravity_compensation")
right_arm.controller_switcher_client.switch_controller("joint_impedance_controller")


# %%
def sync(left_arm, right_arm):
    right_arm.set_target_joint(left_arm.joint_values)

print("Ready for teleop...")
right_arm.node.create_timer(1.0 / 100.0, lambda: sync(left_arm, right_arm))

try:
    while True:
        print(f"Left arm joint values: {left_arm.joint_values}")
        print(f"Right arm joint values: {right_arm.joint_values}")
        print("-" * 40)
        time.sleep(1.0)
except KeyboardInterrupt:
    print("User exits teleop...")
finally:
    print("Going to home position...")
    left_arm.home()
    right_arm.home()
    left_arm.shutdown()
    right_arm.shutdown()