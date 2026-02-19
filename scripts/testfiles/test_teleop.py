import time
import numpy as np 
from crisp_py.robot import make_robot
from crisp_py.gripper.gripper import make_gripper

# %%
np.set_printoptions(precision=3, suppress=True)

left_arm = make_robot("panda_left")
right_arm = make_robot("panda_right")
left_gripper = make_gripper("gripper_left")
right_gripper = make_gripper("gripper_right")
left_arm.wait_until_ready()
print("Left arm is ready!")
right_arm.wait_until_ready()
print("Right arm is ready!")
left_gripper.wait_until_ready()
print("Left gripper is ready!")
right_gripper.wait_until_ready()
print("Right gripper is ready!")

# %%
print("Going to home position...")
left_arm.home(blocking=True)
left_gripper.open()
right_arm.home(blocking=True)
right_gripper.open()

time.sleep(0.5)
print("Switching to teleop controllers...")

left_arm.cartesian_controller_parameters_client.load_param_config(
    file_path="/home/cbrian/Frankabutton/pixi_panda_ros2_ITR/testfile/control/gravity_compensation.yaml"
)
left_arm.controller_switcher_client.switch_controller("cartesian_impedance_controller")

right_arm.joint_controller_parameters_client.load_param_config(
    file_path="/home/cbrian/Frankabutton/pixi_panda_ros2_ITR/testfile/control/joint_control.yaml"
)
right_arm.controller_switcher_client.switch_controller("joint_impedance_controller")


# %%
def sync(left_arm, right_arm, left_gripper, right_gripper):
    right_arm.set_target_joint(left_arm.joint_values)
    right_gripper.set_gripper_state(left_gripper.closing_state())

print("Ready for teleop...")
right_arm.node.create_timer(1.0 / 100.0, lambda: sync(left_arm, right_arm, left_gripper, right_gripper))
right_gripper.node.create_timer(1.0 / 100.0, lambda: sync(left_arm, right_arm, left_gripper, right_gripper))

try:
    while True:
        print(f"Left arm joint values (deg): {np.rad2deg(left_arm.joint_values)}")
        print(f"Left gripper value: {left_gripper.value}")
        
        print(f"Right arm joint values (deg): {np.rad2deg(right_arm.joint_values)}")
        print(f"Right gripper value: {right_gripper.value}")
        print("-" * 40)
        time.sleep(1.0)
except KeyboardInterrupt:
    print("User exits teleop...")
finally:
    print("Going to home position...")
    left_arm.home()
    right_arm.home()
    left_gripper.open()
    right_gripper.open()
    left_arm.shutdown()
    right_arm.shutdown()
    left_gripper.shutdown()
    right_gripper.shutdown()