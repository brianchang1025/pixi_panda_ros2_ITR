import time
from crisp_py.gripper.gripper import make_gripper

left_gripper = make_gripper("gripper_left")

left_gripper.wait_until_ready()
print("Left gripper is ready!")

right_gripper = make_gripper("gripper_right")

right_gripper.wait_until_ready()
print("Right gripper is ready!")

print("Going to home position...")
left_gripper.open()
right_gripper.open()
time.sleep(0.5)

left_gripper.stop()

def sync(left_gripper, right_gripper):
    right_gripper.set_gripper_state(left_gripper.closing_state())

print("Ready for teleop...")
right_gripper.node.create_timer(1.0 / 100.0, lambda: sync(left_gripper, right_gripper))

try:
    while True:
        print(f"Left gripper value: {left_gripper.closing_state()}")
        print(f"Right gripper value: {right_gripper.closing_state()}")
        print("-" * 40)
        time.sleep(1.0)
except KeyboardInterrupt:
    print("User exits teleop...")
finally:
    print("Going to home position...")
    left_gripper.open()
    right_gripper.open()
    left_gripper.shutdown()
    right_gripper.shutdown()