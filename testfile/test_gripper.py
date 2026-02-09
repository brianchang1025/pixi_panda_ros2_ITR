"""Simple example to control the gripper."""

# %%
import time


from crisp_py.gripper.gripper import make_gripper

gripper = make_gripper("gripper_left")
gripper.config.max_delta = 0.15
gripper.wait_until_ready()
print("Gripper is ready!")

# %%
freq = 1.0
rate = gripper.node.create_rate(freq)
t = 0.0
while t < 2.0:
    print(gripper.value)
    print(gripper.torque)
    rate.sleep()
    t += 1.0 / freq

# %%
gripper.value

gripper.set_target(0.3)
time.sleep(3.0)
gripper.set_target(0.0)
time.sleep(3.0) 
gripper.set_target(0.5)
time.sleep(3.0)
gripper.stop()
while True:
    print(gripper.value)
    
    rate.sleep()

