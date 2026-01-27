import rclpy
import time
from crisp_py.robot import make_robot

# %%

robot = make_robot("panda")
print("Waiting for robot to be ready...")
robot.wait_until_ready()
print(f"Robot current joint values: {robot.joint_values}")

print("Homing the robot...")
robot.home(blocking=True)
print(f"Robot joint values after homing: {robot.joint_values}")

robot.controller_switcher_client.switch_controller("gravity_compensation")
print("Ready for manual teleop...")


try:
    while rclpy.ok():
        rclpy.spin_once(robot.node, timeout_sec=0.01)
        print(f"Robot current joint values: {robot.joint_values}")
        print("-" * 40)
        time.sleep(1.0)
except KeyboardInterrupt:
    print("User exits teleop...")
finally:
    print("Going to home position...")
    robot.home() 
    robot.shutdown()