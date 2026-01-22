"""A simple example to demonstrate gravity compensation mode."""
import os
# %%
from crisp_py.robot import make_robot

robot = make_robot("panda")
robot.wait_until_ready()

robot.home(blocking=True)

script_dir = os.path.dirname(os.path.abspath(__file__))
# 2. Join it with the subfolder and filename
# This creates a full absolute path dynamically
yaml_path = os.path.join(script_dir, "control", "gravity_compensation.yaml")
# %%
robot.cartesian_controller_parameters_client.load_param_config(
    file_path=yaml_path
)
robot.controller_switcher_client.switch_controller("cartesian_impedance_controller")
robot.shutdown()