import numpy as np
import os

from robosuite.models.robots.manipulators.manipulator_model import ManipulatorModel

from dual_kinova3_teleop import ASSETS
_dualkinova3_XML_path = str(ASSETS / "source/robot/robot.xml")

class DualKinova3(ManipulatorModel):
    """
    The Gen3 robot is the sparkly newest addition to the Kinova line

    Args:
        idn (int or str): Number or some other unique identification string for this robot instance
    """

    arms = ["right", "left"]

    def __init__(self, idn=0):
        super().__init__(_dualkinova3_XML_path, idn=idn)

    @property
    def default_base(self):
        # return "OmronMobileBaseSRL"
        # return "OmronMobileBase"
        return "RethinkMount"

    @property
    def default_gripper(self):
        """
        Since this is bimanual robot, returns dict with `'right'`, `'left'` keywords corresponding to their respective
        values

        Returns:
            dict: Dictionary containing arm-specific gripper names
        """
        # return {"right": "Robotiq85GripperCorrected", "left": "Robotiq85GripperCorrected"}
        return {"right": "Robotiq2F85V4Gripper", "left": "Robotiq2F85V4Gripper"}

    @property
    def default_controller_config(self):
        return {"right": "default_kinova3", "left": "default_kinova3","head": "default_kinova3_head",}

    @property
    def init_qpos(self):
        return np.array([
            # head joints
            0.0, 0.0,
            # Right arm joints
            np.pi/2, np.pi/2, -np.pi/2, np.pi/2, 0.0, np.pi/6, np.pi/2-np.radians(12),
            # Left arm joints
            np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, 0.0, -np.pi/6, -np.pi/2+np.radians(12),
                         ])

    @property
    def base_xpos_offset(self):
        return {
            "bins": (-0.5, -0.1, 0),
            "empty": (-0.6, 0, 0),
            "table": lambda table_length: (-0.16 - table_length / 2, 0, 0),
        }

    @property
    def top_offset(self):
        return np.array((0, 0, 1.0))

    @property
    def _horizontal_radius(self):
        return 0.5

    @property
    def arm_type(self):
        return "bimanual"

    @property
    def _eef_name(self):
        """
        Since this is bimanual robot, returns dict with `'right'`, `'left'` keywords corresponding to their respective
        values

        Returns:
            dict: Dictionary containing arm-specific eef names
        """
        return {"right": "right_hand", "left": "left_hand"}
