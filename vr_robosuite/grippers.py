"""
6-DoF gripper with its open/close variant (V4 model using 4-bar linkage)
"""
import os
import numpy as np

from robosuite.models.grippers import register_gripper
from robosuite.models.grippers.gripper_model import GripperModel

from dual_kinova3_teleop import ASSETS
_current_dir = str(ASSETS / "source/robotiq")

class Robotiq2F85V4GripperBase(GripperModel):
    """
    Robotiq 2F-85 Gripper V4 (Realistic 4-bar linkage model).

    Args:
        idn (int or str): Number or some other unique identification string for this gripper instance
    """

    def __init__(self, idn=0):
        super().__init__(os.path.join(_current_dir, "2f85_v4.xml"), idn=idn)

    def format_action(self, action):
        return action

    @property
    def init_qpos(self):
        # Only the driver joint
        return np.zeros(1)

    @property
    def joints(self):
        # We only strictly control the driver joint.
        # The others are passive/constrained.
        # We must filter from super().joints to ensure we get the correctly prefixed name.
        return [j for j in super().joints if j.endswith("left_driver_joint")]

    @property
    def _important_geoms(self):
        # Using the pad geoms for contact definition
        return {
            "left_finger": [
                "left_pad1", "left_pad2"
            ],
            "right_finger": [
                "right_pad1", "right_pad2"
            ],
            "left_fingerpad": ["left_pad1", "left_pad2"],
            "right_fingerpad": ["right_pad1", "right_pad2"],
        }


@register_gripper
class Robotiq2F85V4Gripper(Robotiq2F85V4GripperBase):
    """
    1-DoF variant of Robotiq2F85V4GripperBase.
    """

    def format_action(self, action):
        """
        Args:
            action (np.array): gripper-specific action
        """
        # Pass through the action directly (expected to be absolute position 0..0.8)
        return action

    @property
    def speed(self):
        return 0.1 # Adjust as needed

    @property
    def dof(self):
        return 1


class AbilityBase(GripperModel):
    side = 'right'
    def __init__(self,idn=0):
        super().__init__(str(ASSETS/f'source/psyonic_{self.side}/ability_{self.side}_hand_corrected.xml'),idn=idn)
    def format_action(self,action):
        return np.asarray(action)
    @property
    def init_qpos(self):return np.zeros(10)
    @property
    def dof(self):return 10
    @property
    def _important_geoms(self):
        return {'left_finger':['thumb_L1_collision'], 'right_finger':['index_L1_collision'],
                'left_fingerpad':['thumb_L2_collision'],'right_fingerpad':['index_L2_collision']}

@register_gripper
class AbilityRight(AbilityBase):side='right'
@register_gripper
class AbilityLeft(AbilityBase):side='left'
