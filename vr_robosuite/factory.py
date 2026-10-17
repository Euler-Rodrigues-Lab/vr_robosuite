"""Registered study tasks and an import hook for other robosuite environments."""
from pathlib import Path
import importlib
import json
import numpy as np

ENVIRONMENTS=('ExpCabinet','ExpGlassGap','ExpHandover')


def make_env(name='ExpCabinet',variant='robotiq',rate=30,seed=0,plugins=(),**extra):
    import robosuite as suite
    from robosuite.models.robots.robot_model import REGISTERED_ROBOTS
    from robosuite.robots import ROBOT_CLASS_MAPPING, FixedBaseRobot
    from .models import DualKinova3
    from . import grippers, envs
    REGISTERED_ROBOTS['DualKinova3']=DualKinova3
    ROBOT_CLASS_MAPPING['DualKinova3']=FixedBaseRobot
    for module in plugins:importlib.import_module(module)
    config=suite.load_composite_controller_config(controller=str(Path(__file__).parent/'configs/joint_position.json'),robot='DualKinova3')
    for arm in config['body_parts'].values():arm['policy_freq']=rate
    hands='default' if variant=='robotiq' else [['AbilityRight','AbilityLeft']]
    np.random.seed(seed)
    kwargs=dict(robots='DualKinova3',controller_configs=config,gripper_types=hands,
                has_renderer=False,has_offscreen_renderer=False,use_camera_obs=False,
                initialization_noise=None,control_freq=rate,horizon=1000000,ignore_done=True,
                hard_reset=False)
    if name in ENVIRONMENTS:kwargs['log_data']=False
    kwargs.update(extra)
    env=suite.make(name,**kwargs)
    env.reset()
    return env


def native_sim(env):
    return env.sim.model._model, env.sim.data._data
