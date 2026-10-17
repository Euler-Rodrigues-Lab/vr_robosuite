import numpy as np
import pytest
from vr_robosuite.factory import make_env,native_sim

@pytest.mark.parametrize('variant',['robotiq','psyonic'])
@pytest.mark.parametrize('name',['ExpCabinet','ExpGlassGap','ExpHandover'])
def test_original_tasks_step_with_named_absolute_joint_actions(name,variant):
    env=make_env(name,variant)
    try:
        model,data=native_sim(env);robot=env.robots[0];prefix=robot.robot_model.naming_prefix
        commands={s:np.array([data.qpos[model.joint(f'{prefix}{s}_Actuator{i}').qposadr[0]] for i in range(1,8)]) for s in ['right','left']}
        commands.update({s+'_gripper':np.zeros(10 if variant=='psyonic' else 1) for s in ['right','left']})
        action=robot.create_action_vector(commands)
        for _ in range(3):_,_,done,info=env.step(action)
        assert np.isfinite(data.qpos).all()
        assert not done and not info['success']
    finally:env.close()
