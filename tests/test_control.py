import numpy as np
from geo_kin_core.types import RetargetFrame
from vr_robosuite.control import RobotControl,pinch_distance
from vr_robosuite.factory import make_env,native_sim
from vr_robosuite.cli import parser


def test_pose_is_default_and_distance_is_in_metres():
    assert parser().parse_args([]).retarget_mode=='pose'
    frame=RetargetFrame(right_gripper_val=.03)
    assert pinch_distance(frame,'right')==.03
    frame.right_fingers={'thumb':{'thumb_tip':[0,0,0]},'index':{'index_finger_tip':[.08,0,0]}}
    assert pinch_distance(frame,'right')==.08
    assert pinch_distance(RetargetFrame(right_gripper_val=float('nan')),'right') is None
    assert pinch_distance(RetargetFrame(right_gripper_val=-1),'right') is None


def test_pinching_closes_and_separating_opens_physical_gripper():
    env=make_env()
    try:
        control=RobotControl(env)
        model,data=native_sim(env)
        right_joint=env.robots[0].gripper['right'].joints[0]
        adr=model.joint(right_joint).qposadr[0]
        close=RetargetFrame(right_gripper_val=.02,left_gripper_val=.08)
        for _ in range(30):env.step(control.action(close))
        assert control.grips['right'][0]==.8
        assert control.grips['left'][0]==0
        closed=data.qpos[adr]
        assert closed>.5
        for _ in range(30):env.step(control.action(RetargetFrame(right_gripper_val=.09)))
        assert control.grips['right'][0]==0
        assert data.qpos[adr]<closed-.3
        control.action(RetargetFrame())
        assert control.grips['right'][0]==0
    finally:env.close()
