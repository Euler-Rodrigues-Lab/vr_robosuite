import numpy as np
from vr_robosuite.factory import make_env,native_sim
from vr_robosuite.stereo import StereoRenderer


def test_stereo_has_parallax_and_restores_camera_and_physics():
    env=make_env();renderer=StereoRenderer(env,160,120)
    try:
        model,data=native_sim(env)
        position=model.cam_pos.copy();qpos=data.qpos.copy();time=data.time
        left,right=renderer.eyes()
        assert left.shape==right.shape==(120,160,3)
        assert left.dtype==np.uint8
        assert np.mean(np.abs(left.astype(float)-right.astype(float)))>1
        np.testing.assert_array_equal(model.cam_pos,position)
        np.testing.assert_array_equal(data.qpos,qpos)
        assert data.time==time
    finally:renderer.close();env.close()
