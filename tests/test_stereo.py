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


def test_default_camera_matches_sew_neutral_head_without_tracking():
    from scipy.spatial.transform import Rotation
    from geo_kin_core.types import RetargetFrame
    env=make_env();renderer=StereoRenderer(env,160,120)
    try:
        model,data=native_sim(env)
        base=data.xmat[renderer.base].reshape(3,3)
        expected=base@Rotation.from_euler('ZYX',[-90,0,65],degrees=True).as_matrix()
        np.testing.assert_allclose(data.cam_xmat[renderer.cam].reshape(3,3),expected,atol=1e-12)
        before=data.cam_xmat[renderer.cam].copy()
        renderer.track_head(RetargetFrame(head_rotation=np.eye(3)))
        np.testing.assert_allclose(data.cam_xmat[renderer.cam],before,atol=1e-12)
        assert (-expected[:,2])[2] < 0
    finally:renderer.close();env.close()
