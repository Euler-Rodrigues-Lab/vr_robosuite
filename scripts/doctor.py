"""Check simulator imports, video plugins and the selected study assets."""
import importlib
from xrt_devices.video.gstreamer import check_plugins
for name in ['robosuite','mujoco','dual_kinova3_teleop','xrt_devices','vr_robosuite']:
    module=importlib.import_module(name);print(name,module.__file__)
check_plugins();print('GStreamer UDP/MPEG-TS encoder: OK')
from vr_robosuite.factory import make_env
for name in ['ExpCabinet','ExpGlassGap','ExpHandover']:
    env=make_env(name);print(name,'action dimensions',env.action_dim);env.close()
