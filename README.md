# VR Robosuite

![Stereo cabinet scene received through WebRTC](docs/media/webrtc-stereo.png)

Quest binocular teleoperation for the three SEW-Geometric-Teleop study tasks:
**ExpCabinet**, **ExpGlassGap**, and **ExpHandover**. Both Dual Kinova 3
**Robotiq** and **Psyonic Ability** variants run with robosuite task physics.

The pinned [dual_kinova3_teleop](https://github.com/Euler-Rodrigues-Lab/dual_kinova3_teleop)
submodule supplies robot assets, sample motions and retargeting sessions. Its
nested XRT_devices supplies Quest input, replay, recording, and the reusable
GStreamer RGB publisher. There is no dependency on a local SEW source checkout.

## Setup (Linux)

```bash
git clone --recurse-submodules https://github.com/Euler-Rodrigues-Lab/vr_robosuite.git
cd vr_robosuite
uv sync --python 3.10 --extra mink --extra test
sudo apt-get install gstreamer1.0-tools gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly
export MUJOCO_GL=egl
```

Dual Kinova is a **free experimental product**. Its wheel supports Robotiq and
Psyonic Ability hands with no license file, activation or expiry. Other robot
products retain their existing license requirements.

The Linux x86_64 wheel (Python 3.10+, glibc 2.34+) is bundled under
`external/dual_kinova3_teleop/wheels/` and installed automatically by `uv sync`.
No separate download or provisioner command is required:

```bash
git submodule update --init --recursive
uv sync --extra mink --extra test
uv run python -c "import geo_kin; print(geo_kin.FREE_PRODUCTS)"
```

This one wheel covers both gripper variants. Existing licenses for other robots
are preserved, and an unrelated `GEO_KIN_LICENSE` does not block Dual Kinova.
The distribution grant and SHA256 checksum accompany the wheel in the submodule.
Binary use and redistribution are free of charge; solver source remains proprietary.

## Preview and both sample recordings

```bash
uv run vr-robosuite --device idle --no-stream --steps 1 --snapshot cabinet.png
uv run vr-robosuite --device replay --sample picking_up_mustard \
  --variant psyonic --environment ExpGlassGap --no-stream --snapshot mustard.png
uv run vr-robosuite --device replay --sample ipman_roll \
  --variant psyonic --environment ExpHandover --no-stream --snapshot ipman.png
```

Without streaming, these commands render offscreen and save the requested final
image. Omit `--no-stream` after starting MediaMTX to watch in Quest. Both sample
motions come from the submodule; they demonstrate control and are not successful
solutions to the study tasks. Use `--frames PATH` or `--csv PATH` for other inputs.

## Binocular video to Quest

Set the workstation's LAN address (reachable from Quest), then start MediaMTX:

```bash
export STREAM_HOST=192.168.1.100  # replace with your workstation address
docker compose up -d
uv run vr-robosuite --environment ExpCabinet --variant robotiq
```

Alternatively run a native MediaMTX binary with `config/mediamtx.yml`, setting
`MTX_WEBRTCADDITIONALHOSTS` to the same workstation LAN address.

Use the native [XRT-Client](https://github.com/Euler-Rodrigues-Lab/XRT-Client)
Quest application. Set pose signaling to `http://PC_IP:8080/offer`, connect with
**Power**, then press **Play** to start sending tracking. In the FPV/ZEDv2 scene,
connect video to `http://PC_IP:8889/zed_stereo` (the client adds `/whep`).
`zed_stereo` is the existing client's stream name; no ZED camera is needed.
Use its single side-by-side stereo mode, not separate left/right URLs.

The video path is MuJoCo RGB → XRT_devices GStreamer → H.264 MPEG-TS over local
UDP 9000 → MediaMTX → WebRTC/WHEP → Quest. Both eyes use one simulation state:
left eye in the left half, right eye in the right half. Defaults are 640×480 per
eye, 30 fps, 65 mm IPD, 75° vertical FOV. Adjust `--width`, `--height`, `--rate`,
`--ipd`, and `--bitrate` for the workstation. Head yaw/pitch follows the source
experiment's camera mapping; `--no-head-tracking` fixes the view. This is stereo
FPV, not a full six-degree-of-freedom VR renderer.

Allow TCP 8080 (tracking), TCP 8889 (WHEP signaling) and UDP 8189 (video) between
Quest and the workstation. The supplied server configuration is for a trusted
local study network. Do not expose it publicly. Tracking and video are separate
connections; removing the headset can pause poses until Play is pressed again.
Task physics pauses when input is missing/stale, while the last scene continues
streaming.

```bash
uv run python scripts/doctor.py
uv run python scripts/check_stream.py --output received.png
docker compose down
```

The receiver check requires a running publisher. See [validation](docs/validation.md)
for the checks actually performed, including the remaining headset check.

## Study sessions

The source pipeline uses two controllers per environment. Codes are
`1=ExpCabinet`, `2=ExpGlassGap`, `3=ExpHandover`; controller codes are
`1=sew_mimic`, `2=mink`. Choose the counterbalancing sequence for each participant:

```bash
uv run vr-study --participant P01 --order 132 --controllers 121221 \
  --variant psyonic --duration 120
```

This runs six trials, waits for operator Enter between trials, and records an
atomic plan under `recordings/studies/P01`. Repeat the **same command** to resume.
Completed trials are skipped; interrupted/error attempts remain recorded and
are retried in new directories. Changed order, controllers, seed or trial options
require a new participant/output directory. `--plan-only` writes the plan without
launching a trial; `--auto` removes the operator prompt. Use pseudonymous IDs.

Each trial stores `trial.json` (options, package/repository versions and explicit
outcome), `model.xml`, and incremental `trajectory.h5` (qpos, qvel, ctrl, action,
and `[monotonic_time, simulation_time, valid_input]`). Live XR input is recorded
under `input/`. Outcomes distinguish success, timeout, termination, interruption,
source exhaustion, preview and error; missing files never count as task failure.
Use `--env-kwargs '{"log_data":true}'` to additionally enable the source contact
logger; set `VR_ROBOSUITE_CONTACT_DIR` to its desired output directory.

SEW uses the shared TCP-focused solver (tracked elbows remain inputs), index MCP
TCP for Psyonic, fixed robot base, and the hardware-profile self-collision safety
filter by default. Shoulder relocation is disabled for these fixed-base tasks.
`--no-safety-filter` disables the filter. The MINK option uses the shared MINK
fallback with joint limits; it does **not** apply the Rust safety filter and is
not a numerical reproduction of the old experiment's whole-body MINK controller.
The trial metadata records whether the safety filter is active. Self-collision
filtering does not provide task-object collision avoidance.

## Additional robosuite tasks

```bash
uv run vr-robosuite --plugin YOUR_REGISTRATION_MODULE \
  --environment YOUR_REGISTERED_ENV --env-kwargs '{"YOUR_OPTION": 1}'
```

Modules are imported before `robosuite.make`. Additional tasks must support the
DualKinova3 fixed-base robot and installed robosuite version. This is the extension
point for DexMimicGen; DexMimicGen tasks are not bundled or validated here.

```bash
MUJOCO_GL=egl uv run pytest -q
```

See [third-party notices](THIRD_PARTY_NOTICES.md): the wine glass asset carries
CC BY-NC terms. The binary wheel is bundled in the Dual Kinova submodule and available as a release asset; license files are never committed.
