# Validation

Checked locally on Linux with Python 3.10, robosuite 1.5.1, MuJoCo 3.3.7,
GStreamer 1.20, MediaMTX 1.15.6 and EGL rendering:

- All three tasks load and advance physics with both Robotiq and Psyonic hands.
- SEW Psyonic sample replay and MINK Psyonic sample replay produce finite actions.
- Stereo rendering produces distinct images and restores camera / physics state.
- GStreamer RGB → UDP MPEG-TS → MediaMTX → aiortc WHEP decoding succeeds at
  320×240 per eye. The README image is the decoded 640×240 video frame.
- Unit checks cover trial planning, recording explicit error outcomes, refusing
  directory overwrites, and GStreamer pipeline validation.

The original task success rules are retained. Trial termination was repaired so
success is returned in `info` after the normal robosuite step, rather than marking
the environment done before stepping. These are integration checks, not evidence
that the supplied free-space sample motions solve the experiment tasks.

Before collecting participant data, check the actual Quest client on the study
network: eye order, comfortable IPD, head-view mapping, pose reconnection, and
motion-to-display latency under the selected resolution. No physical Quest was
available for this validation. No hardware robot was operated. DexMimicGen task
compatibility has not been tested.

## Demo and full-access validation (2026-10-17)

- 12 Rust checks cover license verification, demo scope, the shared process
  deadline and total banner animation duration.
- 15 isolated demo-wheel checks cover solver parity, mounted grasp-site FK,
  pinch-frame tracking and missing-landmark hold, plus a real 60-second expiry.
  Session recreation and module reload cannot reset that timer. A covering dev
  license unlocks pose/TCP even after the same process's demo timer expires.
- 19 app/full-wheel checks cover camera initialization, metric pinch input,
  actual simulated gripper closure, graceful demo expiry, and solver behavior.
- Seven provisioner checks cover product validation, explicit virtualenv
  interpreter selection, and preserving the previous wheel on validation failure.
- Both full and demo wheels pass the binary symbol/path hygiene audit.

The demo requires explicit `geo-kin-provision install --wheel ...` after
`uv sync`; CI provisions it before checking import and construction with a
missing license path. Normal dependency sync preserves the provisioned wheel.

Separate left/right WHEP endpoints decode successfully, with crop orientation
checked against the combined stream. The user confirmed working Quest streaming.
Headset comfort, reconnection and latency under study conditions still need
operator validation. No physical robot was operated.
