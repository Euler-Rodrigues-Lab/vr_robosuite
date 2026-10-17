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

## Free experimental wheel (2026-10-17)

The free Dual Kinova build passes eight Rust license/grant checks, 12 installed
wheel parity checks, and five provisioner checks. Robotiq and Psyonic SEW replay
both advance with GEO_KIN_LICENSE pointing to a nonexistent file. The default
live Robotiq path starts XR signaling and GStreamer without changing the active
RBY1 license. The binary passes the symbol/path hygiene audit. These checks do
not require a paid license; the grant is scoped to Dual Kinova products.

The bundled wheel installs through `uv sync --locked` in a fresh virtual
environment, imports from that environment's site-packages, and constructs both
Dual Kinova hand variants with a nonexistent license path. CI checks this import
and construction after a recursive checkout.
