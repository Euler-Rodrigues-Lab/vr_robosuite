# Sources and licenses

The experiment environments, custom objects, and contact logger were adapted from
SEW-Geometric-Teleop. Its root MIT notice is retained in `SOURCE_LICENSE.txt`.
`provenance.json` records the original repository revision, source paths, and
SHA-256 hashes **before adaptation**. Local changes include package imports,
shared robot assets, configurable logging, and successful-trial termination fixes.

The wine glass XML explicitly attributes its mesh to
https://www.thingiverse.com/thing:4001273/files under **CC BY-NC**. That notice
is preserved in `vr_robosuite/objects/assets/objects/wine_glass.xml`.
These asset-specific terms remain applicable; the assets are not all MIT.

Robot / hand models and sample motions are referenced through the pinned
`dual_kinova3_teleop` submodule. See its asset provenance and third-party notices.
XRT_devices, geo_kin_core, robosuite, MuJoCo, MediaMTX, and other dependencies
retain their own licenses. The proprietary Dual Kinova geo_kin binary wheel is
provided only for simulation demonstrations within this vr_robosuite environment.
Redistribution, standalone use, other-environment use, and physical robot control
are prohibited; see `external/dual_kinova3_teleop/wheels/FREE_DUAL_KINOVA3.md`.
Only the copyright holder or an expressly authorized distributor may supply it.
Other geo_kin products and organization licenses are supplied separately.
