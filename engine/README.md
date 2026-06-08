# `simhands_bridge.mm` — real webcam hands in the visionOS Simulator

This is a **native engine module** (not a drop-in script). It makes a Godot visionOS build, *while
running in the Simulator*, consume a [VisionOS-SimHands](https://github.com/BenLumenDigital/VisionOS-SimHands)-style
MultipeerConnectivity hand stream (webcam → MediaPipe, 21 landmarks) and write Godot's
`XRHandTracker` at `/user/hand_tracker/left|right` — so your game's normal hand-reading code
"just works" in the sim. GDScript can't speak MultipeerConnectivity, which is why this lives in
the engine instead of an autoload.

## Why an engine build at all
visionOS hand-tracking support is itself an engine capability (see
[Clancey/godot `visionos_master_pr`](https://github.com/Clancey/godot/tree/visionos_master_pr)).
This bridge is built on top of that fork's `modules/visionos_xr`.

## Add it
1. Drop `simhands_bridge.mm` into `modules/visionos_xr/` of a Clancey Godot visionOS checkout.
2. Call `apply_simhands_hand_states()` once per frame in the XR interface's `process()` —
   *between* the (empty-in-sim) ARKit hand update and the existing tracker-write step, so it
   fills the same `XRHandTracker` path the device uses. Reuse the fork's
   `HandInteractionState` / `HandJointState` structs.
3. Build the **simulator** slice and swap it into your app's xcframework:
   ```bash
   scons platform=visionos arch=arm64 simulator=yes target=template_debug
   cp bin/libgodot.visionos.template_debug.arm64.simulator.a \
      <your-app>/out/xcode-visionos/<App>.xcframework/xros-arm64-simulator/libgodot.a
   ```
4. Gate it: the file is `#if TARGET_OS_SIMULATOR`-only and activates at runtime only when
   `GODOT_SIMHANDS=1`, so the device slice is a byte-for-byte no-op. Launch with:
   ```bash
   SIMCTL_CHILD_GODOT_SIMHANDS=1 xcrun simctl launch --terminate-running-process <SIM_UDID> <bundle-id>
   ```

## Feed it hands
Any one of: the real [VisionOS-SimHands](https://github.com/BenLumenDigital/VisionOS-SimHands)
macOS helper (webcam), `../tools/simhands_canned_sender.swift` (canned loop, no webcam), or the
`../tools/SimControlPanel` (canned feed + on-demand pinch + live calibration). All three speak the
same MultipeerConnectivity "Bonjour" contract (21-landmark JSON), so they're interchangeable.

## Calibration
Hand placement/scale is tuned live without rebuilds: the panel sends `K*` verbs on UDP 9999 →
`simulator_input.gd` writes `user://simhands_calibration.cfg` → the bridge re-reads it ~9 Hz.
`KR` resets to compiled defaults.
