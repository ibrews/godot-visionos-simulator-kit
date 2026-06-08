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

## Hand quality & behavior
The bridge synthesizes a full articulated hand from MediaPipe's 21 landmarks:
- **Joint orientations** are synthesized (each bone's `+Y` toward its child joint; palm normal sets
  the roll) so the skinned hand mesh follows the fingers instead of twisting into a claw.
- **Finger metacarpals** (which MediaPipe doesn't provide) are synthesized so the palm skin doesn't
  stretch under the fingers; the **left hand's** mirrored chirality is corrected.
- **WASD-follow:** hands translate with the head's *position* as you navigate the sim (position-only —
  the head *rotation* in the sim doesn't match the rendered camera, so it isn't applied).
- **Out-of-frame guard:** when a fingertip leaves the webcam frame, MediaPipe emits a garbage landmark
  that would stretch the finger into a tendril; the bridge detects the bad bone (too long vs the hand
  scale or vs its own last-good, or non-finite) and holds the last-good bone instead.

Notes: the SimHands macOS helper's WKWebView **pauses MediaPipe when its window isn't frontmost**
(macOS App Nap) — keep it visible; and only **one** "Bonjour" feed should run at a time (multiple
senders fight frame-by-frame).

## Calibration
Hand placement/scale is tuned live without rebuilds: the panel sends `K*` verbs on UDP 9999 →
`simulator_input.gd` writes `user://simhands_calibration.cfg` → the bridge re-reads it ~9 Hz.
`KR` resets to compiled defaults.

## Credit
The MultipeerConnectivity hand-stream approach is from
[VisionOS-SimHands](https://github.com/BenLumenDigital/VisionOS-SimHands) by Ben Harraway (Apache-2.0).
This bridge is an independent re-implementation of that protocol in the engine — see [`../NOTICE`](../NOTICE).
