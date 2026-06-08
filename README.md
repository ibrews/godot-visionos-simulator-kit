# Godot visionOS Simulator Kit

**Develop and test Godot visionOS apps in the visionOS Simulator — with working input, hand-tracking, and a one-command sim/device build switcher.** Reusable across any Godot visionOS project; extracted from the [Cascade Countdown](https://github.com/ibrews/godot-avp-cascade) pilot by Alex Coulombe.

> A Godot visionOS app renders an **immersive, custom-Metal CompositorServices scene**. The visionOS Simulator's built-in pinch/gesture simulation only reaches RealityKit/SwiftUI apps — so a Godot immersive app gets **zero input and zero hands** in the sim out of the box, and (until you wire something up) you can only test on a physical Apple Vision Pro. This kit closes that gap so you can iterate in the Simulator, which is far faster than a device round-trip.

## What's inside

| Piece | What it does | Needs an engine rebuild? |
|-------|--------------|:---:|
| **`build.sh`** | One command to build + run in the **Simulator** or on a **device** — hides the differing `xcodebuild -destination`, code-signing, and install tool (`simctl` vs `devicectl`). | no |
| **`godot/simulator_input.gd`** | A sim-gated GDScript autoload that listens on **UDP `127.0.0.1:9999`** and injects grab / reset / gesture / hand-visibility commands into your game. Loopback ⇒ no Local Network prompt. | no |
| **`tools/SimInputTap.swift`** | A tiny macOS global key-tap that forwards keys → UDP 9999, so you keep the **Simulator focused** for look/move while actions still reach the app. | no |
| **`tools/SimControlPanel/`** | A macOS SwiftUI window with buttons / toggles / sliders driving both channels (UDP input **and** the SimHands hand feed), plus **live hand calibration**. | no |
| **`tools/simhands_canned_sender.swift`** | Streams a canned 21-landmark hand feed over MultipeerConnectivity so you can drive hands with no webcam (great for deterministic/headless testing). | no |
| **`engine/simhands_bridge.mm`** | A native engine module that consumes a [VisionOS-SimHands](https://github.com/BenLumenDigital/VisionOS-SimHands)-style MultipeerConnectivity hand stream (webcam → MediaPipe) and writes Godot's `XRHandTracker` in the sim — real articulated hands. | **yes** (Clancey engine) |

The first five are **drop-in** (no engine rebuild). The SimHands engine bridge gives you *real* webcam hands in the sim but must be compiled into a [Clancey/godot](https://github.com/Clancey/godot/tree/visionos_master_pr) visionOS engine build — see [`engine/`](engine/).

## Quickstart — the build switcher

```bash
cp build.config.example build.config      # then edit it: your Godot path, scheme, bundle id, sim UDID, team
./build.sh sim       # build + run in the visionOS Simulator (recommended for prototyping)
./build.sh device    # build signed + install on a paired Apple Vision Pro
./build.sh export    # just re-export the Godot .pck
```

Everything project-specific lives in `build.config` (or env vars), so the script is identical across projects. The engine slices for both platforms live in your app's `.xcframework` (`xros-arm64` = device, `xros-arm64-simulator` = sim) and xcodebuild auto-selects per destination — **no per-build slice swapping.**

## Things to Try

1. **Switch build targets with one word.** `./build.sh sim` then `./build.sh device` — same project, the script handles the destination, signing, and install tool for each. (Set `build.config` first.)
2. **Drive your game from the keyboard while the Simulator stays focused.** Build `tools/SimInputTap.swift` (`swiftc -O SimInputTap.swift -o siminputtap && ./siminputtap`), add `godot/simulator_input.gd` as an autoload (sim-gated), and tap your mapped keys — grab/reset/gestures reach the immersive app over UDP 9999 while you still look around with the sim's own controls.
3. **Open the SimControlPanel.** `cd tools/SimControlPanel && ./build.sh && open SimControlPanel.app` — clickable buttons/toggles/sliders for input **and** a canned hand feed, with live hand-position calibration sliders.
4. **Feed canned hands with no webcam.** `swiftc -O tools/simhands_canned_sender.swift -o sender && ./sender` streams a looping pinch over MultipeerConnectivity — deterministic hand input for tests.
5. **(Advanced) Real webcam hands in the sim.** Compile `engine/simhands_bridge.mm` into a Clancey Godot visionOS engine build, run [VisionOS-SimHands](https://github.com/BenLumenDigital/VisionOS-SimHands), launch with `GODOT_SIMHANDS=1` — your webcam hands drive `XRHandTracker` in the Simulator. See [`engine/README.md`](engine/).

## How it fits together

```
  macOS host                                   visionOS Simulator (immersive Godot app)
  ─────────────────────────                    ────────────────────────────────────────
  SimControlPanel / SimInputTap  ── UDP 9999 ──►  simulator_input.gd (autoload)  → grab/gestures
  SimControlPanel / canned sender ─ Multipeer ─►  engine simhands_bridge.mm       → XRHandTracker
  VisionOS-SimHands (webcam→MediaPipe) ─────────►  (same Multipeer "Bonjour" feed)
```

- **UDP loopback (9999)** is the no-rebuild input path: GDScript can speak `PacketPeerUDP`, so any Mac-side sender can drive your game. No Local Network permission (loopback).
- **MultipeerConnectivity** is the hand path: GDScript can't speak it, so hands need the native `engine/simhands_bridge.mm`. The SimHands feed is the same MC contract used by [VisionOS-SimHands](https://github.com/BenLumenDigital/VisionOS-SimHands) (21-landmark JSON), so the canned sender, the panel, and a real webcam are interchangeable.

## Background / credits

Why immersive Godot apps get no sim input or hands, the rendering fixes to build Godot for the visionOS sim at all, and the full design notes live in the Cascade Countdown wiki: [Build and Deploy](https://github.com/ibrews/godot-avp-cascade/wiki/Build-and-Deploy) · [visionOS Gotchas](https://github.com/ibrews/godot-avp-cascade/wiki/visionOS-Gotchas).

Built by **Alex Coulombe**. The SimHands engine bridge interoperates with **[VisionOS-SimHands](https://github.com/BenLumenDigital/VisionOS-SimHands)** by Ben Lumen Digital (webcam hand-tracking for the visionOS Simulator). visionOS hand-tracking engine support rides on **[Clancey/godot](https://github.com/Clancey/godot/tree/visionos_master_pr)** (Marshall Nowak). "Godot" is a trademark of the Godot Foundation; this kit is independent and not affiliated.

## License

MIT — see [LICENSE](LICENSE).
