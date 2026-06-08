#!/usr/bin/env bash
#
# build.sh — one-command Godot visionOS build/run switcher: SIMULATOR or DEVICE.
# Part of godot-visionos-simulator-kit. https://github.com/ibrews/godot-visionos-simulator-kit
#
#   ./build.sh sim       re-export the PCK → build for the visionOS Simulator → install + launch
#   ./build.sh device    re-export the PCK → build SIGNED for a real Apple Vision Pro → install
#   ./build.sh export    just re-export the Godot PCK (no Xcode build)
#   ./build.sh help      show this help
#
# WHY: the sim and device builds differ in three annoying ways — the xcodebuild -destination,
# code-signing, and the install tool (simctl vs devicectl). This wraps both so you never
# hand-type (or mis-type) the incantation. Switching is just the first argument.
#
# SETUP (per project): copy `build.config.example` → `build.config` next to this script and fill
# it in (or set the same names as env vars). Then `./build.sh sim`.
#
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"

# --------------------------------- CONFIG ---------------------------------
[ -f ./build.config ] && source ./build.config            # per-project overrides (see build.config.example)
GODOT="${GODOT:-godot}"                                    # path to the Godot visionOS editor binary
PROJECT_DIR="${PROJECT_DIR:-$PWD/project}"                 # Godot project dir (has project.godot)
EXPORT_PRESET="${EXPORT_PRESET:-visionOS}"                 # name="..." in export_presets.cfg
XCODE_DIR="${XCODE_DIR:-$PWD/out/xcode-visionos}"          # exported Xcode project dir
SCHEME="${SCHEME:-MyVisionApp}"                            # Xcode scheme == the .xcodeproj name (no ext)
PCK_OUT="${PCK_OUT:-$XCODE_DIR/$SCHEME.pck}"               # PCK output (absolute!)
XCODEPROJ="${XCODEPROJ:-$XCODE_DIR/$SCHEME.xcodeproj}"
BUNDLE_ID="${BUNDLE_ID:-com.example.myvisionapp}"
SIM_UDID="${SIM_UDID:-}"                                   # xcrun simctl list devices  (Apple Vision Pro)
DEVICE_ID="${DEVICE_ID:-}"                                 # xcrun devicectl list devices
DEV_TEAM="${DEV_TEAM:-}"                                   # Apple Developer team id (device signing)
# Engine slices live in the .xcframework (xros-arm64 = device, xros-arm64-simulator = sim);
# xcodebuild auto-selects per -destination, so no per-build slice swap is needed.
# --------------------------------------------------------------------------

c() { printf '\033[1;36m▶ %s\033[0m\n' "$*"; }
die() { printf '\033[1;31m✗ %s\033[0m\n' "$*" >&2; exit 1; }

export_pck() {
  command -v "$GODOT" >/dev/null || [ -x "$GODOT" ] || die "GODOT not found: '$GODOT' — set it in build.config"
  c "Export PCK ($EXPORT_PRESET) → $PCK_OUT"
  "$GODOT" --headless --path "$PROJECT_DIR" --export-pack "$EXPORT_PRESET" "$PCK_OUT"
}

find_app() {  # $1 = Debug-xrsimulator | Debug-xros
  ls -dt "$HOME/Library/Developer/Xcode/DerivedData/${SCHEME}-"*"/Build/Products/$1/${SCHEME}.app" 2>/dev/null | head -1
}

build_sim() {
  [ -n "$SIM_UDID" ] || die "SIM_UDID is empty — set it in build.config (xcrun simctl list devices)"
  export_pck
  c "xcodebuild → visionOS Simulator ($SIM_UDID)"
  xcodebuild -project "$XCODEPROJ" -scheme "$SCHEME" -configuration Debug \
    -destination "platform=visionOS Simulator,id=$SIM_UDID" \
    CODE_SIGNING_ALLOWED=NO build
  local app; app="$(find_app Debug-xrsimulator)"; [ -n "$app" ] || die ".app not found under DerivedData"
  c "Boot sim + install + launch: $app"
  xcrun simctl boot "$SIM_UDID" 2>/dev/null || true
  open -a Simulator
  xcrun simctl install "$SIM_UDID" "$app"
  xcrun simctl launch "$SIM_UDID" "$BUNDLE_ID"
  c "Running in the Simulator. Keep it FRONTMOST — immersive apps suspend when backgrounded."
}

build_device() {
  { [ -n "$DEVICE_ID" ] && [ -n "$DEV_TEAM" ]; } || die "DEVICE_ID and DEV_TEAM must be set in build.config for device builds"
  export_pck
  c "xcodebuild → Apple Vision Pro device ($DEVICE_ID), team $DEV_TEAM"
  xcodebuild -project "$XCODEPROJ" -scheme "$SCHEME" -configuration Debug \
    -destination "platform=visionOS,id=$DEVICE_ID" \
    CODE_SIGN_IDENTITY="Apple Development" DEVELOPMENT_TEAM="$DEV_TEAM" build
  local app; app="$(find_app Debug-xros)"; [ -n "$app" ] || die ".app not found under DerivedData"
  c "Install on device: $app"
  xcrun devicectl device install app --device "$DEVICE_ID" "$app"
  c "Installed. Open the app on the headset by hand (visionOS has no remote launch for immersive apps)."
}

case "${1:-help}" in
  sim)     build_sim ;;
  device)  build_device ;;
  export)  export_pck ;;
  *)       sed -n '4,18p' "$0" | sed 's/^#\s\{0,1\}//' ;;
esac
