/**************************************************************************/
/*  simhands_bridge.mm                                                    */
/**************************************************************************/
/*  SIMULATOR-ONLY dev/test bridge: ingest VisionOS-SimHands MediaPipe    */
/*  hand-tracking over MultipeerConnectivity and feed Godot's             */
/*  XRHandTracker, so a custom-Metal immersive Godot app gets articulated */
/*  grab/pinch input in the visionOS Simulator (which delivers no ARKit   */
/*  hand anchors). Completely inert on device and when GODOT_SIMHANDS is  */
/*  unset. On real Apple Vision Pro, ARKit hand tracking already works —  */
/*  this path must never run there.                                       */
/*                                                                        */
/*  Mechanism: SimHands' macOS helper advertises an MCSession over the    */
/*  serviceType "Bonjour" and broadcasts JSON                             */
/*    {"landmarks":[[{x,y,z}x21],...],                                    */
/*     "handednesses":[[{"displayName":"Left"|"Right"}],...]}             */
/*  We are a native MultipeerConnectivity *client* that connects, parses  */
/*  that stream, and fills hand_interaction_states[] each frame. The      */
/*  existing apply_hand_states_to_trackers() then writes the trackers     */
/*  exactly as it does for real ARKit data — so the whole tracker-write   */
/*  path is reused verbatim.                                              */
/**************************************************************************/

#ifdef VISIONOS_ENABLED

#include "visionos_xr_interface.h"

#import <TargetConditionals.h>

#if TARGET_OS_SIMULATOR

#include "core/io/config_file.h"
#include "core/math/math_funcs.h"
#include "core/os/os.h"

#import <Foundation/Foundation.h>
#import <MultipeerConnectivity/MultipeerConnectivity.h>
#import <os/lock.h>
#import <os/log.h>

// ---------------------------------------------------------------------------
// Tunables (first-light defaults; exact calibration is human-in-the-loop —
// see REVIEW_NEEDED.md). All metric (meters), all in head-relative space; the
// engine's apply_floor_offset() then lifts y by eye_height (~1.7 roomscale),
// so a wrist y of -0.10 lands ~eye level in front of the XR origin.
// ---------------------------------------------------------------------------
// Compiled-in first-light defaults. The live values below are tuned at RUNTIME from
// user://simhands_calibration.cfg (written by simulator_input.gd on UDP "K*" verbs from the
// SimControlPanel macOS dev tool), so placement/scale can be dialed in while watching the sim
// with no rebuild per tweak. Deleting the cfg (panel "Reset") restores these.
static const float SIMHANDS_DEF_KNUCKLE_M = 0.09f;
static const float SIMHANDS_DEF_PLANE_M = 0.55f;
static const float SIMHANDS_DEF_DEPTH_M = 0.45f;
static const float SIMHANDS_DEF_Y_OFFSET_M = -0.10f;
static const float SIMHANDS_DEF_Z_SHAPE_GAIN = 1.0f;
static const float SIMHANDS_DEF_SMOOTHING = 0.9f;

static float SIMHANDS_HAND_KNUCKLE_M = SIMHANDS_DEF_KNUCKLE_M; // target wrist->middle-knuckle length (self-normalize → metric hand)
static float SIMHANDS_PLANE_M = SIMHANDS_DEF_PLANE_M; // image plane → head-space placement scale (hand x,y in ~±0.275 m)
static float SIMHANDS_DEPTH_M = SIMHANDS_DEF_DEPTH_M; // wrist distance in front of the XR origin
static float SIMHANDS_Y_OFFSET_M = SIMHANDS_DEF_Y_OFFSET_M; // head-relative vertical offset before floor offset
static float SIMHANDS_Z_SHAPE_GAIN = SIMHANDS_DEF_Z_SHAPE_GAIN; // how much MediaPipe z drives finger-curl depth
static float SIMHANDS_SMOOTHING = SIMHANDS_DEF_SMOOTHING; // hand stabilizer: 0 = raw (no smoothing), 1 = max; live-tunable via the panel "smoothing" slider

// Pinch thresholds for the controller "trigger" proxy (local copy of the
// engine's VISIONOS_PINCH_* — the game itself reads the hand-tracker joint
// distance directly, so these only drive the controller-tracker trigger).
static const float SIMHANDS_PINCH_PRESS_M = 0.025f;
static const float SIMHANDS_PINCH_ANALOG_MAX_M = 0.06f;

// MediaPipe landmark index (0..20) → Godot XRHandTracker joint. MediaPipe has
// no separate finger metacarpals; those Godot joints stay untracked (the mesh
// driver skips them). The fingertips + wrist — all the game's pinch logic
// needs — map exactly (wrist=1, thumb_tip=5, index_tip=10, ...).
static const XRHandTracker::HandJoint SIMHANDS_JOINT_MAP[21] = {
	XRHandTracker::HAND_JOINT_WRIST, // 0  wrist
	XRHandTracker::HAND_JOINT_THUMB_METACARPAL, // 1  thumb CMC
	XRHandTracker::HAND_JOINT_THUMB_PHALANX_PROXIMAL, // 2  thumb MCP
	XRHandTracker::HAND_JOINT_THUMB_PHALANX_DISTAL, // 3  thumb IP
	XRHandTracker::HAND_JOINT_THUMB_TIP, // 4  thumb tip
	XRHandTracker::HAND_JOINT_INDEX_FINGER_PHALANX_PROXIMAL, // 5  index MCP (knuckle)
	XRHandTracker::HAND_JOINT_INDEX_FINGER_PHALANX_INTERMEDIATE, // 6  index PIP
	XRHandTracker::HAND_JOINT_INDEX_FINGER_PHALANX_DISTAL, // 7  index DIP
	XRHandTracker::HAND_JOINT_INDEX_FINGER_TIP, // 8  index tip
	XRHandTracker::HAND_JOINT_MIDDLE_FINGER_PHALANX_PROXIMAL, // 9  middle MCP (knuckle)
	XRHandTracker::HAND_JOINT_MIDDLE_FINGER_PHALANX_INTERMEDIATE, // 10 middle PIP
	XRHandTracker::HAND_JOINT_MIDDLE_FINGER_PHALANX_DISTAL, // 11 middle DIP
	XRHandTracker::HAND_JOINT_MIDDLE_FINGER_TIP, // 12 middle tip
	XRHandTracker::HAND_JOINT_RING_FINGER_PHALANX_PROXIMAL, // 13 ring MCP
	XRHandTracker::HAND_JOINT_RING_FINGER_PHALANX_INTERMEDIATE, // 14 ring PIP
	XRHandTracker::HAND_JOINT_RING_FINGER_PHALANX_DISTAL, // 15 ring DIP
	XRHandTracker::HAND_JOINT_RING_FINGER_TIP, // 16 ring tip
	XRHandTracker::HAND_JOINT_PINKY_FINGER_PHALANX_PROXIMAL, // 17 pinky MCP
	XRHandTracker::HAND_JOINT_PINKY_FINGER_PHALANX_INTERMEDIATE, // 18 pinky PIP
	XRHandTracker::HAND_JOINT_PINKY_FINGER_PHALANX_DISTAL, // 19 pinky DIP
	XRHandTracker::HAND_JOINT_PINKY_FINGER_TIP, // 20 pinky tip
};

// Per-joint bone direction = position[DIR_TO] - position[DIR_FROM] (MediaPipe indices).
// For finger/thumb segments this points toward the child joint; for the 5 fingertips it
// continues the parent segment's direction; the wrist points up the hand (→ middle knuckle).
// Used to synthesize joint ORIENTATION (the mesh skins from each joint's basis; without it,
// correctly-placed bones twist the skin → the "distorted claw"). Godot's humanoid-rig
// convention (what hand_mesh_driver's _bone_correction is tuned for) = bone's local +Y points
// distally, so we put +Y along this direction.
static const int SIMHANDS_DIR_FROM[21] = { 0, 1, 2, 3, 3, 5, 6, 7, 7, 9, 10, 11, 11, 13, 14, 15, 15, 17, 18, 19, 19 };
static const int SIMHANDS_DIR_TO[21] = { 9, 2, 3, 4, 4, 6, 7, 8, 8, 10, 11, 12, 12, 14, 15, 16, 16, 18, 19, 20, 20 };

// Parent landmark of each MediaPipe joint along its finger chain (wrist=0 = root). Used by the
// off-frame gating in apply_simhands_hand_states: index < li for every entry, so when a landmark
// that has never yet been seen in-frame must fall back onto its parent, the parent's gated position
// is already resolved (we walk li = 0..20 in order).
static const int SIMHANDS_PARENT[21] = { -1, 0, 1, 2, 3, 0, 5, 6, 7, 0, 9, 10, 11, 0, 13, 14, 15, 0, 17, 18, 19 };

// ---------------------------------------------------------------------------
// Hand-feed source selection (multi-tool coexistence).
// ---------------------------------------------------------------------------
// The bridge auto-connects to EVERY advertised "Bonjour" peer — the real VisionOS-SimHands helper,
// the SimControlPanel canned feed, the canned-sender CLI, even stray host advertisers — and they all
// land in one _latest (last-write-wins). With >1 peer streaming hands, their frames fight frame-by-
// frame (observed jitter between the real and canned poses). So exactly ONE peer may drive at a time:
//   (1) a time-based single-source LOCK — the first peer to send a non-empty hand wins and holds the
//       lock until it goes quiet (no data for SIMHANDS_LOCK_TIMEOUT_S), then a new winner may claim it;
//   (2) an optional FORCED source — the SimControlPanel "Hand source" picker sends a UDP "H" verb
//       (H0/H1/H2) → simulator_input.gd → user://simhands_calibration.cfg "source" → here, restricting
//       which peers may win. AUTO (no cfg / no panel) keeps the zero-config behavior: any peer wins.
enum SimHandsSource {
	SIMHANDS_SRC_AUTO = -1, // no restriction — first non-empty peer wins (default; zero-config)
	SIMHANDS_SRC_OFF = 0, // ignore all feeds — hands hidden
	SIMHANDS_SRC_CANNED = 1, // only the local canned feeds (SimControlPanel / SimHandsCanned)
	SIMHANDS_SRC_WEBCAM = 2, // only the real helper (any peer that is NOT a known canned feed)
};
static const NSTimeInterval SIMHANDS_LOCK_TIMEOUT_S = 1.0; // locked peer "quiet" this long → release

// Known local/deterministic canned-feed peer names. "Canned" accepts exactly these; "Webcam" accepts
// anything else (the real VisionOS-SimHands helper, whatever host-derived name it advertises).
static bool simhands_is_canned_peer(NSString *name) {
	return [name isEqualToString:@"SimControlPanel"] || [name isEqualToString:@"SimHandsCanned"];
}

// Live forced-source, mirrored from the cfg by simhands_reload_calibration() and pushed to the MC
// client each frame by apply_simhands_hand_states(). AUTO until the panel says otherwise.
static int simhands_forced_source = SIMHANDS_SRC_AUTO;

// Plain-old-data carried from the MC delegate queue → the engine main thread.
struct SimHandsLandmark {
	float x = 0.0f;
	float y = 0.0f;
	float z = 0.0f;
};
struct SimHandsPodHand {
	bool present = false;
	int side = -1; // 0 = Godot left, 1 = Godot right (after handedness swap)
	SimHandsLandmark lm[21];
};
struct SimHandsPodFrame {
	SimHandsPodHand hands[2];
	uint32_t hand_count = 0;
	uint64_t seq = 0;
};

static os_log_t simhands_log() {
	static os_log_t log = os_log_create("com.agilelens.godotvisionpilot", "simhands");
	return log;
}

// Build a joint orientation: local +Y points along p_distal (toward the child joint), with
// p_ref_up (the palm normal) resolving the roll about the bone. Right-handed; falls back to an
// arbitrary perpendicular if p_ref_up is parallel to the bone. Matches the humanoid-rig
// convention hand_mesh_driver expects (it then applies its per-hand _bone_correction Y-twist).
static Basis simhands_basis_from_dir(const Vector3 &p_distal, const Vector3 &p_ref_up) {
	Vector3 y = p_distal;
	const float yl = y.length();
	if (yl < 1e-5f) {
		return Basis();
	}
	y /= yl;
	Vector3 z = p_ref_up - y * p_ref_up.dot(y); // component of ref ⟂ to the bone
	if (z.length() < 1e-5f) {
		z = y.cross(Vector3(1, 0, 0));
		if (z.length() < 1e-5f) {
			z = y.cross(Vector3(0, 0, 1));
		}
	}
	z = z.normalized();
	Vector3 x = y.cross(z).normalized();
	z = x.cross(y).normalized(); // re-orthogonalize, keep right-handed
	Basis b;
	b.set_column(0, x);
	b.set_column(1, y);
	b.set_column(2, z);
	return b;
}

// ---------------------------------------------------------------------------
// Native MultipeerConnectivity client. Mirrors SimHands' BonjourSession with
// usage = .combined: it both advertises and browses for the "Bonjour" service,
// auto-invites discovered peers, and auto-accepts invitations — so it connects
// to SimHands' helper (or our canned sender) regardless of which side initiates.
// JSON parsing runs on MC's delegate queue; the parsed frame is handed to the
// engine thread under an os_unfair_lock.
// ---------------------------------------------------------------------------
@interface GodotSimHandsClient : NSObject <MCSessionDelegate, MCNearbyServiceBrowserDelegate, MCNearbyServiceAdvertiserDelegate> {
	MCPeerID *_peerID;
	MCSession *_session;
	MCNearbyServiceBrowser *_browser;
	MCNearbyServiceAdvertiser *_advertiser;
	NSString *_serviceType;
	os_unfair_lock _lock;
	SimHandsPodFrame _latest;
	BOOL _hasFrame;
	// Single-source lock + forced-source filter (multi-feed coexistence). All guarded by _lock.
	NSString *_lockedPeer; // displayName of the peer currently driving the trackers; nil = unlocked
	NSTimeInterval _lastDataTime; // systemUptime of the last ACCEPTED frame (drives lock liveness)
	int _forcedSource; // SimHandsSource: AUTO/-1 (any) · OFF/0 · CANNED/1 · WEBCAM/2
}
- (instancetype)initWithService:(NSString *)serviceType;
- (void)start;
- (void)stop;
- (BOOL)copyLatestFrame:(SimHandsPodFrame *)outFrame;
- (void)setForcedSource:(int)src; // restrict which peers may win the lock (from the panel picker)
- (NSString *)lockedPeerName; // displayName of the current winner (diagnostics); nil if none
@end

@implementation GodotSimHandsClient

- (instancetype)initWithService:(NSString *)serviceType {
	self = [super init];
	if (self) {
		_serviceType = serviceType;
		_lock = OS_UNFAIR_LOCK_INIT;
		_hasFrame = NO;
		_lockedPeer = nil;
		_lastDataTime = 0;
		_forcedSource = SIMHANDS_SRC_AUTO;
		_peerID = [[MCPeerID alloc] initWithDisplayName:@"GodotVisionPilot"];
		_session = [[MCSession alloc] initWithPeer:_peerID securityIdentity:nil encryptionPreference:MCEncryptionNone];
		_session.delegate = self;
		_browser = [[MCNearbyServiceBrowser alloc] initWithPeer:_peerID serviceType:_serviceType];
		_browser.delegate = self;
		_advertiser = [[MCNearbyServiceAdvertiser alloc] initWithPeer:_peerID discoveryInfo:nil serviceType:_serviceType];
		_advertiser.delegate = self;
	}
	return self;
}

- (void)start {
	[_advertiser startAdvertisingPeer];
	[_browser startBrowsingForPeers];
	os_log(simhands_log(), "[SimHands] client started: advertising + browsing service '%{public}@'", _serviceType);
}

- (void)stop {
	[_advertiser stopAdvertisingPeer];
	[_browser stopBrowsingForPeers];
	[_session disconnect];
	os_log(simhands_log(), "[SimHands] client stopped");
}

- (BOOL)copyLatestFrame:(SimHandsPodFrame *)outFrame {
	BOOL ok = NO;
	os_unfair_lock_lock(&_lock);
	if (_hasFrame) {
		*outFrame = _latest;
		ok = YES;
	}
	os_unfair_lock_unlock(&_lock);
	return ok;
}

// Restrict which peers may win the single-source lock (SimControlPanel "Hand source" picker). A
// CHANGE drops the current lock so the newly-selected source can take over immediately (instant
// hand-off) instead of waiting out the quiet timeout.
- (void)setForcedSource:(int)src {
	os_unfair_lock_lock(&_lock);
	if (src != _forcedSource) {
		_forcedSource = src;
		_lockedPeer = nil;
		// Drop the retained frame too: otherwise switching to a source with no active feed (e.g.
		// Webcam with no helper running) would keep re-applying the OLD source's last frame — a stale
		// frozen hand. Cleared, apply_simhands_hand_states() finds no data and the per-frame ARKit
		// reset leaves the hands untracked (gone) until the newly-selected source streams.
		_hasFrame = NO;
	}
	os_unfair_lock_unlock(&_lock);
}

- (NSString *)lockedPeerName {
	os_unfair_lock_lock(&_lock);
	NSString *n = _lockedPeer ? [_lockedPeer copy] : nil;
	os_unfair_lock_unlock(&_lock);
	return n;
}

// --- MCNearbyServiceBrowserDelegate ---
- (void)browser:(MCNearbyServiceBrowser *)browser foundPeer:(MCPeerID *)peerID withDiscoveryInfo:(NSDictionary<NSString *, NSString *> *)info {
	os_log(simhands_log(), "[SimHands] found peer '%{public}@' — inviting", peerID.displayName);
	[browser invitePeer:peerID toSession:_session withContext:nil timeout:10.0];
}

- (void)browser:(MCNearbyServiceBrowser *)browser lostPeer:(MCPeerID *)peerID {
	os_log(simhands_log(), "[SimHands] lost peer '%{public}@'", peerID.displayName);
}

- (void)browser:(MCNearbyServiceBrowser *)browser didNotStartBrowsingForPeers:(NSError *)error {
	os_log_error(simhands_log(), "[SimHands] browse failed: %{public}@", error.localizedDescription);
}

// --- MCNearbyServiceAdvertiserDelegate ---
- (void)advertiser:(MCNearbyServiceAdvertiser *)advertiser didReceiveInvitationFromPeer:(MCPeerID *)peerID withContext:(NSData *)context invitationHandler:(void (^)(BOOL, MCSession *_Nullable))invitationHandler {
	os_log(simhands_log(), "[SimHands] invitation from '%{public}@' — accepting", peerID.displayName);
	invitationHandler(YES, _session);
}

- (void)advertiser:(MCNearbyServiceAdvertiser *)advertiser didNotStartAdvertisingPeer:(NSError *)error {
	os_log_error(simhands_log(), "[SimHands] advertise failed: %{public}@", error.localizedDescription);
}

// --- MCSessionDelegate ---
- (void)session:(MCSession *)session peer:(MCPeerID *)peerID didChangeState:(MCSessionState)state {
	const char *s = (state == MCSessionStateConnected) ? "connected" : (state == MCSessionStateConnecting) ? "connecting"
																									  : "notConnected";
	os_log(simhands_log(), "[SimHands] peer '%{public}@' → %{public}s", peerID.displayName, s);
	// If the winning peer drops, release the lock now so another feed can take over without waiting
	// out the quiet timeout.
	if (state == MCSessionStateNotConnected) {
		os_unfair_lock_lock(&_lock);
		if (_lockedPeer != nil && [_lockedPeer isEqualToString:peerID.displayName]) {
			_lockedPeer = nil;
		}
		os_unfair_lock_unlock(&_lock);
	}
}

- (void)session:(MCSession *)session didReceiveData:(NSData *)data fromPeer:(MCPeerID *)peerID {
	NSError *err = nil;
	id obj = [NSJSONSerialization JSONObjectWithData:data options:0 error:&err];
	if (![obj isKindOfClass:[NSDictionary class]]) {
		return;
	}
	NSDictionary *root = (NSDictionary *)obj;
	NSArray *landmarks = root[@"landmarks"];
	NSArray *handed = root[@"handednesses"];
	if (![landmarks isKindOfClass:[NSArray class]]) {
		return;
	}

	SimHandsPodFrame frame;
	NSUInteger nhands = MIN((NSUInteger)2, landmarks.count);
	for (NSUInteger h = 0; h < nhands; h++) {
		NSArray *joints = landmarks[h];
		if (![joints isKindOfClass:[NSArray class]]) {
			continue;
		}
		SimHandsPodHand &hand = frame.hands[h];
		hand.present = true;
		hand.side = -1;
		NSUInteger njoints = MIN((NSUInteger)21, joints.count);
		for (NSUInteger j = 0; j < njoints; j++) {
			id jd = joints[j];
			if (![jd isKindOfClass:[NSDictionary class]]) {
				continue;
			}
			hand.lm[j].x = [((NSDictionary *)jd)[@"x"] floatValue];
			hand.lm[j].y = [((NSDictionary *)jd)[@"y"] floatValue];
			hand.lm[j].z = [((NSDictionary *)jd)[@"z"] floatValue];
		}

		// Handedness: MediaPipe reports L/R swapped; SimHands swaps it back, so
		// displayName "Left" → Godot RIGHT tracker, "Right" → Godot LEFT.
		if ([handed isKindOfClass:[NSArray class]] && h < handed.count) {
			id hi = handed[h];
			if ([hi isKindOfClass:[NSArray class]] && [(NSArray *)hi count] > 0) {
				id info = ((NSArray *)hi)[0];
				if ([info isKindOfClass:[NSDictionary class]]) {
					NSString *name = ((NSDictionary *)info)[@"displayName"];
					if ([name isKindOfClass:[NSString class]]) {
						if ([name isEqualToString:@"Left"]) {
							hand.side = 1;
						} else if ([name isEqualToString:@"Right"]) {
							hand.side = 0;
						}
					}
				}
			}
		}
		if (hand.side < 0) {
			hand.side = (int)h; // fallback: detection order (0→left, 1→right)
		}
	}
	// Disambiguate if both hands resolved to the same side.
	if (nhands == 2 && frame.hands[0].present && frame.hands[1].present && frame.hands[0].side == frame.hands[1].side) {
		frame.hands[1].side = 1 - frame.hands[0].side;
	}
	frame.hand_count = (uint32_t)nhands;

	// Single-source lock + forced-source filter (see SimHandsSource above). Only ONE peer drives the
	// trackers at a time, so concurrent feeds (real helper + panel + canned sender) can't fight.
	const bool has_hand = (frame.hand_count > 0 && frame.hands[0].present);
	NSString *peer = peerID.displayName;
	const NSTimeInterval now = NSProcessInfo.processInfo.systemUptime;

	os_unfair_lock_lock(&_lock);
	bool source_ok; // does this peer match the panel's forced selection?
	switch (_forcedSource) {
		case SIMHANDS_SRC_OFF:
			source_ok = false;
			break;
		case SIMHANDS_SRC_CANNED:
			source_ok = simhands_is_canned_peer(peer);
			break;
		case SIMHANDS_SRC_WEBCAM:
			source_ok = !simhands_is_canned_peer(peer);
			break;
		default: // AUTO
			source_ok = true;
			break;
	}
	// Free if nobody holds the lock or the holder has gone quiet past the timeout.
	const bool lock_free = (_lockedPeer == nil) || ((now - _lastDataTime) > SIMHANDS_LOCK_TIMEOUT_S);
	bool accept = false;
	if (source_ok) {
		if (_lockedPeer != nil && [_lockedPeer isEqualToString:peer]) {
			accept = true; // already the winner — keep it (an empty frame still keeps the lock alive)
		} else if (lock_free && has_hand) {
			_lockedPeer = peer; // claim — only a non-empty frame may take the lock
			accept = true;
		}
	}
	if (accept) {
		_lastDataTime = now;
		frame.seq = _latest.seq + 1;
		_latest = frame;
		_hasFrame = YES;
	}
	os_unfair_lock_unlock(&_lock);
}

// Required MCSessionDelegate methods we don't use.
- (void)session:(MCSession *)session didReceiveStream:(NSInputStream *)stream withName:(NSString *)streamName fromPeer:(MCPeerID *)peerID {}
- (void)session:(MCSession *)session didStartReceivingResourceWithName:(NSString *)resourceName fromPeer:(MCPeerID *)peerID withProgress:(NSProgress *)progress {}
- (void)session:(MCSession *)session didFinishReceivingResourceWithName:(NSString *)resourceName fromPeer:(MCPeerID *)peerID atURL:(NSURL *)localURL withError:(NSError *)error {}
- (void)session:(MCSession *)session didReceiveCertificate:(NSArray *)certificate fromPeer:(MCPeerID *)peerID certificateHandler:(void (^)(BOOL))certificateHandler {
	certificateHandler(YES); // encryption is .none (matches SimHands) — accept.
}

@end

// ===========================================================================
// VisionOSXRInterface members (defined here to keep the MC/ObjC client isolated
// from the main interface translation unit).
// ===========================================================================

void VisionOSXRInterface::simhands_initialize() {
	simhands_enabled = false;
	simhands_client = nullptr;

	OS *os = OS::get_singleton();
	if (os == nullptr || !os->has_environment("GODOT_SIMHANDS")) {
		return;
	}
	const String value = os->get_environment("GODOT_SIMHANDS");
	if (value.is_empty() || value == "0") {
		return;
	}

	GodotSimHandsClient *client = [[GodotSimHandsClient alloc] initWithService:@"Bonjour"];
	[client start];
	simhands_client = (__bridge_retained void *)client;
	simhands_enabled = true;
	os_log(simhands_log(), "[SimHands] ENABLED via GODOT_SIMHANDS — simulator hand bridge active");
}

void VisionOSXRInterface::simhands_uninitialize() {
	if (simhands_client != nullptr) {
		GodotSimHandsClient *client = (__bridge_transfer GodotSimHandsClient *)simhands_client;
		[client stop];
		simhands_client = nullptr;
		client = nil;
	}
	simhands_enabled = false;
}

// Re-read live calibration from user://simhands_calibration.cfg, throttled (~every 10 frames
// — slider tuning needs a few Hz, not 90). simulator_input.gd writes this file on UDP "K*"
// verbs from the SimControlPanel dev tool. Missing file (panel "Reset") → compiled defaults;
// a parse error (reader caught a mid-write) keeps the last good values and retries next cycle.
static void simhands_reload_calibration() {
	static uint32_t tick = 0;
	if ((tick++ % 10) != 0) {
		return;
	}
	Ref<ConfigFile> cf;
	cf.instantiate();
	const Error err = cf->load("user://simhands_calibration.cfg");
	if (err == ERR_FILE_NOT_FOUND || err == ERR_FILE_CANT_OPEN) {
		SIMHANDS_HAND_KNUCKLE_M = SIMHANDS_DEF_KNUCKLE_M;
		SIMHANDS_PLANE_M = SIMHANDS_DEF_PLANE_M;
		SIMHANDS_DEPTH_M = SIMHANDS_DEF_DEPTH_M;
		SIMHANDS_Y_OFFSET_M = SIMHANDS_DEF_Y_OFFSET_M;
		SIMHANDS_Z_SHAPE_GAIN = SIMHANDS_DEF_Z_SHAPE_GAIN;
		SIMHANDS_SMOOTHING = SIMHANDS_DEF_SMOOTHING;
		simhands_forced_source = SIMHANDS_SRC_AUTO; // no panel / "Reset" → any feed may drive
		return;
	}
	if (err != OK) {
		return; // mid-write/corrupt — keep last good values, retry next cycle
	}
	SIMHANDS_HAND_KNUCKLE_M = (float)(double)cf->get_value("simhands", "hand_scale", (double)SIMHANDS_DEF_KNUCKLE_M);
	SIMHANDS_PLANE_M = (float)(double)cf->get_value("simhands", "plane", (double)SIMHANDS_DEF_PLANE_M);
	SIMHANDS_DEPTH_M = (float)(double)cf->get_value("simhands", "depth", (double)SIMHANDS_DEF_DEPTH_M);
	SIMHANDS_Y_OFFSET_M = (float)(double)cf->get_value("simhands", "y_offset", (double)SIMHANDS_DEF_Y_OFFSET_M);
	SIMHANDS_Z_SHAPE_GAIN = (float)(double)cf->get_value("simhands", "z_gain", (double)SIMHANDS_DEF_Z_SHAPE_GAIN);
	SIMHANDS_SMOOTHING = (float)(double)cf->get_value("simhands", "smoothing", (double)SIMHANDS_DEF_SMOOTHING);
	simhands_forced_source = (int)(int64_t)cf->get_value("simhands", "source", (int64_t)SIMHANDS_SRC_AUTO);
}

void VisionOSXRInterface::apply_simhands_hand_states() {
	if (!simhands_enabled || simhands_client == nullptr) {
		return;
	}

	simhands_reload_calibration();

	GodotSimHandsClient *client = (__bridge GodotSimHandsClient *)simhands_client;
	[client setForcedSource:simhands_forced_source]; // panel picker → restrict which peer drives

	// Always-on source/lock diagnostic (~every 60 frames) so EVERY mode is verifiable — including the
	// no-hands modes (Off, or Webcam with no helper) that return before the per-hand log below.
	static uint32_t src_log = 0;
	if ((src_log++ % 60) == 0) {
		NSString *locked = [client lockedPeerName];
		os_log(simhands_log(), "[SimHands] source=%d locked=%{public}@", simhands_forced_source, locked ? locked : @"(none)");
	}

	// Forced "Off": hide hands regardless of what's still advertising/streaming (the picker's Off
	// must mean off even if a webcam helper or canned sender is connected).
	if (simhands_forced_source == SIMHANDS_SRC_OFF) {
		for (int i = 0; i < HAND_INDEX_MAX; i++) {
			reset_hand_state((HandIndex)i);
		}
		return;
	}

	SimHandsPodFrame frame;
	if (![client copyLatestFrame:&frame]) {
		return; // no data received yet — leave the (empty) ARKit-derived state untouched
	}

	// SimHands' mirror/center mapping: normalized image coords → a head-space
	// direction. The constant 0.5 offsets cancel in (u_i - u_0), so relative
	// joint shape is pure landmark geometry; absolute placement uses u_0.
	auto u_of = [](const SimHandsLandmark &lm) -> Vector3 {
		return Vector3(0.5f - lm.x, 0.5f - lm.y, lm.z - 0.5f);
	};

	// WASD-follow: translate the hands by how far the head has MOVED since the first frame, so
	// navigation carries them along and you can reach past the start position. POSITION-ONLY — we do
	// NOT apply the head's rotation: the sim's head_tracker orientation doesn't match the rendered
	// camera, so full head-anchoring threw the hands off-axis. Translation tracks WASD cleanly, and
	// look-rotation (which barely moves the head position) leaves the hands put — no tilt artifacts.
	static Vector3 simhands_head_baseline;
	static bool simhands_baseline_set = false;
	Vector3 head_delta;
	if (head_tracker.is_valid()) {
		Ref<XRPose> hp = head_tracker->get_pose(SNAME("default"));
		if (hp.is_valid()) {
			const Vector3 head_pos = hp->get_transform().origin;
			if (!simhands_baseline_set) {
				simhands_head_baseline = head_pos;
				simhands_baseline_set = true;
			}
			head_delta = head_pos - simhands_head_baseline;
			// The sim head pose is noisy/drifty (observed ~1 m z jump with no real movement), which
			// shoves the hands behind the viewer and out of sight. Bound the WASD-follow so the hands
			// can never leave view; ample for real reach, small enough they stay in front.
			const float HD_MAX = 0.35f;
			head_delta.x = CLAMP(head_delta.x, -HD_MAX, HD_MAX);
			head_delta.y = CLAMP(head_delta.y, -HD_MAX, HD_MAX);
			head_delta.z = CLAMP(head_delta.z, -HD_MAX, HD_MAX);
		}
	}

	bool filled[HAND_INDEX_MAX] = { false, false };

	// Off-frame gating + smoothing state, per side:
	//  - uhold/uok: each landmark's last IN-FRAME wrist-relative u-offset (held when it goes off-frame)
	//  - wrist_hold/wrist_ok: the wrist anchor's last in-frame u (placement stays put if the wrist clips)
	//  - smooth/smooth_ok: previous smoothed metric positions for the velocity-adaptive EMA
	//  - held/maxfinger: diagnostics surfaced in the os_log line (held landmarks; max cumulative finger)
	static Vector3 simhands_uhold[HAND_INDEX_MAX][21];
	static bool simhands_uok[HAND_INDEX_MAX][21] = {};
	static Vector3 simhands_wrist_hold[HAND_INDEX_MAX];
	static bool simhands_wrist_ok[HAND_INDEX_MAX] = { false, false };
	static Vector3 simhands_smooth[HAND_INDEX_MAX][21];
	static bool simhands_smooth_ok[HAND_INDEX_MAX] = { false, false };
	static int simhands_held[HAND_INDEX_MAX] = { 0, 0 };
	static float simhands_maxfinger[HAND_INDEX_MAX] = { 0.0f, 0.0f };

	for (uint32_t h = 0; h < frame.hand_count && h < 2; h++) {
		const SimHandsPodHand &pod = frame.hands[h];
		if (!pod.present) {
			continue;
		}
		const int side = pod.side;
		if (side < 0 || side >= HAND_INDEX_MAX || filled[side]) {
			continue;
		}

		// --- Off-frame gating: hold each landmark's last IN-FRAME wrist-relative shape ---------------
		// When a fingertip leaves the webcam frame MediaPipe extrapolates it to normalized x/y OUTSIDE
		// [0,1]; mapped to metric that stretches the finger into a tendril. The hand-landmarker JSON has
		// no usable per-landmark visibility, but off-frame IS directly detectable from the coords. For
		// any off-frame (or non-finite) landmark, reuse its last in-frame wrist-relative offset so the
		// finger keeps its last good shape. This bounds the WHOLE finger (not per-bone), is immune to
		// foreshortening, and can never trigger on an in-frame curl -- unlike the reverted clamps
		// (1a129ec collapsed toward the parent; 993d2c7 capped per-bone but 4 chained bones still
		// drooped ~0.5 m cumulatively).
		auto off_frame = [](const SimHandsLandmark &lm) -> bool {
			const float M = 0.04f; // margin; extrapolated off-frame points land well outside [0,1]
			return !(Math::is_finite(lm.x) && Math::is_finite(lm.y) && Math::is_finite(lm.z)) ||
					lm.x < -M || lm.x > 1.0f + M || lm.y < -M || lm.y > 1.0f + M;
		};

		// Wrist anchor (absolute u) -- held if the wrist itself clips, so placement does not jump.
		Vector3 u0;
		if (!off_frame(pod.lm[0])) {
			u0 = u_of(pod.lm[0]);
			simhands_wrist_hold[side] = u0;
			simhands_wrist_ok[side] = true;
		} else if (simhands_wrist_ok[side]) {
			u0 = simhands_wrist_hold[side];
		} else {
			u0 = u_of(pod.lm[0]); // first frames, wrist already off -- nothing better yet
		}

		// Per-landmark wrist-relative u, with hold. li runs 0..20 and SIMHANDS_PARENT[li] < li, so a
		// parent's urel is always ready when a never-yet-valid child falls back onto it.
		Vector3 urel[21];
		int held = 0;
		for (int li = 0; li < 21; li++) {
			if (li == 0) {
				urel[0] = Vector3();
				continue;
			}
			if (!off_frame(pod.lm[li])) {
				urel[li] = u_of(pod.lm[li]) - u0;
				simhands_uhold[side][li] = urel[li];
				simhands_uok[side][li] = true;
			} else if (simhands_uok[side][li]) {
				urel[li] = simhands_uhold[side][li]; // hold last in-frame shape
				held++;
			} else {
				urel[li] = urel[SIMHANDS_PARENT[li]]; // never seen valid -> sit on the parent (no tendril)
				held++;
			}
		}

		// Self-normalize by the (now off-frame-robust) wrist->middle-knuckle xy span -> metric hand.
		const float ref = Vector3(urel[9].x, urel[9].y, 0.0f).length();
		if (ref < 1e-4f) {
			continue; // degenerate hand -- skip
		}
		// Foreshortening guard: when the hand points toward the camera, the xy wrist->knuckle span
		// (ref) collapses, so KNUCKLE/ref balloons and the whole hand (z especially) inflates into
		// tendrils even with every landmark in-frame (observed: held=0 maxfinger=0.42 m). Floor ref so
		// shape_scale can't blow up past ~1.6x nominal (nominal ref ~0.23 face-on; floor bites <0.14).
		const float shape_scale = SIMHANDS_HAND_KNUCKLE_M / MAX(ref, 0.14f);

		// Hand-scale → travel: a bigger hand should travel proportionally FARTHER for the same real-hand
		// move. The u0-driven x/y placement (how far the wrist roams across the scene as the real hand
		// crosses the webcam frame) scales with hand size relative to the nominal 0.09 m knuckle, so 2x
		// hand scale → 2x travel (3 cm real → 6 cm in-game). The hand's internal size already scales via
		// shape_scale (also knuckle-based), so the whole hand grows AND ranges together. Depth is a fixed
		// standoff (weak monocular z carries no travel), so only x/y placement scales.
		const float travel_scale = SIMHANDS_HAND_KNUCKLE_M / SIMHANDS_DEF_KNUCKLE_M;
		const Vector3 wrist_place(
				u0.x * SIMHANDS_PLANE_M * travel_scale,
				u0.y * SIMHANDS_PLANE_M * travel_scale + SIMHANDS_Y_OFFSET_M,
				-SIMHANDS_DEPTH_M);

		reset_hand_state((HandIndex)side);
		HandInteractionState &state = hand_interaction_states[side];
		state.tracked = true;
		state.has_joint_data = true;

		// Metric joint positions (gated urel -> scaled -> placed; +head_delta = WASD-follow).
		Vector3 jpos[21];
		for (int li = 0; li < 21; li++) {
			Vector3 rel = urel[li] * shape_scale;
			rel.z *= SIMHANDS_Z_SHAPE_GAIN;
			jpos[li] = wrist_place + rel + head_delta;
		}

		// --- Cumulative finger-length cap (the direct, cause-agnostic tendril bound) -----------------
		// A per-bone cap can't bound a finger (it's 4 chained bones); foreshortening, off-frame
		// extrapolation, or z-noise can each blow the wrist->tip total to 2-5x normal. Cap each
		// finger's TOTAL length and scale its (original) bone vectors proportionally toward the wrist,
		// preserving the finger's shape/direction -- never collapses (wrist + bone ratios kept). Normal
		// wrist->tip ~0.18-0.22 m, so the cap only bites on pathological frames.
		{
			static const int chains[5][5] = { { 0, 1, 2, 3, 4 }, { 0, 5, 6, 7, 8 }, { 0, 9, 10, 11, 12 }, { 0, 13, 14, 15, 16 }, { 0, 17, 18, 19, 20 } };
			const float max_finger = SIMHANDS_HAND_KNUCKLE_M * 2.7f; // ~0.24 m
			for (int c = 0; c < 5; c++) {
				Vector3 bone[5];
				float total = 0.0f;
				for (int k = 1; k < 5; k++) {
					bone[k] = jpos[chains[c][k]] - jpos[chains[c][k - 1]];
					total += bone[k].length();
				}
				if (total > max_finger && total > 1e-6f) {
					const float s = max_finger / total;
					for (int k = 1; k < 5; k++) {
						jpos[chains[c][k]] = jpos[chains[c][k - 1]] + bone[k] * s; // parent already shrunk; use original bone
					}
				}
			}
		}

		// --- Smoothing: velocity-adaptive EMA (one-euro-lite) --------------------------------------
		// Raw MediaPipe is jittery at rest. Blend each joint toward its running value with an alpha
		// that rises with the joint's frame-to-frame speed: still -> heavy smoothing (kills jitter),
		// moving fast -> near pass-through (no lag for grabs). Seeded on the first frame per side.
		if (simhands_smooth_ok[side]) {
			// Map the slider s in [0,3] to an EMA. 0..1 is UNCHANGED (floor 1.0 -> 0.15, speed gain 12 so
			// grabs stay low-lag). Past 1 it keeps getting smoother for slow, deliberate promo capture: the
			// floor falls 0.15 -> 0.03 AND the speed gain falls 12 -> 2, so even FAST motion is damped (a
			// glassy glide). lo/gain are per-hand (joint-independent) -> compute once.
			const float s = CLAMP(SIMHANDS_SMOOTHING, 0.0f, 3.0f);
			const float lo = (s <= 1.0f) ? (1.0f - 0.85f * s) // 0->1.0 (raw), 1->0.15 (heavy)
										 : MAX(0.15f - 0.06f * (s - 1.0f), 0.03f); // 1->0.15, 3->0.03 (ultra)
			const float gain = (s <= 1.0f) ? 12.0f : MAX(12.0f - 5.0f * (s - 1.0f), 2.0f); // 1->12, 3->2 (damp fast)
			for (int li = 0; li < 21; li++) {
				const Vector3 prev = simhands_smooth[side][li];
				const float speed = (jpos[li] - prev).length();
				const float alpha = CLAMP(lo + speed * gain, lo, 1.0f);
				jpos[li] = prev.lerp(jpos[li], alpha);
			}
		}
		for (int li = 0; li < 21; li++) {
			simhands_smooth[side][li] = jpos[li];
		}
		simhands_smooth_ok[side] = true;

		// Diagnostic: max CUMULATIVE finger length (wrist->tip) -- the metric that actually catches a
		// tendril (a per-bone max does NOT: 4 chained bones each "under cap" still droop ~0.5 m).
		// Normal ~0.18-0.22 m; a tendril is much larger.
		{
			static const int chains[5][5] = { { 0, 1, 2, 3, 4 }, { 0, 5, 6, 7, 8 }, { 0, 9, 10, 11, 12 }, { 0, 13, 14, 15, 16 }, { 0, 17, 18, 19, 20 } };
			float maxf = 0.0f;
			for (int c = 0; c < 5; c++) {
				float len = 0.0f;
				for (int k = 1; k < 5; k++) {
					len += jpos[chains[c][k]].distance_to(jpos[chains[c][k - 1]]);
				}
				maxf = MAX(maxf, len);
			}
			simhands_maxfinger[side] = maxf;
		}
		simhands_held[side] = held;

		// Palm normal (roll reference for every joint basis): wrist→index-knuckle × wrist→pinky-knuckle.
		Vector3 palm_normal = (jpos[5] - jpos[0]).cross(jpos[17] - jpos[0]);
		if (palm_normal.length() < 1e-5f) {
			palm_normal = Vector3(0.0f, 0.0f, 1.0f);
		}
		palm_normal = palm_normal.normalized();
		// The left hand is the mirror of the right, so the (index×pinky) palm normal comes out with
		// the opposite sign — which inverts the left mesh. Flip it so both hands feed the rig the same
		// basis chirality (the mesh driver's per-hand _bone_correction handles the rig mirror).
		if (side == HAND_INDEX_LEFT) {
			palm_normal = -palm_normal;
		}

		// --- Per-finger roll reference (fixes the mesh going inside-out on curl) --------------------
		// simhands_basis_from_dir resolves each bone's roll from a reference "up". Using the single
		// palm normal for EVERY joint flips the basis when a finger bone rotates parallel to it -- which
		// is exactly what curling a finger does -> the skin winds inside-out. A finger instead curls in
		// a stable PLANE whose normal stays perpendicular to its bones; use that as the roll reference.
		// It degenerates only when the finger is straight, where we fall back to the palm normal
		// (identical to before -> open/flat hands are unchanged). Sign-aligned to palm_normal so both
		// hands stay consistent with the chirality handling above.
		auto finger_roll = [&](int a, int b, int c) -> Vector3 {
			Vector3 n = (jpos[b] - jpos[a]).cross(jpos[c] - jpos[b]);
			if (n.length() < 1e-5f) {
				return palm_normal; // straight finger -> no plane -> palm normal (unchanged behavior)
			}
			n = n.normalized();
			return (n.dot(palm_normal) < 0.0f) ? -n : n;
		};
		Vector3 roll_ref[21];
		roll_ref[0] = palm_normal; // wrist
		const Vector3 r_thumb = finger_roll(1, 2, 3);
		const Vector3 r_index = finger_roll(5, 6, 7);
		const Vector3 r_middle = finger_roll(9, 10, 11);
		const Vector3 r_ring = finger_roll(13, 14, 15);
		const Vector3 r_pinky = finger_roll(17, 18, 19);
		for (int li = 1; li <= 4; li++) {
			roll_ref[li] = r_thumb;
		}
		for (int li = 5; li <= 8; li++) {
			roll_ref[li] = r_index;
		}
		for (int li = 9; li <= 12; li++) {
			roll_ref[li] = r_middle;
		}
		for (int li = 13; li <= 16; li++) {
			roll_ref[li] = r_ring;
		}
		for (int li = 17; li <= 20; li++) {
			roll_ref[li] = r_pinky;
		}

		// Pass 2: write each joint with position + synthesized orientation (+Y toward its child),
		// anchored to the head (so the hand sits in front of the view, not pinned to the origin).
		for (int li = 0; li < 21; li++) {
			const Vector3 distal = jpos[SIMHANDS_DIR_TO[li]] - jpos[SIMHANDS_DIR_FROM[li]];
			HandJointState &js = state.joints[SIMHANDS_JOINT_MAP[li]];
			js.tracked = true;
			js.transform = Transform3D(simhands_basis_from_dir(distal, roll_ref[li]), jpos[li]);
			js.radius = 0.01f;
		}

		// Palm = midpoint(wrist, middle knuckle), oriented up the hand; drives the "default" pose.
		HandJointState &palm = state.joints[XRHandTracker::HAND_JOINT_PALM];
		palm.tracked = true;
		palm.transform = Transform3D(simhands_basis_from_dir(jpos[9] - jpos[0], palm_normal), (jpos[0] + jpos[9]) * 0.5f);
		palm.radius = 0.01f;
		state.palm_transform = palm.transform;
		state.default_transform = palm.transform;
		state.aim_transform = palm.transform;
		state.grip_transform = state.joints[XRHandTracker::HAND_JOINT_WRIST].transform;

		// Finger metacarpals (Godot 6/11/16/21) have no MediaPipe landmark, so without posing them
		// the palm bones stay at rest while wrist+knuckles move → the skin under the fingers stretches.
		// Synthesize each: a point partway up the palm from the wrist toward that finger's knuckle
		// (MCP), oriented toward the knuckle. (mediapipe wrist=0; index/middle/ring/pinky MCP=5/9/13/17.)
		const XRHandTracker::HandJoint meta_joint[4] = {
			XRHandTracker::HAND_JOINT_INDEX_FINGER_METACARPAL,
			XRHandTracker::HAND_JOINT_MIDDLE_FINGER_METACARPAL,
			XRHandTracker::HAND_JOINT_RING_FINGER_METACARPAL,
			XRHandTracker::HAND_JOINT_PINKY_FINGER_METACARPAL,
		};
		const int meta_knuckle_lm[4] = { 5, 9, 13, 17 };
		for (int m = 0; m < 4; m++) {
			const Vector3 knuckle = jpos[meta_knuckle_lm[m]];
			HandJointState &mj = state.joints[meta_joint[m]];
			mj.tracked = true;
			mj.transform = Transform3D(simhands_basis_from_dir(knuckle - jpos[0], palm_normal), jpos[0].lerp(knuckle, 0.25f));
			mj.radius = 0.01f;
		}

		// Pinch (thumb tip ↔ index tip) → controller-tracker trigger proxy.
		const Vector3 thumb = state.joints[XRHandTracker::HAND_JOINT_THUMB_TIP].transform.origin;
		const Vector3 index = state.joints[XRHandTracker::HAND_JOINT_INDEX_FINGER_TIP].transform.origin;
		const float pinch_dist = thumb.distance_to(index);
		state.pinch_value = CLAMP((SIMHANDS_PINCH_ANALOG_MAX_M - pinch_dist) / (SIMHANDS_PINCH_ANALOG_MAX_M - SIMHANDS_PINCH_PRESS_M), 0.0f, 1.0f);
		state.pinch_click = pinch_dist < SIMHANDS_PINCH_PRESS_M;
		state.grasp_value = state.pinch_click ? 1.0f : state.pinch_value;
		state.grasp_click = state.pinch_click;

		filled[side] = true;
	}

	// Any side without SimHands data this frame → ensure it reads untracked.
	for (int i = 0; i < HAND_INDEX_MAX; i++) {
		if (!filled[i]) {
			reset_hand_state((HandIndex)i);
		}
	}

	// Throttled diagnostic (~every 60 frames). print_line does not surface in
	// the sim; os_log does — `log stream --predicate 'subsystem ==
	// "com.agilelens.godotvisionpilot"'`.
	static uint32_t log_counter = 0;
	if ((log_counter++ % 60) == 0) {
		for (int i = 0; i < HAND_INDEX_MAX; i++) {
			const HandInteractionState &s = hand_interaction_states[i];
			if (!s.tracked) {
				continue;
			}
			const Vector3 w = s.joints[XRHandTracker::HAND_JOINT_WRIST].transform.origin;
			const float d = s.joints[XRHandTracker::HAND_JOINT_THUMB_TIP].transform.origin.distance_to(
					s.joints[XRHandTracker::HAND_JOINT_INDEX_FINGER_TIP].transform.origin);
			os_log(simhands_log(), "[SimHands] %{public}s tracked wrist=(%.2f, %.2f, %.2f) thumb-index=%.3fm pinch=%d maxfinger=%.3fm held=%d",
					i == HAND_INDEX_LEFT ? "left" : "right", w.x, w.y, w.z, d, s.pinch_click ? 1 : 0, simhands_maxfinger[i], simhands_held[i]);
		}
	}
}

#else // !TARGET_OS_SIMULATOR — device: complete no-ops (ARKit hands are real there).

void VisionOSXRInterface::simhands_initialize() {}
void VisionOSXRInterface::simhands_uninitialize() {}
void VisionOSXRInterface::apply_simhands_hand_states() {}

#endif // TARGET_OS_SIMULATOR

#endif // VISIONOS_ENABLED
